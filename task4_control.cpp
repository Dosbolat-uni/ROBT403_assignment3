#include <lgpio.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "IMUSensor.h"

using namespace exploringRPi;
using Clock = std::chrono::steady_clock;

// ---------------------------------------------------------------- hardware --
constexpr int PIN_IN1 = 23;
constexpr int PIN_IN2 = 24;
constexpr int ENC_A   = 17;
constexpr int ENC_B   = 27;
const std::string PWM_CHIP = "/sys/class/pwm/pwmchip0/";
const std::string PWM_DIR  = PWM_CHIP + "pwm0/";
constexpr int PWM_PERIOD_NS = 1000000;                        // 1 kHz

// Pololu: 48 CPR already counts both edges of both channels (= x4 decoding).
// Exact gear ratio 22*22*22*23 / (12*10*10*10) = 20.4087
constexpr double CPR = 48.0 * (22.0 * 22 * 22 * 23) / (12.0 * 10 * 10 * 10);  // 979.6

// Sign conventions - check once with the motor free:
// ENCODER_SIGN: in the demo the count went UP while the command was negative.
constexpr int ENCODER_SIGN = -1;
// IMU_SIGN: flip if tilting the board "positive" makes the motor turn the wrong way.
constexpr int IMU_SIGN = +1;

// ------------------------------------------------------------ control loop --
constexpr double LOOP_HZ = 100.0;
constexpr double DT      = 1.0 / LOOP_HZ;                     // 10 ms

// Zone thresholds with hysteresis (deg)
constexpr double Z1_ENTER = 9.0,  Z1_EXIT = 11.0;            // |ref| < 10 -> stop
constexpr double Z3_ENTER = 31.0, Z3_EXIT = 29.0;            // |ref| > 30 -> follow

// PID (command in % duty, error in deg). Tuned for +-20 deg steps; override with --kp/--ki/--kd.
double KP = 2.8;          // %/deg
double KI = 0.5;          // %/(deg*s)  
double KD = 0.15;         // %*s/deg  
constexpr double I_LIMIT    = 30.0;   // max |KI * integral| contribution, % duty
constexpr double D_FILTER   = 0.7;    // derivative low-pass: 0 = none, 0.9 = heavy
constexpr double MIN_DUTY   = 0.0;    // dead-zone compensation, %. Keep 0: at 100 Hz even 10% overshoots
                                      // the deadband and the motor hunts. The integral removes the offset instead.
constexpr double DEADBAND   = 0.5;    // deg; inside this the PID output is zero
constexpr double DUTY_MAX   = 100.0;  // saturation, %

// Zone 3 "follow": track the moving IMU angle = P + damping + velocity feed-forward
constexpr double KF  = 4.0;           // %/deg
constexpr double KDF = 0.15;          // %*s/deg, damping on motor speed
constexpr double KV  = 100.0 / 2880;  // %/(deg/s): 100% duty ~ 480 rpm = 2880 deg/s
constexpr double REF_V_FILTER = 0.8;  // low-pass on the IMU angular rate

// ------------------------------------------------------------------ state --
std::atomic<bool> g_stop{false};
int g_chip = -1;

void onSignal(int) { g_stop = true; }                         // only set a flag; cleanup in main

// ------------------------------------------------------------------ PWM ----
bool writeSysfs(const std::string& path, const std::string& value) {
    std::ofstream f(path);
    if (!f) return false;
    f << value;
    f.flush();
    return f.good();
}

bool initPWM() {
    if (!std::ifstream(PWM_DIR + "duty_cycle")) {
        if (!writeSysfs(PWM_CHIP + "export", "0")) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    writeSysfs(PWM_DIR + "duty_cycle", "0");                   // duty must be <= period before period is set
    return writeSysfs(PWM_DIR + "period", std::to_string(PWM_PERIOD_NS)) &&
           writeSysfs(PWM_DIR + "enable", "1");
}

void setDuty(double percent) {
    percent = std::clamp(percent, 0.0, 100.0);
    writeSysfs(PWM_DIR + "duty_cycle", std::to_string(std::lround(percent * PWM_PERIOD_NS / 100.0)));
}

void stopMotor() {
    setDuty(0.0);
    lgGpioWrite(g_chip, PIN_IN1, 0);
    lgGpioWrite(g_chip, PIN_IN2, 0);
}

// u in [-100, 100] % : DIR = sign(u), duty = |u|
void driveMotor(double u) {
    if (u == 0.0) { stopMotor(); return; }
    lgGpioWrite(g_chip, PIN_IN1, u > 0 ? 1 : 0);
    lgGpioWrite(g_chip, PIN_IN2, u > 0 ? 0 : 1);
    setDuty(std::fabs(u));
}

// --------------------------------------------------------------- encoder ---
struct Encoder {
    std::atomic<std::int64_t> count{0};
    int state = 0;
    int chipNum = -1;

    static void events(int n, lgGpioAlert_p alerts, void* data) {
        auto& e = *static_cast<Encoder*>(data);
        static constexpr int delta[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
        for (int i = 0; i < n; ++i) {
            const auto& r = alerts[i].report;
            if (r.chip != e.chipNum || r.flags || r.level > 1) continue;
            int next = e.state;
            if (r.gpio == ENC_A)      next = (next & 1) | (r.level << 1);
            else if (r.gpio == ENC_B) next = (next & 2) | r.level;
            else continue;
            e.count.fetch_add(delta[(e.state << 2) | next]);
            e.state = next;
        }
    }
    double angleDeg() const { return ENCODER_SIGN * 360.0 * count.load() / CPR; }
};

// --------------------------------------------------------------- helpers ---
double pitchDeg(const Quaternion& q) {
    double s = std::clamp(2.0 * (q.q0 * q.q2 - q.q3 * q.q1), -1.0, 1.0);
    return std::asin(s) * 180.0 / M_PI;
}

int openRp1Chip(int& chipNum) {
    for (int c = 0; c < 32; ++c) {
        int h = lgGpiochipOpen(c);
        if (h < 0) continue;
        lgChipInfo_t info{};
        if (lgGpioGetChipInfo(h, &info) >= 0 && std::strstr(info.label, "rp1")) {
            chipNum = c;
            return h;
        }
        lgGpiochipClose(h);
    }
    return -1;
}

// ------------------------------------------------------------------- PID ---
struct PID {
    double integral = 0, dFilt = 0, prevMeas = 0;
    bool fresh = true;

    void reset(double meas) { integral = 0; dFilt = 0; prevMeas = meas; fresh = true; }

    double update(double ref, double meas) {
        if (fresh) { prevMeas = meas; fresh = false; }        // bumpless entry: no derivative kick
        double e = ref - meas;
        if (std::fabs(e) < DEADBAND) e = 0.0;

        // derivative on measurement (not on error) + first-order low-pass
        double dMeas = (meas - prevMeas) / DT;
        prevMeas = meas;
        dFilt = D_FILTER * dFilt + (1.0 - D_FILTER) * dMeas;

        double uPD = KP * e - KD * dFilt;
        double uI  = KI * integral;
        double u   = uPD + uI;

        // anti-windup: integrate only if not pushing further into saturation
        bool sat = std::fabs(u) >= DUTY_MAX;
        if (!sat || (u > 0) != (e > 0)) {
            integral += e * DT;
            if (KI > 0) integral = std::clamp(integral, -I_LIMIT / KI, I_LIMIT / KI);
        }
        if (e == 0.0) return 0.0;

        // dead-zone compensation: small commands would not move the gearbox
        u += (u > 0 ? MIN_DUTY : -MIN_DUTY);
        return std::clamp(u, -DUTY_MAX, DUTY_MAX);
    }
};

// ------------------------------------------------------- step-test metrics -
struct Segment { double t0, t1, target; };

void reportStep(const std::vector<double>& t, const std::vector<double>& ref,
                const std::vector<double>& th, const std::vector<Segment>& segs) {
    std::cout << "\n  target | start | peak    | overshoot | settle(s) | ss error\n"
              << "  -------+-------+---------+-----------+-----------+---------\n";
    for (const auto& s : segs) {
        if (s.target == 0.0) continue;                        // only report the +A / -A steps
        double start = 0, peak = 0, ss = 0; int nss = 0; bool first = true;
        double settle = -1, lastOut = s.t0;
        double band = std::max(1.0, 0.05 * std::fabs(s.target));
        for (size_t i = 0; i < t.size(); ++i) {
            if (t[i] < s.t0 || t[i] >= s.t1) continue;
            if (first) { start = th[i]; peak = th[i]; first = false; }
            double dir = (s.target > start) ? 1.0 : -1.0;
            if (dir * th[i] > dir * peak) peak = th[i];
            if (std::fabs(th[i] - s.target) > band) lastOut = t[i];
            if (t[i] > s.t1 - 0.5) { ss += th[i] - s.target; ++nss; }
        }
        if (first) continue;
        double stepSize = s.target - start;
        double os = std::fabs(stepSize) > 1e-6 ? 100.0 * (peak - s.target) / stepSize : 0.0;
        settle = lastOut - s.t0;
        std::printf("  %+6.1f | %+5.1f | %+7.2f | %8.1f%% | %9.2f | %+7.2f\n",
                    s.target, start, peak, os, settle, nss ? ss / nss : 0.0);
    }
    std::cout << "  (overshoot < 0 means it stopped short = undershoot; settle band = max(1 deg, 5%))\n";
    (void)ref;
}

// ------------------------------------------------------------------ main ---
int main(int argc, char** argv) {
    double stepAmp = 0.0;
    std::string logName = "task4_log.csv";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](double& v) { if (i + 1 < argc) v = std::atof(argv[++i]); };
        if (a == "--step") next(stepAmp);
        else if (a == "--kp") next(KP);
        else if (a == "--ki") next(KI);
        else if (a == "--kd") next(KD);
        else if (a == "--log" && i + 1 < argc) logName = argv[++i];
        else { std::cerr << "unknown option " << a << "\n"; return 1; }
    }
    const bool stepMode = stepAmp != 0.0;
    if (stepMode && logName == "task4_log.csv")
        logName = "step_" + std::to_string((int)std::lround(stepAmp)) + ".csv";

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // --- GPIO (direction pins + encoder) on the RP1 chip -------------------
    static Encoder enc;
    g_chip = openRp1Chip(enc.chipNum);
    if (g_chip < 0) { std::cerr << "RP1 gpiochip not found (run with sudo?)\n"; return 1; }
    if (lgGpioClaimOutput(g_chip, 0, PIN_IN1, 0) < 0 || lgGpioClaimOutput(g_chip, 0, PIN_IN2, 0) < 0 ||
        lgGpioClaimAlert(g_chip, 0, LG_BOTH_EDGES, ENC_A, -1) < 0 ||
        lgGpioClaimAlert(g_chip, 0, LG_BOTH_EDGES, ENC_B, -1) < 0) {
        std::cerr << "Could not claim GPIO 17/23/24/27 (another program using them?)\n";
        lgGpiochipClose(g_chip);
        return 1;
    }
    enc.state = (lgGpioRead(g_chip, ENC_A) << 1) | lgGpioRead(g_chip, ENC_B);
    lgGpioSetSamplesFunc(Encoder::events, &enc);

    // --- PWM -----------------------------------------------------------------
    if (!initPWM()) { std::cerr << "PWM init failed (check dtoverlay=pwm on GPIO12)\n"; g_stop = true; }
    stopMotor();

    // --- IMU + zero reference -------------------------------------------------
    IMUSensor imu(1, 0x6B);
    double imuZero = 0.0;
    if (!g_stop && !stepMode) {
        if (imu.initSensors() != 0) { std::cerr << "IMU init failed\n"; g_stop = true; }
        else {
            std::cout << "Settling orientation filter - keep the IMU level and still (3 s)...\n";
            beta = 2.0f;                                       // fast convergence from the identity quaternion
            double sum = 0; int n = 0;
            auto next = Clock::now();
            for (int i = 0; i < 300 && !g_stop; ++i) {
                if (i == 200) beta = 0.1f;                     // back to normal gain for the last second
                double p = pitchDeg(imu.updateAndGetQuaternion());
                if (i >= 250) { sum += p; ++n; }
                next += std::chrono::microseconds(10000);
                std::this_thread::sleep_until(next);
            }
            imuZero = n ? sum / n : 0.0;
            std::cout << "IMU zero = " << std::fixed << std::setprecision(2) << imuZero << " deg\n";
        }
    }
    enc.count = 0;                                             // motor's current position = 0 deg

    std::ofstream log(logName);
    log << "time_s,mode,theta_ref_deg,theta_motor_deg,error_deg,u_percent,counts\n";
    std::cout << (stepMode ? "STEP TEST" : "THREE-ZONE MODE") << "  Kp=" << KP << " Ki=" << KI
              << " Kd=" << KD << "  log -> " << logName << "  (Ctrl+C to stop)\n"
              << "time_s, mode, ref, motor, error, u\n";

    // step schedule: 0 for 1 s, +A for 4 s, 0 for 3 s, -A for 4 s, 0 for 3 s
    std::vector<Segment> segs;
    if (stepMode) {
        double tt = 0;
        for (auto [dur, tgt] : std::vector<std::pair<double, double>>{
                 {1, 0}, {4, stepAmp}, {3, 0}, {4, -stepAmp}, {3, 0}}) {
            segs.push_back({tt, tt + dur, tgt});
            tt += dur;
        }
    }
    std::vector<double> vt, vref, vth;

    PID pid;
    double refVel = 0, motorVel = 0, prevRef = 0, prevMotor = 0;
    enum Zone { STOP, PIDZ, FOLLOW } zone = STOP;
    const char* names[] = {"ZONE1_STOP", "ZONE2_PID", "ZONE3_FOLLOW"};

    const auto t0 = Clock::now();
    auto nextTick = t0;
    long k = 0;
    while (!g_stop) {
        nextTick += std::chrono::microseconds(10000);
        const double t = std::chrono::duration<double>(Clock::now() - t0).count();
        const double thMotor = enc.angleDeg();
        double ref = 0, u = 0;
        const char* mode;

        if (stepMode) {
            if (t >= segs.back().t1) break;
            for (const auto& s : segs) if (t >= s.t0 && t < s.t1) ref = s.target;
            u = pid.update(ref, thMotor);
            mode = "STEP_PID";
            vt.push_back(t); vref.push_back(ref); vth.push_back(thMotor);
        } else {
            ref = IMU_SIGN * (pitchDeg(imu.updateAndGetQuaternion()) - imuZero);
            const double a = std::fabs(ref);

            // zone selection with hysteresis
            Zone nz = zone;
            switch (zone) {
                case STOP:   if (a > Z1_EXIT) nz = (a > Z3_ENTER) ? FOLLOW : PIDZ; break;
                case PIDZ:   if (a < Z1_ENTER) nz = STOP; else if (a > Z3_ENTER) nz = FOLLOW; break;
                case FOLLOW: if (a < Z3_EXIT) nz = (a < Z1_ENTER) ? STOP : PIDZ; break;
            }
            if (nz != zone && nz == PIDZ) pid.reset(thMotor);  // bumpless transfer into PID
            zone = nz;

            const double e = ref - thMotor;
            refVel   = REF_V_FILTER * refVel + (1 - REF_V_FILTER) * (ref - prevRef) / DT;
            motorVel = 0.7 * motorVel + 0.3 * (thMotor - prevMotor) / DT;
            prevRef = ref; prevMotor = thMotor;
            if (zone == STOP) {
                u = 0.0;                                       // stop where it is
            } else if (zone == PIDZ) {
                u = pid.update(ref, thMotor);
            } else {                                           // follow the rotation
                u = KF * e - KDF * motorVel + KV * refVel;
                if (std::fabs(e) < DEADBAND && std::fabs(refVel) < 2.0) u = 0.0;
                u = std::clamp(u, -DUTY_MAX, DUTY_MAX);
            }
            mode = names[zone];
        }

        driveMotor(u);

        log << std::fixed << std::setprecision(3) << t << ',' << mode << ',' << ref << ',' << thMotor
            << ',' << ref - thMotor << ',' << u << ',' << enc.count.load() << '\n';
        if (k++ % 10 == 0)                                     // terminal at 10 Hz, file at 100 Hz
            std::cout << std::fixed << std::setprecision(2) << t << ", " << mode << ", " << ref << ", "
                      << thMotor << ", " << ref - thMotor << ", " << u << std::endl;

        if (Clock::now() > nextTick) nextTick = Clock::now();
        std::this_thread::sleep_until(nextTick);
    }

    // --- shutdown ----------------------------------------------------------------
    stopMotor();
    std::cout << "\nMotor stopped. Log saved to " << logName << "\n";
    if (stepMode && !vt.empty()) reportStep(vt, vref, vth, segs);
    lgGpioSetSamplesFunc(nullptr, nullptr);
    for (int p : {PIN_IN1, PIN_IN2, ENC_A, ENC_B}) lgGpioFree(g_chip, p);
    lgGpiochipClose(g_chip);
    return 0;
}
