// Pi 5 Task 3 encoder test using lgpio. BCM A=17, B=27.
// Encoder powered at 5 V; A/B MUST pass through a 5 V -> 3.3 V level shifter.
// This program only claims encoder pins. Use your existing Task 2 program
// to run/stop the motor on its separate pins. Ctrl+C stops READING only.
// Build: g++ -std=c++17 -O2 -Wall -Wextra -pthread task3_encoder_test.cpp -llgpio -o task3_encoder_test
// Run: sudo ./task3_encoder_test
#include <lgpio.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

constexpr int A = 17, B = 27;
constexpr double PI = 3.14159265358979323846;
constexpr double CPR = 48.0 * (22.0*22*22*23)/(12.0*10*10*10); // 979.616
constexpr int ENCODER_SIGN = +1; // Change to -1 if needed after direction test.
volatile std::sig_atomic_t stop_requested = 0;
void onSignal(int) { stop_requested = 1; }
struct Encoder {
    std::atomic<std::int64_t> count{0};
    int state = 0;
    int chip = -1;
    static void events(int n, lgGpioAlert_p alerts, void* data) {
        auto& e = *static_cast<Encoder*>(data);
        static constexpr int delta[16] = {
            0,+1,-1,0, -1,0,0,+1, +1,0,0,-1, 0,-1,+1,0
        };
        // lgpio supplies timestamp-ordered edge reports in batches.
        for (int i=0; i<n; ++i) {
            const auto& r = alerts[i].report;
            if (r.chip != e.chip || r.flags || r.level > 1) continue;
            int next = e.state;
            if (r.gpio == A) next = (next & 1) | (r.level << 1);
            else if (r.gpio == B) next = (next & 2) | r.level;
            else continue;
            e.count.fetch_add(ENCODER_SIGN * delta[(e.state << 2) | next]);
            e.state = next;
        }
    }
};
int main() {
    int h = -1;
    static Encoder enc; // Remains alive through callback shutdown.
    // Find RP1 by label: gpiochip number varies with OS/kernel.
    for (int chip=0; chip<32; ++chip) {
        int candidate = lgGpiochipOpen(chip);
        if (candidate < 0) continue;
        lgChipInfo_t info{};
        if (lgGpioGetChipInfo(candidate, &info) >= 0 &&
            std::strstr(info.label, "rp1") != nullptr) {
            h = candidate;
            enc.chip = chip;
            std::cerr << "Using gpiochip" << chip << " (" << info.label << ")\n";
            break;
        }
        lgGpiochipClose(candidate);
    }
    if (h < 0) {
        std::cerr << "RP1 gpiochip not found/openable. Check permissions and gpiochip labels.\n";
        return 1;
    }
    bool claimed_a = false, claimed_b = false;
    int result = 0;
    int rc = lgGpioClaimAlert(h, 0, LG_BOTH_EDGES, A, -1);
    if (rc >= 0) {
        claimed_a = true;
        rc = lgGpioClaimAlert(h, 0, LG_BOTH_EDGES, B, -1);
        claimed_b = rc >= 0;
    }
    if (rc < 0) {
        std::cerr << "Could not claim encoder pins: " << lguErrorText(rc) << '\n';
        result = 1;
    } else {
        // Keep motor stationary during initialization.
        const int a = lgGpioRead(h, A), b = lgGpioRead(h, B);
        if (a < 0 || b < 0) {
            std::cerr << "Could not read encoder pins.\n";
            result = 1;
        } else {
            enc.state = (a << 1) | b;
            lgGpioSetSamplesFunc(Encoder::events, &enc);
            std::signal(SIGINT, onSignal);
            std::signal(SIGTERM, onSignal);
            std::cerr << "CPR=" << CPR << "; ready. Start motor with Task 2 program.\n"
                      << "Ctrl+C stops encoder reading only; stop motor through Task 2.\n";
            std::cout << "time_s,counts,angle_deg,rpm,omega_rad_s\n";
            using Clock = std::chrono::steady_clock;
            const auto start = Clock::now();
            auto prev_time = start, deadline = start;
            auto prev_count = enc.count.load();
            while (!stop_requested) {
                deadline += std::chrono::milliseconds(100);
                std::this_thread::sleep_until(deadline);
                const auto now = Clock::now();
                const auto count = enc.count.load();
                const double dt = std::chrono::duration<double>(now-prev_time).count();
                const double rev_s = (count-prev_count)/(CPR*dt);
                std::cout << std::fixed << std::setprecision(6)
                          << std::chrono::duration<double>(now-start).count() << ','
                          << count << ',' << 360.0*count/CPR << ','
                          << 60.0*rev_s << ',' << 2*PI*rev_s << std::endl;
                prev_time = now;
                prev_count = count;
                if (Clock::now() > deadline + std::chrono::milliseconds(100))
                    deadline = Clock::now();
            }
            lgGpioSetSamplesFunc(nullptr, nullptr);
        }
    }
    if (claimed_a) lgGpioFree(h, A);
    if (claimed_b) lgGpioFree(h, B);
    lgGpiochipClose(h);
    return result;
}
