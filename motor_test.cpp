#include <gpiod.h>
#include <iostream> 
#include <fstream> 
#include <thread> 
#include <chrono> 
#include <string>
using namespace std;
const char* CHIP = "/dev/gpiochip0";
const int IN1 = 23; const int IN2 = 24;
const string PWM_PATH = "/sys/class/pwm/pwmchip0/pwm0/";
gpiod_chip* chip = nullptr; gpiod_line_request* request = nullptr;
bool writePWM(const string& file, int value) { ofstream out(PWM_PATH + file);
if (!out.is_open()) {
    cerr << "Cannot open PWM " << file << endl;
    return false;
}

out << value;
out.flush();
return out.good();
}
bool setSpeed(int duty) { if (duty < 0) duty = 0; if (duty > 100) duty = 100;
int duty_ns = duty * 10000;

return writePWM("duty_cycle", duty_ns);
}
bool setDirection(int in1, int in2) { gpiod_line_value values[2] = { in1 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE, in2 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE };
unsigned int offsets[2] = {IN1, IN2};

return gpiod_line_request_set_values_subset(
    request, 2, offsets, values
) == 0;
}
void stopMotor() { setSpeed(0); }
void waitSeconds(int seconds) { this_thread::sleep_for(chrono::seconds(seconds)); }
bool forward() { if (!setSpeed(0)) return false; return setDirection(1, 0); }
bool reverse() { if (!setSpeed(0)) return false; return setDirection(0, 1); }
int main() { chip = gpiod_chip_open(CHIP);
if (!chip) {
    cerr << "Cannot open GPIO chip!" << endl;
    return 1;
}

gpiod_line_settings* settings =
    gpiod_line_settings_new();

gpiod_line_config* lineConfig =
    gpiod_line_config_new();

gpiod_request_config* requestConfig =
    gpiod_request_config_new();

if (!settings || !lineConfig || !requestConfig) {
    cerr << "GPIO configuration allocation failed!" << endl;
    return 1;
}

gpiod_line_settings_set_direction(
    settings, GPIOD_LINE_DIRECTION_OUTPUT
);

gpiod_line_settings_set_output_value(
    settings, GPIOD_LINE_VALUE_INACTIVE
);

unsigned int offsets[2] = {IN1, IN2};

gpiod_line_config_add_line_settings(
    lineConfig, offsets, 2, settings
);

gpiod_request_config_set_consumer(
    requestConfig, "motor_control"
);

request = gpiod_chip_request_lines(
    chip, requestConfig, lineConfig
);

gpiod_line_settings_free(settings);
gpiod_line_config_free(lineConfig);
gpiod_request_config_free(requestConfig);

if (!request) {
    cerr << "Cannot request GPIO lines!" << endl;
    gpiod_chip_close(chip);
    return 1;
}

// Start with PWM at zero
if (!setSpeed(0)) {
    cerr << "PWM initialization failed!" << endl;
    gpiod_line_request_release(request);
    gpiod_chip_close(chip);
    return 1;
}

cout << "Motor control initialized." << endl;

// TEST 1: Forward
cout << "Forward rotation: 100%" << endl;

if (!forward() || !setSpeed(100)) goto cleanup;

waitSeconds(2);
stopMotor();
waitSeconds(2);

// TEST 2: Reverse
cout << "Reverse rotation: 100%" << endl;

if (!reverse() || !setSpeed(100)) goto cleanup;

waitSeconds(2);
stopMotor();
waitSeconds(2);

// TEST 3: Variable PWM
cout << "Testing PWM speed control..." << endl;

if (!forward()) goto cleanup;

for (int duty = 20; duty <= 100; duty += 20) {
    cout << "PWM duty: " << duty << "%" << endl;

    if (!setSpeed(duty)) goto cleanup;

    waitSeconds(3);
}
cleanup: stopMotor(); setDirection(0, 0);
cout << "Motor stopped. Test finished." << endl;

gpiod_line_request_release(request);
gpiod_chip_close(chip);

return 0;
}
