#ifndef HARDWARE_H
#define HARDWARE_H

#include <stdint.h>

#include "firmware_config.h"

namespace RobotHardware {

void initialize();
bool initialize_imu();
bool imu_is_initialized();
bool read_imu(ImuSample &sample);
int32_t read_encoder_count(uint8_t wheel_index);
void update_simulation(const float target_wheel_speed_rad_s[kWheelCount],
                       float delta_sec);
void set_motor_output(uint8_t wheel_index, float normalized_output);
bool read_estop();
bool read_motor_fault();
void init_watchdog();
void feed_watchdog();
void set_status_indicators(bool estop, bool fault, bool warning);
bool save_settings_nvram(const FirmwareSettings &settings);
bool load_settings_nvram(FirmwareSettings &settings);
uint32_t now_ms();

}  // namespace RobotHardware

#endif