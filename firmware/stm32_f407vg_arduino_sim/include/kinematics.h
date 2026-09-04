#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "firmware_config.h"

bool compute_wheel_speeds(const TwistCommand &twist,
                          const FirmwareSettings &settings,
                          float wheel_speeds[kWheelCount]);

bool compute_body_twist(const float wheel_speeds[kWheelCount],
                        const FirmwareSettings &settings,
                        TwistCommand &twist);

float wrap_angle(float angle_rad);

#endif