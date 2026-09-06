#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

#include "hardware.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

namespace {

volatile int32_t encoder_counts[kWheelCount] = {0, 0, 0, 0};
float simulated_counts[kWheelCount] = {0.0F, 0.0F, 0.0F, 0.0F};
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensor_value;
ImuSample imu_sample_cache = {{0.0F, 0.0F, 0.0F},
                              {0.0F, 0.0F, 0.0F},
                              {0.0F, 0.0F, 0.0F, 1.0F}, false};
bool imu_initialized = false;

const uint8_t kMotorPwmPins[kWheelCount] = {PA8, PB4, PB5, PB9};
const uint8_t kMotorDirectionPins[kWheelCount] = {PC0, PC1, PC2, PC3};
const uint8_t kEncoderAPins[kWheelCount] = {PB0, PC6, PC8, PC10};
const uint8_t kEncoderBPins[kWheelCount] = {PB1, PC7, PC9, PC11};
const int8_t kEncoderSigns[kWheelCount] = {1, 1, 1, 1};
const int8_t kMotorSigns[kWheelCount] = {1, 1, 1, 1};
const uint8_t kEstopPin = PD0;
const uint8_t kStatusLedPin = LED_BUILTIN;
const uint8_t kMotorFaultPin = PE0;
const uint8_t kBuzzerPin = PD12;

static FirmwareSettings s_nvram_settings;
static bool s_nvram_saved = false;

void update_encoder_0() {
    encoder_counts[0] += digitalRead(kEncoderBPins[0]) ? 1 : -1;
}

void update_encoder_1() {
    encoder_counts[1] += digitalRead(kEncoderBPins[1]) ? 1 : -1;
}

void update_encoder_2() {
    encoder_counts[2] += digitalRead(kEncoderBPins[2]) ? 1 : -1;
}

void update_encoder_3() {
    encoder_counts[3] += digitalRead(kEncoderBPins[3]) ? 1 : -1;
}

}  // namespace

namespace RobotHardware {

void initialize() {
    pinMode(kEstopPin, INPUT_PULLUP);
    pinMode(kStatusLedPin, OUTPUT);
    pinMode(kMotorFaultPin, INPUT_PULLUP);
    pinMode(kBuzzerPin, OUTPUT);
    digitalWrite(kBuzzerPin, LOW);
    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        pinMode(kMotorDirectionPins[index], OUTPUT);
        pinMode(kMotorPwmPins[index], OUTPUT);
        analogWrite(kMotorPwmPins[index], 0);
        pinMode(kEncoderAPins[index], INPUT_PULLUP);
        pinMode(kEncoderBPins[index], INPUT_PULLUP);
    }
    Wire.begin();
    Serial.begin(115200);
#ifndef STM32_RENODE_SIM
    attachInterrupt(digitalPinToInterrupt(kEncoderAPins[0]), update_encoder_0,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncoderAPins[1]), update_encoder_1,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncoderAPins[2]), update_encoder_2,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncoderAPins[3]), update_encoder_3,
                    CHANGE);
#endif
}

bool initialize_imu() {
#ifdef STM32_RENODE_SIM
    imu_initialized = true;
    return true;
#else
    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire)) {
        imu_initialized = false;
        return false;
    }
    imu_initialized =
        bno08x.enableReport(SH2_LINEAR_ACCELERATION, kImuPeriodMs * 1000U) &&
        bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,
                            kImuPeriodMs * 1000U) &&
        bno08x.enableReport(SH2_ROTATION_VECTOR, kImuPeriodMs * 1000U);
    return imu_initialized;
#endif
}

bool imu_is_initialized() {
    return imu_initialized;
}

bool read_imu(ImuSample &sample) {
#ifdef STM32_RENODE_SIM
    memset(&sample, 0, sizeof(sample));
    sample.quaternion_xyzw[3] = 1.0F;
    sample.valid = true;
    return true;
#else
    if (!imu_initialized) {
        return false;
    }
    if (bno08x.wasReset()) {
        if (!initialize_imu()) {
            return false;
        }
    }
    if (!bno08x.getSensorEvent(&sensor_value)) {
        return false;
    }
    if (sensor_value.sensorId == SH2_LINEAR_ACCELERATION) {
        imu_sample_cache.linear_accel_mps2[0] =
            sensor_value.un.linearAcceleration.x;
        imu_sample_cache.linear_accel_mps2[1] =
            sensor_value.un.linearAcceleration.y;
        imu_sample_cache.linear_accel_mps2[2] =
            sensor_value.un.linearAcceleration.z;
    } else if (sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        imu_sample_cache.gyro_rad_s[0] = sensor_value.un.gyroscope.x;
        imu_sample_cache.gyro_rad_s[1] = sensor_value.un.gyroscope.y;
        imu_sample_cache.gyro_rad_s[2] = sensor_value.un.gyroscope.z;
    } else if (sensor_value.sensorId == SH2_ROTATION_VECTOR) {
        imu_sample_cache.quaternion_xyzw[0] =
            sensor_value.un.rotationVector.i;
        imu_sample_cache.quaternion_xyzw[1] =
            sensor_value.un.rotationVector.j;
        imu_sample_cache.quaternion_xyzw[2] =
            sensor_value.un.rotationVector.k;
        imu_sample_cache.quaternion_xyzw[3] =
            sensor_value.un.rotationVector.real;
    } else {
        sample = imu_sample_cache;
        return imu_sample_cache.valid;
    }
    imu_sample_cache.valid = true;
    sample = imu_sample_cache;
    return true;
#endif
}

int32_t read_encoder_count(uint8_t wheel_index) {
    if (wheel_index >= kWheelCount) {
        return 0;
    }
    taskENTER_CRITICAL();
    const int32_t value = encoder_counts[wheel_index];
    taskEXIT_CRITICAL();
    return value * kEncoderSigns[wheel_index];
}

void update_simulation(const float target_wheel_speed_rad_s[kWheelCount],
                       float delta_sec) {
#ifdef STM32_RENODE_SIM
    if (delta_sec <= 0.0F) {
        return;
    }
    const float counts_per_rad =
        static_cast<float>(kEncoderCountsPerRevolution) / 6.28318530718F;
    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        simulated_counts[index] += target_wheel_speed_rad_s[index] *
            counts_per_rad * delta_sec;
        encoder_counts[index] = static_cast<int32_t>(simulated_counts[index]);
    }
#else
    (void)target_wheel_speed_rad_s;
    (void)delta_sec;
#endif
}

void set_motor_output(uint8_t wheel_index, float normalized_output) {
    if (wheel_index >= kWheelCount) {
        return;
    }
    const float output = fmaxf(-1.0F, fminf(1.0F, normalized_output));
    const bool forward = output * kMotorSigns[wheel_index] >= 0.0F;
    digitalWrite(kMotorDirectionPins[wheel_index], forward ? HIGH : LOW);
    analogWrite(kMotorPwmPins[wheel_index],
                static_cast<int>(fabsf(output) * 255.0F));
}

bool read_estop() {
#ifdef STM32_RENODE_SIM
    return false;
#else
    return digitalRead(kEstopPin) == LOW;
#endif
}

bool read_motor_fault() {
#ifdef STM32_RENODE_SIM
    return false;
#else
    return digitalRead(kMotorFaultPin) == LOW;
#endif
}


void init_watchdog() {
#if !defined(STM32_RENODE_SIM) && defined(IWDG)
    IWDG->KR = 0x5555;
    IWDG->PR = 0x04;
    IWDG->RLR = 0x0FFF;
    IWDG->KR = 0xCCCC;
#endif
}

void feed_watchdog() {
#if !defined(STM32_RENODE_SIM) && defined(IWDG)
    IWDG->KR = 0xAAAA;
#endif
}

void set_status_indicators(bool estop, bool fault, bool warning) {
    if (estop || fault) {
        digitalWrite(kStatusLedPin, HIGH);
#ifndef STM32_RENODE_SIM
        digitalWrite(kBuzzerPin, HIGH);
#endif
    } else {
        digitalWrite(kStatusLedPin, warning ? HIGH : LOW);
#ifndef STM32_RENODE_SIM
        digitalWrite(kBuzzerPin, LOW);
#endif
    }
}

bool save_settings_nvram(const FirmwareSettings &settings) {
    s_nvram_settings = settings;
    s_nvram_saved = true;
    return true;
}

bool load_settings_nvram(FirmwareSettings &settings) {
    if (!s_nvram_saved) {
        return false;
    }
    settings = s_nvram_settings;
    return true;
}

uint32_t now_ms() {
    return static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
}

}  // namespace RobotHardware