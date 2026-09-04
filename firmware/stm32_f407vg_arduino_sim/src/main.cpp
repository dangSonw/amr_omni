#include <Arduino.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <micro_ros_platformio.h>

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "firmware_config.h"
#include "hardware.h"
#include "kalman.h"
#include "kinematics.h"
#include "pid.h"
#include "robot_state.h"

namespace {

const char *const kNodeName = "omni_stm32_f407vg";
const char *const kFrameId = "odom";
const char *const kChildFrameId = "base_link";
const char *const kImuFrameId = "imu_link";
const uint8_t kDiagnosticKeyCount = 2U;
const uint8_t kDiagnosticMessageCapacity = 64U;

RobotState robot_state;
SemaphoreHandle_t state_mutex = nullptr;

rcl_allocator_t ros_allocator;
rclc_support_t ros_support;
rcl_node_t ros_node;
rclc_executor_t ros_executor;

rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t estop_subscriber;
rcl_publisher_t odometry_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t wheel_state_publisher;
rcl_publisher_t encoder_counts_publisher;
rcl_publisher_t diagnostics_publisher;
rcl_publisher_t status_publisher;

geometry_msgs__msg__Twist cmd_vel_message;
std_msgs__msg__Bool estop_message;
nav_msgs__msg__Odometry odometry_message;
sensor_msgs__msg__Imu imu_message;
std_msgs__msg__Float32MultiArray wheel_state_message;
std_msgs__msg__Int32MultiArray encoder_counts_message;
diagnostic_msgs__msg__DiagnosticArray diagnostics_message;
std_msgs__msg__String status_message;

float wheel_state_values[kWheelCount * 3U];
int32_t encoder_count_values[kWheelCount];
diagnostic_msgs__msg__DiagnosticStatus diagnostic_statuses[1];
diagnostic_msgs__msg__KeyValue diagnostic_values[kDiagnosticKeyCount];
char diagnostic_key_storage[kDiagnosticKeyCount][24];
char diagnostic_value_storage[kDiagnosticKeyCount]
    [kDiagnosticMessageCapacity];
char odometry_frame_storage[16];
char odometry_child_frame_storage[16];
char imu_frame_storage[16];
char diagnostics_frame_storage[1];
char diagnostic_name_storage[32];
char diagnostic_message_storage[kDiagnosticMessageCapacity];
char diagnostic_hardware_storage[32];
char status_storage[kDiagnosticMessageCapacity];
std_msgs__msg__MultiArrayDimension wheel_state_dimensions[1];

ScalarKalman wheel_filters[kWheelCount] = {
    ScalarKalman(0.5F, 0.04F), ScalarKalman(0.5F, 0.04F),
    ScalarKalman(0.5F, 0.04F), ScalarKalman(0.5F, 0.04F),
};
ScalarKalman acceleration_filters[3] = {
    ScalarKalman(0.2F, 0.1F), ScalarKalman(0.2F, 0.1F),
    ScalarKalman(0.2F, 0.1F),
};
ScalarKalman gyro_filters[3] = {
    ScalarKalman(0.2F, 0.1F), ScalarKalman(0.2F, 0.1F),
    ScalarKalman(0.2F, 0.1F),
};
ScalarKalman quaternion_filters[4] = {
    ScalarKalman(0.05F, 0.02F), ScalarKalman(0.05F, 0.02F),
    ScalarKalman(0.05F, 0.02F), ScalarKalman(0.05F, 0.02F),
};
ScalarKalman body_velocity_filters[3] = {
    ScalarKalman(0.2F, 0.04F), ScalarKalman(0.2F, 0.04F),
    ScalarKalman(0.2F, 0.04F),
};
WheelSpeedPid wheel_speed_controllers[kWheelCount] = {
    WheelSpeedPid(kDefaultMotorKp, kDefaultMotorKi, kDefaultMotorKd, 1.0F),
    WheelSpeedPid(kDefaultMotorKp, kDefaultMotorKi, kDefaultMotorKd, 1.0F),
    WheelSpeedPid(kDefaultMotorKp, kDefaultMotorKi, kDefaultMotorKd, 1.0F),
    WheelSpeedPid(kDefaultMotorKp, kDefaultMotorKi, kDefaultMotorKd, 1.0F),
};

extern "C" void freertos_tick_handler(void);

extern "C" void osSystickHandler(void) {
    freertos_tick_handler();
}

float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

void stop_motors() {
    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        RobotHardware::set_motor_output(index, 0.0F);
    }
}

extern "C" void vApplicationMallocFailedHook(void) {
    stop_motors();
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

bool copy_state(RobotState &destination) {
    if (state_mutex == nullptr ||
        xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) {
        return false;
    }
    destination = robot_state;
    xSemaphoreGive(state_mutex);
    return true;
}

void update_state(const RobotState &source) {
    if (state_mutex == nullptr ||
        xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) {
        return;
    }
    robot_state = source;
    xSemaphoreGive(state_mutex);
}

bool finite_twist(const geometry_msgs__msg__Twist &message,
                  TwistCommand &twist) {
    twist.vx_mps = static_cast<float>(message.linear.x);
    twist.vy_mps = static_cast<float>(message.linear.y);
    twist.wz_rad_s = static_cast<float>(message.angular.z);
    return isfinite(twist.vx_mps) != 0 && isfinite(twist.vy_mps) != 0 &&
           isfinite(twist.wz_rad_s) != 0 &&
           fabsf(twist.vx_mps) <= kMaxLinearSpeedMps &&
           fabsf(twist.vy_mps) <= kMaxLinearSpeedMps &&
           fabsf(twist.wz_rad_s) <= kMaxAngularSpeedRadS;
}

void set_string(rosidl_runtime_c__String &string, char *storage,
                size_t capacity, const char *value) {
    const size_t length = strlen(value);
    const size_t copied = length < capacity - 1U ? length : capacity - 1U;
    if (value != storage) {
        memcpy(storage, value, copied);
    }
    storage[copied] = '\0';
    string.data = storage;
    string.size = copied;
    string.capacity = capacity;
}

void stamp_message(std_msgs__msg__Header &header, char *frame_storage,
                   size_t frame_capacity, const char *frame_id) {
    const uint32_t milliseconds = RobotHardware::now_ms();
    header.stamp.sec = static_cast<int32_t>(milliseconds / 1000U);
    header.stamp.nanosec = (milliseconds % 1000U) * 1000000U;
    set_string(header.frame_id, frame_storage, frame_capacity, frame_id);
}

void publish_message(rcl_publisher_t &publisher, const void *message) {
    const rcl_ret_t result = rcl_publish(&publisher, message, nullptr);
    (void)result;
}

void command_callback(const void *message) {
    const geometry_msgs__msg__Twist *received =
        static_cast<const geometry_msgs__msg__Twist *>(message);
    RobotState state;
    TwistCommand twist;
    if (!finite_twist(*received, twist)) {
        if (copy_state(state)) {
            state.command_valid = false;
            update_state(state);
        }
        return;
    }
    if (!copy_state(state)) {
        return;
    }
    state.command_twist = twist;
    state.command_valid = !state.estop_active;
    state.last_command_ms = RobotHardware::now_ms();
    update_state(state);
}

void estop_callback(const void *message) {
    const std_msgs__msg__Bool *received =
        static_cast<const std_msgs__msg__Bool *>(message);
    RobotState state;
    if (!copy_state(state)) {
        return;
    }
    state.estop_active = received->data;
    state.command_valid = false;
    update_state(state);
}

bool initialize_ros_message_memory() {
    memset(&cmd_vel_message, 0, sizeof(cmd_vel_message));
    memset(&estop_message, 0, sizeof(estop_message));
    memset(&odometry_message, 0, sizeof(odometry_message));
    memset(&imu_message, 0, sizeof(imu_message));
    memset(&wheel_state_message, 0, sizeof(wheel_state_message));
    memset(&encoder_counts_message, 0, sizeof(encoder_counts_message));
    memset(&diagnostics_message, 0, sizeof(diagnostics_message));
    memset(&status_message, 0, sizeof(status_message));

    wheel_state_message.layout.dim.data = wheel_state_dimensions;
    wheel_state_message.layout.dim.size = 0U;
    wheel_state_message.layout.dim.capacity = 1U;
    wheel_state_message.layout.data_offset = 0U;
    wheel_state_message.data.data = wheel_state_values;
    wheel_state_message.data.size = kWheelCount * 3U;
    wheel_state_message.data.capacity = kWheelCount * 3U;
    encoder_counts_message.data.data = encoder_count_values;
    encoder_counts_message.data.size = kWheelCount;
    encoder_counts_message.data.capacity = kWheelCount;

    diagnostics_message.status.data = diagnostic_statuses;
    diagnostics_message.status.size = 1U;
    diagnostics_message.status.capacity = 1U;
    diagnostic_statuses[0].values.data = diagnostic_values;
    diagnostic_statuses[0].values.size = 0U;
    diagnostic_statuses[0].values.capacity = kDiagnosticKeyCount;
    for (uint8_t index = 0U; index < kDiagnosticKeyCount; ++index) {
        diagnostic_values[index].key.data = diagnostic_key_storage[index];
        diagnostic_values[index].key.size = 0U;
        diagnostic_values[index].key.capacity =
            sizeof(diagnostic_key_storage[index]);
        diagnostic_values[index].value.data = diagnostic_value_storage[index];
        diagnostic_values[index].value.size = 0U;
        diagnostic_values[index].value.capacity =
            sizeof(diagnostic_value_storage[index]);
    }
    set_string(odometry_message.header.frame_id, odometry_frame_storage,
               sizeof(odometry_frame_storage), kFrameId);
    set_string(odometry_message.child_frame_id, odometry_child_frame_storage,
               sizeof(odometry_child_frame_storage), kChildFrameId);
    set_string(imu_message.header.frame_id, imu_frame_storage,
               sizeof(imu_frame_storage), kImuFrameId);
    set_string(diagnostics_message.header.frame_id, diagnostics_frame_storage,
               sizeof(diagnostics_frame_storage), "");
    set_string(diagnostic_statuses[0].name, diagnostic_name_storage,
               sizeof(diagnostic_name_storage), "stm32_f407vg");
    set_string(diagnostic_statuses[0].message, diagnostic_message_storage,
               sizeof(diagnostic_message_storage), "starting");
    set_string(diagnostic_statuses[0].hardware_id, diagnostic_hardware_storage,
               sizeof(diagnostic_hardware_storage), "stm32f407vg");
    set_string(status_message.data, status_storage, sizeof(status_storage),
               "starting");
    return true;
}

bool initialize_ros_entities() {
    ros_allocator = rcl_get_default_allocator();
    if (rclc_support_init(&ros_support, 0, nullptr, &ros_allocator) !=
        RCL_RET_OK) {
        return false;
    }
    if (rclc_node_init_default(&ros_node, kNodeName, "", &ros_support) !=
        RCL_RET_OK) {
        return false;
    }
#define INIT_PUBLISHER(publisher, type_support, topic) \
    if (rclc_publisher_init_default( \
            &(publisher), &ros_node, (type_support), (topic)) != RCL_RET_OK) { \
        return false; \
    }
#define INIT_SUBSCRIBER(subscriber, type_support, topic) \
    if (rclc_subscription_init_default( \
            &(subscriber), &ros_node, (type_support), (topic)) != RCL_RET_OK) { \
        return false; \
    }
    INIT_SUBSCRIBER(cmd_vel_subscriber,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                    "stm32_cmd_vel");
    INIT_SUBSCRIBER(estop_subscriber,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "estop");
    INIT_PUBLISHER(odometry_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                   "odom");
    INIT_PUBLISHER(imu_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                   "imu/data_raw");
    INIT_PUBLISHER(wheel_state_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                   "wheel_state");
    INIT_PUBLISHER(encoder_counts_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                   "encoder_counts");
    INIT_PUBLISHER(diagnostics_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg,
                                               DiagnosticArray),
                   "diagnostics");
    INIT_PUBLISHER(status_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                   "status");
#undef INIT_PUBLISHER
#undef INIT_SUBSCRIBER
    if (rclc_executor_init(&ros_executor, &ros_support.context, 2U,
                           &ros_allocator) != RCL_RET_OK) {
        return false;
    }
    if (rclc_executor_add_subscription(&ros_executor, &cmd_vel_subscriber,
                                       &cmd_vel_message, &command_callback,
                                       ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_subscription(&ros_executor, &estop_subscriber,
                                       &estop_message, &estop_callback,
                                       ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }
    return true;
}

void fill_diagnostics(const RobotState &state, uint32_t now) {
    diagnostic_msgs__msg__DiagnosticStatus &status = diagnostic_statuses[0];
    status.level = state.estop_active || state.imu_fault ? 2U : 0U;
    set_string(status.name, diagnostic_name_storage,
               sizeof(diagnostic_name_storage), "stm32_f407vg");
    set_string(status.message, diagnostic_message_storage,
               sizeof(diagnostic_message_storage),
               state.estop_active ? "estop" :
               (state.imu_fault ? "imu fault" : "ok"));
    set_string(status.hardware_id, diagnostic_hardware_storage,
               sizeof(diagnostic_hardware_storage), "stm32f407vg");
    status.values.size = kDiagnosticKeyCount;
    set_string(diagnostic_values[0].key, diagnostic_key_storage[0],
               sizeof(diagnostic_key_storage[0]), "command_age_ms");
    snprintf(diagnostic_values[0].value.data,
             sizeof(diagnostic_value_storage[0]), "%lu",
             static_cast<unsigned long>(now - state.last_command_ms));
    diagnostic_values[0].value.size = strlen(diagnostic_values[0].value.data);
    set_string(diagnostic_values[1].key, diagnostic_key_storage[1],
               sizeof(diagnostic_key_storage[1]), "imu_fault");
    snprintf(diagnostic_values[1].value.data,
             sizeof(diagnostic_value_storage[1]), "%u",
             state.imu_fault ? 1U : 0U);
    diagnostic_values[1].value.size = strlen(diagnostic_values[1].value.data);
}

void publish_telemetry(const RobotState &state) {
    stamp_message(odometry_message.header, odometry_frame_storage,
                  sizeof(odometry_frame_storage), kFrameId);
    set_string(odometry_message.child_frame_id, odometry_child_frame_storage,
               sizeof(odometry_child_frame_storage), kChildFrameId);
    odometry_message.pose.pose.position.x = state.pose.x_m;
    odometry_message.pose.pose.position.y = state.pose.y_m;
    odometry_message.pose.pose.orientation.z =
        sinf(state.pose.yaw_rad * 0.5F);
    odometry_message.pose.pose.orientation.w =
        cosf(state.pose.yaw_rad * 0.5F);
    odometry_message.twist.twist.linear.x = state.pose.vx_mps;
    odometry_message.twist.twist.linear.y = state.pose.vy_mps;
    odometry_message.twist.twist.angular.z = state.pose.wz_rad_s;
    publish_message(odometry_publisher, &odometry_message);

    stamp_message(imu_message.header, imu_frame_storage,
                  sizeof(imu_frame_storage), kImuFrameId);
    imu_message.linear_acceleration.x = state.imu.linear_accel_mps2[0];
    imu_message.linear_acceleration.y = state.imu.linear_accel_mps2[1];
    imu_message.linear_acceleration.z = state.imu.linear_accel_mps2[2];
    imu_message.angular_velocity.x = state.imu.gyro_rad_s[0];
    imu_message.angular_velocity.y = state.imu.gyro_rad_s[1];
    imu_message.angular_velocity.z = state.imu.gyro_rad_s[2];
    imu_message.orientation.x = state.imu.quaternion_xyzw[0];
    imu_message.orientation.y = state.imu.quaternion_xyzw[1];
    imu_message.orientation.z = state.imu.quaternion_xyzw[2];
    imu_message.orientation.w = state.imu.quaternion_xyzw[3];
    publish_message(imu_publisher, &imu_message);

    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        wheel_state_values[index] = state.measured_wheel_speed_rad_s[index];
        wheel_state_values[kWheelCount + index] =
            state.target_wheel_speed_rad_s[index];
        wheel_state_values[2U * kWheelCount + index] =
            state.motor_output[index];
    }
    publish_message(wheel_state_publisher, &wheel_state_message);

    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        encoder_count_values[index] = state.encoder_counts[index];
    }
    publish_message(encoder_counts_publisher, &encoder_counts_message);

    fill_diagnostics(state, RobotHardware::now_ms());
    stamp_message(diagnostics_message.header, diagnostics_frame_storage,
                  sizeof(diagnostics_frame_storage), "");
    publish_message(diagnostics_publisher, &diagnostics_message);

    snprintf(status_storage, sizeof(status_storage), "%s",
             state.estop_active ? "estop" :
             (state.imu_fault ? "imu_fault" : "ok"));
    set_string(status_message.data, status_storage, sizeof(status_storage),
               status_storage);
    publish_message(status_publisher, &status_message);
}

void micro_ros_task(void *) {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    vTaskDelay(pdMS_TO_TICKS(2000U));
    if (!initialize_ros_message_memory() || !initialize_ros_entities()) {
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }
    TickType_t wake_time = xTaskGetTickCount();
    uint32_t last_publish_ms = RobotHardware::now_ms();
    for (;;) {
        rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(2));
        const uint32_t now = RobotHardware::now_ms();
        if (now - last_publish_ms >= kTelemetryPeriodMs) {
            RobotState state;
            if (copy_state(state)) {
                publish_telemetry(state);
                state.last_telemetry_ms = now;
                update_state(state);
            }
            last_publish_ms = now;
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(2U));
    }
}

void control_task(void *) {
    TickType_t wake_time = xTaskGetTickCount();
    const float delta_sec = kControlPeriodMs * 0.001F;
    for (;;) {
        RobotState state;
        if (copy_state(state)) {
            const uint32_t now = RobotHardware::now_ms();
            const bool command_stale = now - state.last_command_ms >
                state.settings.command_timeout_ms;
            const bool zero_command = state.command_twist.vx_mps == 0.0F &&
                state.command_twist.vy_mps == 0.0F &&
                state.command_twist.wz_rad_s == 0.0F;
            const bool stop_requested = command_stale || state.estop_active ||
                !state.command_valid || zero_command;
            if (stop_requested) {
                memset(state.target_wheel_speed_rad_s, 0,
                       sizeof(state.target_wheel_speed_rad_s));
                for (uint8_t index = 0U; index < kWheelCount; ++index) {
                    wheel_speed_controllers[index].reset();
                    state.motor_output[index] = 0.0F;
                    RobotHardware::set_motor_output(index, 0.0F);
                }
            } else if (!compute_wheel_speeds(
                           state.command_twist, state.settings,
                           state.target_wheel_speed_rad_s)) {
                memset(state.target_wheel_speed_rad_s, 0,
                       sizeof(state.target_wheel_speed_rad_s));
                for (uint8_t index = 0U; index < kWheelCount; ++index) {
                    wheel_speed_controllers[index].reset();
                    state.motor_output[index] = 0.0F;
                    RobotHardware::set_motor_output(index, 0.0F);
                }
            } else {
                for (uint8_t index = 0U; index < kWheelCount; ++index) {
                    const float feedback = wheel_speed_controllers[index].update(
                        state.target_wheel_speed_rad_s[index],
                        state.measured_wheel_speed_rad_s[index], delta_sec);
                    const float feedforward =
                        state.target_wheel_speed_rad_s[index] /
                        state.settings.max_wheel_speed_rad_s;
                    state.motor_output[index] = clamp(
                        feedforward + feedback, -1.0F, 1.0F);
                    RobotHardware::set_motor_output(
                        index, state.motor_output[index]);
                }
            }
            if (command_stale) {
                state.command_valid = false;
            }
            update_state(state);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

void encoder_task(void *) {
    TickType_t wake_time = xTaskGetTickCount();
    int32_t previous_counts[kWheelCount] = {0, 0, 0, 0};
    for (;;) {
        RobotState state;
        if (copy_state(state)) {
            const uint32_t now = RobotHardware::now_ms();
            const float delta_sec = (now - state.last_encoder_ms) * 0.001F;
            const float safe_delta = delta_sec > 0.0F ? delta_sec : 0.01F;
            for (uint8_t index = 0U; index < kWheelCount; ++index) {
                const int32_t counts = RobotHardware::read_encoder_count(index);
                const int32_t delta_counts = counts - previous_counts[index];
                previous_counts[index] = counts;
                const float raw_speed = delta_counts * 6.28318530718F /
                    (static_cast<float>(kEncoderCountsPerRevolution) *
                     safe_delta);
                state.measured_wheel_speed_rad_s[index] =
                    wheel_filters[index].update(raw_speed, safe_delta);
                state.encoder_counts[index] = counts;
            }
            state.last_encoder_ms = now;
            RobotHardware::update_simulation(state.target_wheel_speed_rad_s,
                                             safe_delta);
            update_state(state);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kEncoderPeriodMs));
    }
}

void imu_task(void *) {
    TickType_t wake_time = xTaskGetTickCount();
    for (;;) {
        RobotState state;
        if (copy_state(state)) {
            ImuSample sample;
            if (RobotHardware::read_imu(sample)) {
                for (uint8_t index = 0U; index < 3U; ++index) {
                    sample.linear_accel_mps2[index] = acceleration_filters[index]
                        .update(sample.linear_accel_mps2[index],
                                kImuPeriodMs * 0.001F);
                    sample.gyro_rad_s[index] = gyro_filters[index].update(
                        sample.gyro_rad_s[index], kImuPeriodMs * 0.001F);
                }
                float quaternion_norm = 0.0F;
                for (uint8_t index = 0U; index < 4U; ++index) {
                    sample.quaternion_xyzw[index] = quaternion_filters[index]
                        .update(sample.quaternion_xyzw[index],
                                kImuPeriodMs * 0.001F);
                    quaternion_norm += sample.quaternion_xyzw[index] *
                        sample.quaternion_xyzw[index];
                }
                if (quaternion_norm > 0.01F) {
                    quaternion_norm = sqrtf(quaternion_norm);
                    for (uint8_t index = 0U; index < 4U; ++index) {
                        sample.quaternion_xyzw[index] /= quaternion_norm;
                    }
                }
                state.imu = sample;
                state.imu_fault = false;
            } else {
                state.imu_fault = true;
            }
            state.last_imu_ms = RobotHardware::now_ms();
            update_state(state);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kImuPeriodMs));
    }
}

void odometry_task(void *) {
    TickType_t wake_time = xTaskGetTickCount();
    const float delta_sec = kOdometryPeriodMs * 0.001F;
    for (;;) {
        RobotState state;
        if (copy_state(state)) {
            TwistCommand wheel_twist;
            if (compute_body_twist(state.measured_wheel_speed_rad_s,
                                   state.settings, wheel_twist)) {
                const float vx = body_velocity_filters[0].update(
                    wheel_twist.vx_mps, delta_sec);
                const float vy = body_velocity_filters[1].update(
                    wheel_twist.vy_mps, delta_sec);
                float wz = body_velocity_filters[2].update(
                    wheel_twist.wz_rad_s, delta_sec);
                if (!state.imu_fault && isfinite(state.imu.gyro_rad_s[2])) {
                    wz = body_velocity_filters[2].update(
                        state.imu.gyro_rad_s[2], delta_sec);
                }
                const float cos_yaw = cosf(state.pose.yaw_rad);
                const float sin_yaw = sinf(state.pose.yaw_rad);
                state.pose.x_m += (cos_yaw * vx - sin_yaw * vy) * delta_sec;
                state.pose.y_m += (sin_yaw * vx + cos_yaw * vy) * delta_sec;
                state.pose.yaw_rad = wrap_angle(
                    state.pose.yaw_rad + wz * delta_sec);
                state.pose.vx_mps = vx;
                state.pose.vy_mps = vy;
                state.pose.wz_rad_s = wz;
            }
            update_state(state);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kOdometryPeriodMs));
    }
}

void safety_task(void *) {
    for (;;) {
        RobotState state;
        if (copy_state(state)) {
            const uint32_t now = RobotHardware::now_ms();
            const bool physical_estop = RobotHardware::read_estop();
            const bool command_stale = now - state.last_command_ms >
                state.settings.command_timeout_ms;
            if (physical_estop || command_stale) {
                state.estop_active = state.estop_active || physical_estop;
                state.command_valid = false;
                stop_motors();
                update_state(state);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

}  // namespace

void setup() {
    RobotHardware::initialize();
    RobotHardware::initialize_imu();
    initialize_robot_state(robot_state);
    const uint32_t now = RobotHardware::now_ms();
    robot_state.last_command_ms = now;
    robot_state.last_encoder_ms = now;
    robot_state.last_imu_ms = now;
    robot_state.last_telemetry_ms = now;
    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == nullptr) {
        stop_motors();
        for (;;) {
        }
    }

    xTaskCreate(micro_ros_task, "micro_ros", 1024U, nullptr, 5U, nullptr);
    xTaskCreate(control_task, "control", 512U, nullptr, 4U, nullptr);
    xTaskCreate(encoder_task, "encoder", 384U, nullptr, 4U, nullptr);
    xTaskCreate(imu_task, "imu", 512U, nullptr, 3U, nullptr);
    xTaskCreate(odometry_task, "odom", 384U, nullptr, 3U, nullptr);
    xTaskCreate(safety_task, "safety", 320U, nullptr, 6U, nullptr);

    vTaskStartScheduler();
    stop_motors();
    for (;;) {
    }
}

void loop() {}