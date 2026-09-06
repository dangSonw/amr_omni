#include <Arduino.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <micro_ros_platformio.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/header.h>
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
const uint8_t kStatusMessageCapacity = 64U;

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
rcl_publisher_t status_publisher;
rcl_publisher_t debug_publisher;

geometry_msgs__msg__Twist cmd_vel_message;
std_msgs__msg__Bool estop_message;
nav_msgs__msg__Odometry odometry_message;
sensor_msgs__msg__Imu imu_message;
std_msgs__msg__String status_message;
std_msgs__msg__String debug_message;

char odometry_frame_storage[16];
char odometry_child_frame_storage[16];
char imu_frame_storage[16];
char status_storage[kStatusMessageCapacity];
char debug_storage[384];

enum RosInitFlag : uint16_t {
    kInitSupport    = 1U << 0U,
    kInitNode       = 1U << 1U,
    kInitCmdVelSub  = 1U << 2U,
    kInitEstopSub   = 1U << 3U,
    kInitOdomPub    = 1U << 4U,
    kInitImuPub     = 1U << 5U,
    kInitStatusPub  = 1U << 6U,
    kInitDebugPub   = 1U << 7U,
    kInitExecutor   = 1U << 8U,
};
uint16_t ros_init_flags = 0U;

ScalarKalman wheel_filters[kWheelCount] = {
    ScalarKalman(0.5F, 0.04F), ScalarKalman(0.5F, 0.04F),
    ScalarKalman(0.5F, 0.04F), ScalarKalman(0.5F, 0.04F),
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

void update_command_fields(const TwistCommand &twist, bool valid, uint32_t command_ms, bool estop) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    robot_state.command_twist = twist;
    robot_state.command_valid = valid;
    robot_state.last_command_ms = command_ms;
    robot_state.estop_active = estop;
    xSemaphoreGive(state_mutex);
}

void update_encoder_fields(const float measured[kWheelCount], const float raw[kWheelCount], const int32_t counts[kWheelCount], uint32_t encoder_ms) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    memcpy(robot_state.measured_wheel_speed_rad_s, measured, sizeof(robot_state.measured_wheel_speed_rad_s));
    memcpy(robot_state.raw_wheel_speed_rad_s, raw, sizeof(robot_state.raw_wheel_speed_rad_s));
    memcpy(robot_state.encoder_counts, counts, sizeof(robot_state.encoder_counts));
    robot_state.last_encoder_ms = encoder_ms;
    xSemaphoreGive(state_mutex);
}

void update_control_fields(const float target[kWheelCount], const float output[kWheelCount], bool command_valid) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    memcpy(robot_state.target_wheel_speed_rad_s, target, sizeof(robot_state.target_wheel_speed_rad_s));
    memcpy(robot_state.motor_output, output, sizeof(robot_state.motor_output));
    robot_state.command_valid = command_valid;
    xSemaphoreGive(state_mutex);
}

void update_imu_fields(const ImuSample &sample, bool fault, uint32_t imu_ms) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    robot_state.imu = sample;
    robot_state.imu_fault = fault;
    robot_state.last_imu_ms = imu_ms;
    xSemaphoreGive(state_mutex);
}

void update_pose_fields(const PoseState &pose) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    robot_state.pose = pose;
    xSemaphoreGive(state_mutex);
}

void update_telemetry_ms(uint32_t ms) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    robot_state.last_telemetry_ms = ms;
    xSemaphoreGive(state_mutex);
}

void update_safety_fields(bool estop, bool motor_fault, bool command_valid) {
    if (state_mutex == nullptr || xSemaphoreTake(state_mutex, pdMS_TO_TICKS(2U)) != pdTRUE) return;
    robot_state.estop_active = estop;
    robot_state.motor_fault = motor_fault;
    robot_state.command_valid = command_valid;
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
            update_command_fields(state.command_twist, false, state.last_command_ms, state.estop_active);
        }
        return;
    }
    if (!copy_state(state)) {
        return;
    }
    update_command_fields(twist, !state.estop_active, RobotHardware::now_ms(), state.estop_active);
}

void estop_callback(const void *message) {
    const std_msgs__msg__Bool *received =
        static_cast<const std_msgs__msg__Bool *>(message);
    RobotState state;
    if (!copy_state(state)) {
        return;
    }
    update_safety_fields(received->data, state.motor_fault, false);
}

bool initialize_ros_message_memory() {
    memset(&cmd_vel_message, 0, sizeof(cmd_vel_message));
    memset(&estop_message, 0, sizeof(estop_message));
    memset(&odometry_message, 0, sizeof(odometry_message));
    memset(&imu_message, 0, sizeof(imu_message));
    memset(&status_message, 0, sizeof(status_message));

    set_string(odometry_message.header.frame_id, odometry_frame_storage,
               sizeof(odometry_frame_storage), kFrameId);
    set_string(odometry_message.child_frame_id, odometry_child_frame_storage,
               sizeof(odometry_child_frame_storage), kChildFrameId);
    set_string(imu_message.header.frame_id, imu_frame_storage,
               sizeof(imu_frame_storage), "imu_link");
    set_string(status_message.data, status_storage, sizeof(status_storage),
               "starting");
    return true;
}

bool initialize_ros_entities() {
    ros_init_flags = 0U;
    ros_allocator = rcl_get_default_allocator();
    if (rclc_support_init(&ros_support, 0, nullptr, &ros_allocator) !=
        RCL_RET_OK) {
        return false;
    }
    ros_init_flags |= kInitSupport;
    
    if (rclc_node_init_default(&ros_node, kNodeName, "", &ros_support) !=
        RCL_RET_OK) {
        return false;
    }
    ros_init_flags |= kInitNode;
    
#define INIT_PUBLISHER(publisher, type_support, topic, flag) \
    if (rclc_publisher_init_default( \
            &(publisher), &ros_node, (type_support), (topic)) != RCL_RET_OK) { \
        return false; \
    } else { ros_init_flags |= (flag); }
#define INIT_SUBSCRIBER(subscriber, type_support, topic, flag) \
    if (rclc_subscription_init_default( \
            &(subscriber), &ros_node, (type_support), (topic)) != RCL_RET_OK) { \
        return false; \
    } else { ros_init_flags |= (flag); }
    
    INIT_SUBSCRIBER(cmd_vel_subscriber,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                    "stm32_cmd_vel", kInitCmdVelSub);
    INIT_SUBSCRIBER(estop_subscriber,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "estop", kInitEstopSub);
    INIT_PUBLISHER(odometry_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                   "wheel/odom", kInitOdomPub);
    INIT_PUBLISHER(imu_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                   "imu/data", kInitImuPub);
    INIT_PUBLISHER(status_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                   "status", kInitStatusPub);
    INIT_PUBLISHER(debug_publisher,
                   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                   "debug/data", kInitDebugPub);
#undef INIT_PUBLISHER
#undef INIT_SUBSCRIBER
    
    if (rclc_executor_init(&ros_executor, &ros_support.context, 2U,
                           &ros_allocator) != RCL_RET_OK) {
        return false;
    }
    ros_init_flags |= kInitExecutor;
    
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

void publish_telemetry(const RobotState &state) {
    stamp_message(odometry_message.header, odometry_frame_storage,
                  sizeof(odometry_frame_storage), kFrameId);
    set_string(odometry_message.child_frame_id, odometry_child_frame_storage,
               sizeof(odometry_child_frame_storage), kChildFrameId);
    odometry_message.pose.pose.position.x = state.pose.x_m;
    odometry_message.pose.pose.position.y = state.pose.y_m;
    odometry_message.pose.pose.position.z = 0.0F;

    odometry_message.pose.pose.orientation.x = 0.0F;
    odometry_message.pose.pose.orientation.y = 0.0F;
    odometry_message.pose.pose.orientation.z = sinf(state.pose.yaw_rad * 0.5F);
    odometry_message.pose.pose.orientation.w = cosf(state.pose.yaw_rad * 0.5F);

    odometry_message.twist.twist.linear.x = state.pose.vx_mps;
    odometry_message.twist.twist.linear.y = state.pose.vy_mps;
    odometry_message.twist.twist.angular.z = state.pose.wz_rad_s;
    publish_message(odometry_publisher, &odometry_message);

    stamp_message(imu_message.header, imu_frame_storage,
                  sizeof(imu_frame_storage), "imu_link");
    imu_message.orientation.x = state.imu.quaternion_xyzw[0];
    imu_message.orientation.y = state.imu.quaternion_xyzw[1];
    imu_message.orientation.z = state.imu.quaternion_xyzw[2];
    imu_message.orientation.w = state.imu.quaternion_xyzw[3];
    imu_message.angular_velocity.x = state.imu.gyro_rad_s[0];
    imu_message.angular_velocity.y = state.imu.gyro_rad_s[1];
    imu_message.angular_velocity.z = state.imu.gyro_rad_s[2];
    imu_message.linear_acceleration.x = state.imu.linear_accel_mps2[0];
    imu_message.linear_acceleration.y = state.imu.linear_accel_mps2[1];
    imu_message.linear_acceleration.z = state.imu.linear_accel_mps2[2];
    publish_message(imu_publisher, &imu_message);

    snprintf(status_storage, sizeof(status_storage), "%s",
             state.estop_active ? "estop" :
             (state.imu_fault ? "imu_fault" : "ok"));
    set_string(status_message.data, status_storage, sizeof(status_storage),
               status_storage);
    publish_message(status_publisher, &status_message);

    snprintf(debug_storage, sizeof(debug_storage),
        "{\"raw_ws\":[%.3f,%.3f,%.3f,%.3f],"
        "\"filt_ws\":[%.3f,%.3f,%.3f,%.3f],"
        "\"tgt_ws\":[%.3f,%.3f,%.3f,%.3f],"
        "\"mot_out\":[%.3f,%.3f,%.3f,%.3f],"
        "\"imu_q\":[%.3f,%.3f,%.3f,%.3f],"
        "\"body_v\":[%.3f,%.3f,%.3f],"
        "\"cmd_v\":[%.3f,%.3f,%.3f]}",
        state.raw_wheel_speed_rad_s[0], state.raw_wheel_speed_rad_s[1], state.raw_wheel_speed_rad_s[2], state.raw_wheel_speed_rad_s[3],
        state.measured_wheel_speed_rad_s[0], state.measured_wheel_speed_rad_s[1], state.measured_wheel_speed_rad_s[2], state.measured_wheel_speed_rad_s[3],
        state.target_wheel_speed_rad_s[0], state.target_wheel_speed_rad_s[1], state.target_wheel_speed_rad_s[2], state.target_wheel_speed_rad_s[3],
        state.motor_output[0], state.motor_output[1], state.motor_output[2], state.motor_output[3],
        state.imu.quaternion_xyzw[0], state.imu.quaternion_xyzw[1], state.imu.quaternion_xyzw[2], state.imu.quaternion_xyzw[3],
        state.pose.vx_mps, state.pose.vy_mps, state.pose.wz_rad_s,
        state.command_twist.vx_mps, state.command_twist.vy_mps, state.command_twist.wz_rad_s);
    set_string(debug_message.data, debug_storage, sizeof(debug_storage), debug_storage);
    publish_message(debug_publisher, &debug_message);
}

void clean_ros_entities() {
    if (ros_init_flags & kInitExecutor) rclc_executor_fini(&ros_executor);
    if (ros_init_flags & kInitDebugPub) rcl_publisher_fini(&debug_publisher, &ros_node);
    if (ros_init_flags & kInitStatusPub) rcl_publisher_fini(&status_publisher, &ros_node);
    if (ros_init_flags & kInitImuPub) rcl_publisher_fini(&imu_publisher, &ros_node);
    if (ros_init_flags & kInitOdomPub) rcl_publisher_fini(&odometry_publisher, &ros_node);
    if (ros_init_flags & kInitEstopSub) rcl_subscription_fini(&estop_subscriber, &ros_node);
    if (ros_init_flags & kInitCmdVelSub) rcl_subscription_fini(&cmd_vel_subscriber, &ros_node);
    if (ros_init_flags & kInitNode) rcl_node_fini(&ros_node);
    if (ros_init_flags & kInitSupport) rclc_support_fini(&ros_support);
    ros_init_flags = 0U;
}

void micro_ros_task(void *) {
    set_microros_serial_transports(Serial);
    vTaskDelay(pdMS_TO_TICKS(2000U));
    for (;;) {
        while (!initialize_ros_message_memory() || !initialize_ros_entities()) {
            clean_ros_entities();
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
        TickType_t wake_time = xTaskGetTickCount();
        uint32_t last_publish_ms = RobotHardware::now_ms();
#if !defined(STM32_RENODE_SIM)
        uint32_t last_ping_ms = RobotHardware::now_ms();
        uint8_t ping_failures = 0U;
#endif
        for (;;) {
            rclc_executor_spin_some(&ros_executor, RCL_MS_TO_NS(2));
            const uint32_t now = RobotHardware::now_ms();
            if (now - last_publish_ms >= kTelemetryPeriodMs) {
                RobotState state;
                if (copy_state(state)) {
                    publish_telemetry(state);
                    update_telemetry_ms(now);
                }
                last_publish_ms = now;
            }
#if !defined(STM32_RENODE_SIM)
            if (now - last_ping_ms >= 1000U) {
                last_ping_ms = now;
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    ping_failures++;
                    if (ping_failures >= 3U) {
                        clean_ros_entities();
                        break;
                    }
                } else {
                    ping_failures = 0U;
                }
            }
#endif
            vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(2U));
        }
        vTaskDelay(pdMS_TO_TICKS(1000U));
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
            update_control_fields(state.target_wheel_speed_rad_s, state.motor_output, state.command_valid);
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
                state.raw_wheel_speed_rad_s[index] = raw_speed;
                state.measured_wheel_speed_rad_s[index] =
                    wheel_filters[index].update(raw_speed, safe_delta);
                state.encoder_counts[index] = counts;
            }
            RobotHardware::update_simulation(state.target_wheel_speed_rad_s,
                                             safe_delta);
            update_encoder_fields(state.measured_wheel_speed_rad_s, state.raw_wheel_speed_rad_s, state.encoder_counts, now);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kEncoderPeriodMs));
    }
}

void imu_task(void *) {
    TickType_t wake_time = xTaskGetTickCount();
    uint32_t imu_last_valid_ms = RobotHardware::now_ms();
    for (;;) {
        RobotState state;
        if (copy_state(state)) {
            ImuSample sample;
            bool fault = state.imu_fault;
            if (RobotHardware::read_imu(sample)) {
                float quaternion_norm = sample.quaternion_xyzw[0] * sample.quaternion_xyzw[0] +
                                        sample.quaternion_xyzw[1] * sample.quaternion_xyzw[1] +
                                        sample.quaternion_xyzw[2] * sample.quaternion_xyzw[2] +
                                        sample.quaternion_xyzw[3] * sample.quaternion_xyzw[3];
                if (quaternion_norm > 0.01F) {
                    quaternion_norm = sqrtf(quaternion_norm);
                    for (uint8_t index = 0U; index < 4U; ++index) {
                        sample.quaternion_xyzw[index] /= quaternion_norm;
                    }
                }
                imu_last_valid_ms = RobotHardware::now_ms();
                fault = false;
            } else {
                fault = (RobotHardware::now_ms() - imu_last_valid_ms) > 500U;
                sample = state.imu;  // keep previous
            }
            update_imu_fields(sample, fault, RobotHardware::now_ms());
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
                const float raw_wz = (!state.imu_fault && isfinite(state.imu.gyro_rad_s[2]))
                                         ? state.imu.gyro_rad_s[2]
                                         : wheel_twist.wz_rad_s;
                const float wz = body_velocity_filters[2].update(raw_wz, delta_sec);

                const float qx = state.imu.quaternion_xyzw[0];
                const float qy = state.imu.quaternion_xyzw[1];
                const float qz = state.imu.quaternion_xyzw[2];
                const float qw = state.imu.quaternion_xyzw[3];
                const float norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
                if (!state.imu_fault && norm_sq > 0.5F) {
                    const float siny_cosp = 2.0F * (qw * qz + qx * qy);
                    const float cosy_cosp = 1.0F - 2.0F * (qy * qy + qz * qz);
                    state.pose.yaw_rad = atan2f(siny_cosp, cosy_cosp);
                } else {
                    state.pose.yaw_rad = wrap_angle(
                        state.pose.yaw_rad + wz * delta_sec);
                }

                const float cos_yaw = cosf(state.pose.yaw_rad);
                const float sin_yaw = sinf(state.pose.yaw_rad);
                state.pose.x_m += (cos_yaw * vx - sin_yaw * vy) * delta_sec;
                state.pose.y_m += (sin_yaw * vx + cos_yaw * vy) * delta_sec;
                state.pose.vx_mps = vx;
                state.pose.vy_mps = vy;
                state.pose.wz_rad_s = wz;
            }
            update_pose_fields(state.pose);
        }
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(kOdometryPeriodMs));
    }
}

void safety_task(void *) {
    for (;;) {
        RobotHardware::feed_watchdog();
        RobotState state;
        if (copy_state(state)) {
            const uint32_t now = RobotHardware::now_ms();
            const bool physical_estop = RobotHardware::read_estop();
            const bool motor_fault = RobotHardware::read_motor_fault();
            const bool command_stale = now - state.last_command_ms >
                state.settings.command_timeout_ms;

            state.motor_fault = motor_fault;

            if (physical_estop || motor_fault || command_stale) {
                state.estop_active = state.estop_active || physical_estop ||
                    motor_fault;
                state.command_valid = false;
                stop_motors();
            }
            RobotHardware::set_status_indicators(
                state.estop_active, motor_fault, command_stale);
            update_safety_fields(state.estop_active, state.motor_fault, state.command_valid);
        }
        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

}  // namespace

void setup() {
    RobotHardware::initialize();
    RobotHardware::init_watchdog();
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