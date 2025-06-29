#ifndef COMM_H
#define COMM_H

/**
 * @file comm.hpp
 * @date 2025-06-29
 * @author George Ngigi (jetsup)
 * @version 0.0.1
 * @brief Communication functions for the Micro-ROS rover project. This file
 * will handle all communication to and from rover through ROS2. It will include
 * functions to publish and subscribe to topics, as well as any other
 * communication-related functions.
 */

#include <geometry_msgs/msg/vector3.h>
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <stdio.h>

#include "config.h"
// TODO: Work on custom message types
// #include "uros_rover_message/msg/uros_rover_data.h"

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && \
    !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

// extern vehicle_data_t vehicle_data;
// Publishers
extern rcl_publisher_t magPublisher;
extern rcl_publisher_t gyroPublisher;
extern rcl_publisher_t accelPublisher;
extern rcl_publisher_t proximityPublisher;
// Publisher's data
extern geometry_msgs__msg__Vector3 vehicle_mag_data;
extern geometry_msgs__msg__Vector3 vehicle_gyro_data;
extern geometry_msgs__msg__Vector3 vehicle_accel_data;
extern geometry_msgs__msg__Vector3 vehicleProximity;

// Subscribers
extern rcl_subscription_t leftMotorControlSubscriber;
extern rcl_subscription_t rightMotorControlSubscriber;
extern rcl_subscription_t vehicleControlSubscriber; /* lights and hooting */
// Subscription's buffer
extern geometry_msgs__msg__Vector3 leftMotorControlData;
extern geometry_msgs__msg__Vector3 rightMotorControlData;
extern geometry_msgs__msg__Vector3 vehicleControlData; /* lights and hooting */

extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

void error_loop();
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

void leftMotorSubscriptionCallback(const void *msg);
void rightMotorSubscriptionCallback(const void *msg);
void vehicleControlSubscriptionCallback(const void *msg);

#endif
