#ifndef CONFIG_H
#define CONFIG_H

/**
 * @file config.h
 * @date 2025-06-29
 * @author George Ngigi (jetsup)
 * @version 0.0.1
 * @brief Configuration file for the Micro-ROS rover project. All pin
 * definitions and settings should be placed here.
 */

#include "credentials.h"

// Pin definitions
#define UROS_LED_PIN 2

#define UROS_L298N_ENA_PIN 25
#define UROS_L298N_IN1_PIN 26
#define UROS_L298N_IN2_PIN 27
#define UROS_L298N_ENB_PIN 14
#define UROS_L298N_IN3_PIN 12
#define UROS_L298N_IN4_PIN 13

#define UROS_BUZZER_PIN 18
#define UROS_HEADLIGHT_PIN 4
#define UROS_TAIL_LIGHT_PIN 16
#define UROS_REVERSE_LIGHT_PIN 17
#define UROS_FRONT_PROXIMITY_TRIGGER_PIN 33
#define UROS_FRONT_PROXIMITY_ECHO_PIN 32

#define UROS_LEFT_MOTOR_ENCODER_A_PIN 34
#define UROS_LEFT_MOTOR_ENCODER_B_PIN 35

// I2C pins
#define UROS_I2C_SDA_PIN 21
#define UROS_I2C_SCL_PIN 22

// Motor speed thresholds
#define UROS_MOTOR_SPEED_MIN 50

// Proximity sensor settings
#define UROS_FRONT_PROXIMITY_MAX_DISTANCE_CM 400
#define UROS_PROXIMITY_MEASURE_INTERVAL_MS 10

// MPU6500 settings
#define UROS_MPU6500_ADDRESS 0x68

// Proximity Sensor Settings
#define UROS_PROXIMITY_MAX_DISTANCE_CM 400

// GLOBAL VARIABLES
extern int leftMotorReceivedControl[3];
extern int rightMotorReceivedControl[3];
extern int vehicleControlReceived[3];

#endif
