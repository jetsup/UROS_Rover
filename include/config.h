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

#define UROS_L298N_ENA_PIN 4
#define UROS_L298N_IN1_PIN 5
#define UROS_L298N_IN2_PIN 6
#define UROS_L298N_ENB_PIN 3
#define UROS_L298N_IN3_PIN 7
#define UROS_L298N_IN4_PIN 8

#define UROS_BUZZER_PIN 9
#define UROS_HEADLIGHT_PIN 10
#define UROS_TAIL_LIGHT_PIN 11
#define UROS_REVERSE_LIGHT_PIN 12
#define UROS_FRONT_PROXIMITY_TRIGGER_PIN 13
#define UROS_FRONT_PROXIMITY_ECHO_PIN 14

// Motor speed thresholds
#define UROS_MOTOR_SPEED_MIN 50

// Proximity sensor settings
#define UROS_FRONT_PROXIMITY_MAX_DISTANCE_CM 400
#define UROS_PROXIMITY_MEASURE_INTERVAL_MS 10  

#endif
