#pragma once

/**
 * @file motors.hpp
 * @date 2025-06-29
 * @author George Ngigi (jetsup)
 * @version 0.0.1
 * @brief Motor control functions for the Micro-ROS rover project. The motors
 * for this rover are defined as a class encapsulating both left and right
 * motors. This class provides methods to control the speed and direction of the
 * motors.
 */

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <MPU9250_WE.h>
#include <NewPing.h>
#include <math.h>

#include "comm.hpp"
#include "config.h"

typedef enum {
  E_UROS_TURNING_LEFT = -1,
  E_UROS_TURNING_NONE,
  E_UROS_TURNING_RIGHT,
} TurningDirection;

class Vehicle {
 private:
  // Pin definitions for the L298N motor driver
  uint8_t _enAPin = UROS_L298N_ENA_PIN;
  uint8_t _in1Pin = UROS_L298N_IN1_PIN;
  uint8_t _in2Pin = UROS_L298N_IN2_PIN;
  uint8_t _enBPin = UROS_L298N_ENB_PIN;
  uint8_t _in3Pin = UROS_L298N_IN3_PIN;
  uint8_t _in4Pin = UROS_L298N_IN4_PIN;
  // Pin definitions for the the vehicles functionalities
  uint8_t _hornPin = UROS_BUZZER_PIN;
  uint8_t _headLightPin = UROS_HEADLIGHT_PIN;
  uint8_t _tailLightPin = UROS_TAIL_LIGHT_PIN;
  uint8_t _reverseLightPin = UROS_REVERSE_LIGHT_PIN;
  uint8_t _frontProximityTriggerPin = UROS_FRONT_PROXIMITY_TRIGGER_PIN;
  uint8_t _frontProximityEchoPin = UROS_FRONT_PROXIMITY_ECHO_PIN;

  // Control variables
  bool _isReversing = false;
  unsigned long _reverseStartTime = 0;
  bool _isHooting = false;
  bool _isHornHigh = false;
  bool _headLightOn = false;
  bool _tailLightOn = false;
  bool _reverseLightOn = false;
  float _frontProximity = 0.0f;
  unsigned long _previousProximityReadTime = 0;
  // When this variable is set, even if the vehicle is not turning, it will
  // display some animation
  bool _indicatorPatterns = false;
  uint8_t _indicatorAnimIndex = 0;
  unsigned long _lastIndicatorUpdate = 0;

  // Speed variables
  short _leftSpeed = 0;
  short _rightSpeed = 0;

  bool _animateIn = false;

  // Class objects
  NewPing* _frontProximitySensor;
  Adafruit_NeoPixel* _indicatorStrip;

 public:
  Vehicle() = delete;

  /**
   * @brief Construct a new Vehicle object with the specified pin numbers.
   * This constructor initializes the vehicle's motor control pins and the horn
   * pin.
   * @param enaPin Pin number for the ENA pin of the L298N motor driver.
   * @param in1Pin Pin number for the IN1 pin of the L298N motor driver.
   * @param in2Pin Pin number for the IN2 pin of the L298N motor driver.
   * @param enbPin Pin number for the ENB pin of the L298N motor driver.
   * @param in3Pin Pin number for the IN3 pin of the L298N motor driver.
   * @param in4Pin Pin number for the IN4 pin of the L298N motor driver.
   * @param hornPin Pin number for the horn (buzzer). Default is
   * `UROS_BUZZER_PIN`.
   */
  Vehicle(int enaPin, int in1Pin, int in2Pin, int enbPin, int in3Pin,
          int in4Pin, int hornPin = UROS_BUZZER_PIN);

  /**
   * @brief Main loop for the vehicle control.
   */
  void loop();

  /**
   * @brief Drive the vehicle by setting the speed of the left and right motors.
   * This function sets the speed of the left and right motors based on the
   * provided speed values. The speed values should be in the range of 0 to 255,
   * where 0 is stopped and 255 is full speed.
   * @param leftSpeed Speed of the left motor (0-255).
   * @param rightSpeed Speed of the right motor (0-255).
   * @note The speed values can be negative to indicate reverse direction.
   */
  void drive(int leftSpeed, int rightSpeed);

  /**
   * @brief Stop the vehicle by setting both motors to zero speed.
   */
  void stop();
  //   void accelerate();
  //   void decelerate();

  /**
   * @brief Make the hooting sound using the horn (buzzer).
   * @param hoot If true, the horn will sound; if false, it will stop.
   */
  void hoot(bool hoot);

  /**
   * @brief Checks if the vehicle is reversing and play the pattern beep if it
   * is reversing. This function should be called in the main loop to ensure
   * that the vehicle's reversing state is always checked and the appropriate
   * action is taken.
   */
  void checkReverse();

  /**
   * @brief Set the reversing state of the vehicle.
   * @param isReversing If true, the vehicle is set to reversing mode;
   * otherwise, it is set to normal driving mode.
   */
  void setReversing(bool isReversing);

  /**
   * @brief Toggle the lights of the vehicle.
   * @param lightPin The pin number for the light to be toggled.
   * @param on If true, the lights will be turned on; if false, they
   * will be turned off.
   * @note This function controls the headlight, tail light, and reverse light
   * of the vehicle. It will turn on or off the lights based on the provided
   * parameter.
   */
  void toggleLights(uint8_t lightPin, bool on);

  /**
   * @brief Set the light status of the vehicle.
   * @param headLightOn If true, the headlight will be turned on; otherwise,
   * it will be turned off.
   * @param tailLightOn If true, the tail light will be turned on; otherwise,
   * it will be turned off.
   * @note This function controls the headlight and tail light of the vehicle.
   */
  void setLightStatus(bool headLightOn, bool tailLightOn);

  /**
   * @brief Show the turn indicator lights based on the vehicle's turning
   * direction.
   * @param direction The turning direction to display (left, right, or none).
   */
  void showIndicator(
      TurningDirection direction = TurningDirection::E_UROS_TURNING_NONE);

  /**
   * @brief Read the proximity sensor values in cm
   */
  void measureProximity();
};

class VehicleSensors {
 private:
  MPU9250_WE* _mpu9250;
  NewPing* _frontProximity;

 public:
  VehicleSensors(uint8_t mpu9250Address = UROS_MPU9250_ADDRESS);
  ~VehicleSensors();

  void loop();
};
