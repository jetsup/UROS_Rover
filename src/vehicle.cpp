#include "vehicle.hpp"

Vehicle::Vehicle(int enaPin, int in1Pin, int in2Pin, int enbPin, int in3Pin,
                 int in4Pin, int hornPin)
    : _hornPin(hornPin),
      _enAPin(enaPin),
      _in1Pin(in1Pin),
      _in2Pin(in2Pin),
      _enBPin(enbPin),
      _in3Pin(in3Pin),
      _in4Pin(in4Pin) {
  pinMode(_enAPin, OUTPUT);
  pinMode(_in1Pin, OUTPUT);
  pinMode(_in2Pin, OUTPUT);
  pinMode(_enBPin, OUTPUT);
  pinMode(_in3Pin, OUTPUT);
  pinMode(_in4Pin, OUTPUT);

  _frontProximitySensor =
      new NewPing(_frontProximityTriggerPin, _frontProximityEchoPin,
                  UROS_FRONT_PROXIMITY_MAX_DISTANCE_CM);
}

void Vehicle::loop() {
  Serial.printf(
      "Left: x=%d, y=%d, z=%d\tRight: x=%d, y=%d, z=%d\tControl: x=%d, y=%d, "
      "z=%d\n",
      leftMotorReceivedControl[0], leftMotorReceivedControl[1],
      leftMotorReceivedControl[2], rightMotorReceivedControl[0],
      rightMotorReceivedControl[1], rightMotorReceivedControl[2],
      vehicleControlReceived[0], vehicleControlReceived[1],
      vehicleControlReceived[2]);
  delay(50);

  drive(leftMotorReceivedControl[0], rightMotorReceivedControl[0]);
  hoot(vehicleControlReceived[2]);
}

void Vehicle::drive(int leftSpeed, int rightSpeed) {
  _leftSpeed = leftSpeed;
  _rightSpeed = rightSpeed;

  // Set left motor direction and speed
  if (_leftSpeed > UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in1Pin, HIGH);
    digitalWrite(_in2Pin, LOW);
  } else if (_leftSpeed < 0 && -_leftSpeed > UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, HIGH);
    _leftSpeed = -_leftSpeed;
  } else {
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    analogWrite(_enAPin, 0);
  }

  // Set right motor direction and speed
  if (_rightSpeed > UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in3Pin, HIGH);
    digitalWrite(_in4Pin, LOW);
  } else if (_rightSpeed < 0 && -_rightSpeed > UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in3Pin, LOW);
    digitalWrite(_in4Pin, HIGH);
    _rightSpeed = -_rightSpeed;
  } else {
    digitalWrite(_in3Pin, LOW);
    digitalWrite(_in4Pin, LOW);
    analogWrite(_enBPin, 0);
  }

  // Set the speed for both motors
  analogWrite(_enAPin, _leftSpeed);
  analogWrite(_enBPin, _rightSpeed);
}

void Vehicle::stop() {
  // Stop both motors
  digitalWrite(_in1Pin, LOW);
  digitalWrite(_in2Pin, LOW);
  digitalWrite(_in3Pin, LOW);
  digitalWrite(_in4Pin, LOW);
  analogWrite(_enAPin, 0);
  analogWrite(_enBPin, 0);

  digitalWrite(_tailLightPin, _tailLightOn ? HIGH : LOW);
}

void Vehicle::hoot(bool hoot) {
  if (_isHooting == hoot) {
    return;
  }

  _isHooting = hoot;

  if (!_isHooting) {
    _isHornHigh = true;
    digitalWrite(_hornPin, HIGH);
  } else {
    if (!_isReversing) {
      _isHornHigh = false;
      digitalWrite(_hornPin, LOW);
    }
  }
}

void Vehicle::checkReverse() {
  if (_isReversing) {
    _reverseLightOn = true;

    if (!_isHornHigh && millis() - _reverseStartTime > 500) {
      _isHornHigh = true;
      digitalWrite(_hornPin, HIGH);
    } else if (_isHornHigh && millis() - _reverseStartTime > 300 &&
               !_isHooting) {
      _isHornHigh = false;
      digitalWrite(_hornPin, LOW);
    }
  } else {
    digitalWrite(_hornPin, LOW);

    _reverseLightOn = false;
  }

  toggleLights(_reverseLightPin, _reverseLightOn ? HIGH : LOW);
}

void Vehicle::toggleLights(uint8_t lightPin, bool on) {
  digitalWrite(lightPin, on ? HIGH : LOW);
}

void Vehicle::measureProximity() {
  if (millis() - _previousProximityReadTime >=
      UROS_PROXIMITY_MEASURE_INTERVAL_MS) {
    _frontProximity = _frontProximitySensor->ping_cm();
    _previousProximityReadTime = millis();
  }
}
