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
  pinMode(_hornPin, OUTPUT);
  pinMode(_headLightPin, OUTPUT);
  pinMode(_tailLightPin, OUTPUT);
  pinMode(_reverseLightPin, OUTPUT);

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

  drive(leftMotorReceivedControl[0], rightMotorReceivedControl[0]);
  hoot(vehicleControlReceived[2]);
  setReversing(leftMotorReceivedControl[1] < 0);
  setLightStatus(vehicleControlReceived[0], vehicleControlReceived[1]);

  if (_headLightOn) {
    digitalWrite(_headLightPin, HIGH);
  } else {
    digitalWrite(_headLightPin, LOW);
  }

  if (_tailLightOn) {
    if ((leftMotorReceivedControl[0] < -UROS_MOTOR_SPEED_MIN ||
         leftMotorReceivedControl[0] > UROS_MOTOR_SPEED_MIN ||
         rightMotorReceivedControl[0] < -UROS_MOTOR_SPEED_MIN ||
         rightMotorReceivedControl[0] > UROS_MOTOR_SPEED_MIN) &&
        (leftMotorReceivedControl[1] != 0 ||
         rightMotorReceivedControl[1] != 0)) {
      analogWrite(_tailLightPin, 50);
    } else {
      analogWrite(_tailLightPin, 255);
    }
  } else {
    analogWrite(_tailLightPin, 0);
  }
}

void Vehicle::drive(int leftSpeed, int rightSpeed) {
  _leftSpeed = leftSpeed;
  _rightSpeed = rightSpeed;

  // Turning when the vehicle is stopped
  if (_leftSpeed == 0 && _rightSpeed != 0) {
    _leftSpeed = _rightSpeed;
    _rightSpeed = 0;

    if (_leftSpeed < 0 && leftMotorReceivedControl[1] >= 0) {
      _leftSpeed = -_leftSpeed;
    }
  } else if (_rightSpeed == 0 && _leftSpeed != 0) {
    _rightSpeed = _leftSpeed;
    _leftSpeed = 0;

    if (_rightSpeed < 0 && rightMotorReceivedControl[1] >= 0) {
      _rightSpeed = -_rightSpeed;
    }
  }

  //   Serial.printf(
  //       "Left: x=%d, y=%d, z=%d\tRight: x=%d, y=%d, z=%d\t\tLeft: %d\tRight:
  //       "
  //       "%d\n",
  //       leftMotorReceivedControl[0], leftMotorReceivedControl[1],
  //       leftMotorReceivedControl[2], rightMotorReceivedControl[0],
  //       rightMotorReceivedControl[1], rightMotorReceivedControl[2],
  //       _leftSpeed, _rightSpeed);

  // Set left motor direction and speed
  if (_leftSpeed > (int)UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in1Pin, HIGH);
    digitalWrite(_in2Pin, LOW);
  } else if (_leftSpeed < 0 && fabs(_leftSpeed) > (int)UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, HIGH);
    _leftSpeed = -_leftSpeed;
  } else {
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    analogWrite(_enAPin, 0);
  }

  // Set right motor direction and speed
  if (_rightSpeed > (int)UROS_MOTOR_SPEED_MIN) {
    digitalWrite(_in3Pin, HIGH);
    digitalWrite(_in4Pin, LOW);
  } else if (_rightSpeed < 0 && fabs(_rightSpeed) > (int)UROS_MOTOR_SPEED_MIN) {
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

  analogWrite(_tailLightPin, 255);
  _tailLightOn = true;
}

void Vehicle::hoot(bool hoot) {
  _isHooting = hoot;

  if (_isHooting && !_isReversing) {
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

    if (!_isHornHigh &&
        millis() - _reverseStartTime > UROS_ROVER_REVERSING_SOUND_HIGH) {
      _isHornHigh = true;
      digitalWrite(_hornPin, HIGH);
      _reverseStartTime = millis();
    } else if (_isHornHigh &&
               millis() - _reverseStartTime > UROS_ROVER_REVERSING_SOUND_LOW) {
      _isHornHigh = false;
      digitalWrite(_hornPin, LOW);
      _reverseStartTime = millis();
    }
  } else {
    if (!_isHooting) {
      _isHornHigh = false;
      digitalWrite(_hornPin, LOW);
    }

    _reverseLightOn = false;
  }

  toggleLights(_reverseLightPin, _reverseLightOn ? HIGH : LOW);
}

void Vehicle::setReversing(bool isReversing) { _isReversing = isReversing; }

void Vehicle::toggleLights(uint8_t lightPin, bool on) {
  Serial.printf("LED: %d State: %d\n", lightPin, on);
  digitalWrite(lightPin, on ? HIGH : LOW);
}

void Vehicle::setLightStatus(bool headLightOn, bool tailLightOn) {
  _headLightOn = headLightOn;
  _tailLightOn = tailLightOn;
}

void Vehicle::measureProximity() {
  if (millis() - _previousProximityReadTime >=
      UROS_PROXIMITY_MEASURE_INTERVAL_MS) {
    _frontProximity = _frontProximitySensor->ping_cm();
    _previousProximityReadTime = millis();
  }
}

// ========================= Vehicle Sensors =========================
VehicleSensors::VehicleSensors(uint8_t mpu9250Address) {
// ******************** MPU9250 ********************
#if UROS_ROVER_IMU_PRESENT
  _mpu9250 = new MPU9250_WE(mpu9250Address);

  if (!_mpu9250->init()) {
    Serial.println("MPU9250 initialization failed!");
    while (true);
  } else {
    Serial.println("MPU9250 initialized successfully.");
  }

  Serial.println("Position you MPU9250 flat and don't move it\ncalibrating...");
  delay(1000);

  _mpu9250->autoOffsets();
  Serial.println("Calibration complete!");
  _mpu9250->enableGyrDLPF();
  _mpu9250->setGyrDLPF(MPU9250_DLPF_6);
  _mpu9250->setSampleRateDivider(5);
  _mpu9250->setAccRange(MPU9250_ACC_RANGE_2G);
  _mpu9250->setGyrRange(MPU9250_GYRO_RANGE_250);
  _mpu9250->enableAccDLPF(true);
  _mpu9250->setAccDLPF(MPU9250_DLPF_6);
#endif  // UROS_ROVER_IMU_PRESENT

  // ========================= Proximity Sensors =========================
  _frontProximity = new NewPing(UROS_FRONT_PROXIMITY_TRIGGER_PIN,
                                UROS_FRONT_PROXIMITY_ECHO_PIN,
                                UROS_PROXIMITY_MAX_DISTANCE_CM);
}

VehicleSensors::~VehicleSensors() { delete _mpu9250; }

void VehicleSensors::loop() {
#if UROS_ROVER_IMU_PRESENT
  xyzFloat gValue = _mpu9250->getGValues();
  xyzFloat gyr = _mpu9250->getGyrValues();
  float temp = _mpu9250->getTemperature();
  float resultantG = _mpu9250->getResultantG(gValue);

  vehicle_accel_data.x = gValue.x;
  vehicle_accel_data.y = gValue.y;
  vehicle_accel_data.z = gValue.z;

  vehicle_gyro_data.x = gyr.x;
  vehicle_gyro_data.y = gyr.y;
  vehicle_gyro_data.z = gyr.z;

  vehicle_orientation_data.x = resultantG;
#endif  // UROS_ROVER_IMU_PRESENT

  vehicleProximity.x = _frontProximity->ping_cm();
}
