#include "comm.hpp"

// vehicle_data_t vehicle_data;
rcl_publisher_t magPublisher;
rcl_publisher_t gyroPublisher;
rcl_publisher_t accelPublisher;
rcl_publisher_t orientationPublisher;
rcl_publisher_t proximityPublisher;

geometry_msgs__msg__Vector3 vehicle_mag_data;
geometry_msgs__msg__Vector3 vehicle_gyro_data;
geometry_msgs__msg__Vector3 vehicle_accel_data;
geometry_msgs__msg__Vector3 vehicle_orientation_data;
geometry_msgs__msg__Vector3 vehicleProximity;

rcl_subscription_t leftMotorControlSubscriber;
rcl_subscription_t rightMotorControlSubscriber;
rcl_subscription_t vehicleControlSubscriber; /* lights and hooting */

geometry_msgs__msg__Vector3 leftMotorControlData;
geometry_msgs__msg__Vector3 rightMotorControlData;
geometry_msgs__msg__Vector3 vehicleControlData; /* lights and hooting */

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

void error_loop() {
  while (1) {
    digitalWrite(UROS_LED_PIN, !digitalRead(UROS_LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&magPublisher, &vehicle_mag_data, NULL));
    RCSOFTCHECK(rcl_publish(&gyroPublisher, &vehicle_gyro_data, NULL));
    RCSOFTCHECK(rcl_publish(&accelPublisher, &vehicle_accel_data, NULL));
    RCSOFTCHECK(rcl_publish(&orientationPublisher, &vehicle_orientation_data, NULL));
    RCSOFTCHECK(rcl_publish(&proximityPublisher, &vehicleProximity, NULL));
    // msg.data++;

    // populate mag and gyro data with random data
    vehicle_mag_data.x = random(-100, 100) / 100.0;
    vehicle_mag_data.y = random(-100, 100) / 100.0;
    vehicle_mag_data.z = random(-100, 100) / 100.0;
  }
}

void leftMotorSubscriptionCallback(const void* msg) {
  const geometry_msgs__msg__Vector3* data =
      (const geometry_msgs__msg__Vector3*)msg;

  leftMotorReceivedControl[0] = data->x;
  leftMotorReceivedControl[1] = data->y;
  leftMotorReceivedControl[2] = data->z;
}

void rightMotorSubscriptionCallback(const void* msg) {
  const geometry_msgs__msg__Vector3* data =
      (const geometry_msgs__msg__Vector3*)msg;

  rightMotorReceivedControl[0] = data->x;
  rightMotorReceivedControl[1] = data->y;
  rightMotorReceivedControl[2] = data->z;
}

void vehicleControlSubscriptionCallback(const void* msg) {
  const geometry_msgs__msg__Vector3* data =
      (const geometry_msgs__msg__Vector3*)msg;

  vehicleControlReceived[0] = data->x;
  vehicleControlReceived[1] = data->y;
  vehicleControlReceived[2] = data->z;
}
