#include "comm.hpp"

rcl_publisher_t magPublisher;
rcl_publisher_t gyroPublisher;
rcl_publisher_t accelPublisher;
// vehicle_data_t vehicle_data;
geometry_msgs__msg__Vector3 vehicle_mag_data;
geometry_msgs__msg__Vector3 vehicle_gyro_data;
geometry_msgs__msg__Vector3 vehicle_accel_data;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

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
    // msg.data++;

    // populate mag and gyro data with random data
    vehicle_mag_data.x = random(-100, 100) / 100.0;
    vehicle_mag_data.y = random(-100, 100) / 100.0;
    vehicle_mag_data.z = random(-100, 100) / 100.0;
    vehicle_gyro_data.x = random(-100, 100) / 100.0;
    vehicle_gyro_data.y = random(-100, 100) / 100.0;
    vehicle_gyro_data.z = random(-100, 100) / 100.0;
    vehicle_accel_data.x = random(-100, 100) / 100.0;
    vehicle_accel_data.y = random(-100, 100) / 100.0;
    vehicle_accel_data.z = random(-100, 100) / 100.0;
  }
}
