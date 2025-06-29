#include "comm.hpp"

rcl_publisher_t publisher;
vehicle_data_t vehicle_data;
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
    RCSOFTCHECK(rcl_publish(&publisher, &vehicle_data, NULL));
    // msg.data++;

    // populate mag and gyro data with random data
    vehicle_data.mag_x = random(-100, 100) / 100.0;
    vehicle_data.mag_y = random(-100, 100) / 100.0;
    vehicle_data.mag_z = random(-100, 100) / 100.0;
    vehicle_data.gyro_x = random(-100, 100) / 100.0;
    vehicle_data.gyro_y = random(-100, 100) / 100.0;
    vehicle_data.gyro_z = random(-100, 100) / 100.0;
  }
}
