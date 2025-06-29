#include "main.hpp"

TaskHandle_t comm_task_handle;
TaskHandle_t vehicle_task_handle;

Vehicle* vehicle;

void comm_task(void* pvParameters) {
  (void)pvParameters;

  while (true) {
    RCSOFTCHECK(rcl_publish(&publisher, &vehicle_data, NULL));

    // randomize vehicle data
    vehicle_data.mag_x = random(-100, 100) / 100.0;
    vehicle_data.mag_y = random(-100, 100) / 100.0;
    vehicle_data.mag_z = random(-100, 100) / 100.0;
    vehicle_data.gyro_x = random(-100, 100) / 100.0;
    vehicle_data.gyro_y = random(-100, 100) / 100.0;
    vehicle_data.gyro_z = random(-100, 100) / 100.0;

    delay(50);
  }
}

void vehicle_task(void* pvParameters) {
  (void)pvParameters;

  while (true) {
    vehicle->measureProximity();
    vehicle->checkReverse();
    // vehicle.loop();
    delay(100);
  }
}
