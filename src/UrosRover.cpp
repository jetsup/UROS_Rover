#include "main.hpp"

TaskHandle_t comm_task_handle;
TaskHandle_t vehicle_task_handle;

Vehicle* vehicle;
VehicleSensors *vehicleSensors;

void comm_task(void* pvParameters) {
  (void)pvParameters;

  while (true) {
    RCSOFTCHECK(rcl_publish(&magPublisher, &vehicle_mag_data, NULL));

    // randomize vehicle data
    vehicle_mag_data.x = random(-100, 100) / 100.0;
    vehicle_mag_data.y = random(-100, 100) / 100.0;
    vehicle_mag_data.z = random(-100, 100) / 100.0;
    vehicle_gyro_data.x = random(-100, 100) / 100.0;
    vehicle_gyro_data.y = random(-100, 100) / 100.0;
    vehicle_gyro_data.z = random(-100, 100) / 100.0;
    vehicle_accel_data.x = random(-100, 100) / 100.0;
    vehicle_accel_data.y = random(-100, 100) / 100.0;
    vehicle_accel_data.z = random(-100, 100) / 100.0;

    delay(50);
  }
}

void vehicle_task(void* pvParameters) {
  (void)pvParameters;

  while (true) {
    vehicle->measureProximity();
    vehicle->checkReverse();
    vehicle->loop();

    vehicleSensors->loop();
  }
}
