#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "comm.hpp"
#include "config.h"
#include "vehicle.hpp"

extern TaskHandle_t comm_task_handle;
extern TaskHandle_t vehicle_task_handle;

extern Vehicle* vehicle;
extern VehicleSensors* vehicleSensors;

void comm_task(void* pvParameters);
void vehicle_task(void* pvParameters);
