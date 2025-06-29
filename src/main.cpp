#include "main.hpp"

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting micro-ROS on WiFi...");

  set_microros_wifi_transports(UROS_WIFI_SSID, UROS_WIFI_PASSWORD,
                               UROS_MICRO_ROS_AGENT_IP,
                               UROS_MICRO_ROS_AGENT_PORT);

  // Wait for Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(UROS_LED_PIN,
                 !digitalRead(UROS_LED_PIN));  // Blink LED while connecting
  }

  Serial.println("");
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(UROS_LED_PIN, OUTPUT);
  digitalWrite(UROS_LED_PIN, HIGH);

  allocator = rcl_get_default_allocator();
  Serial.println("Using default allocator");

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("Support initialized");

  // create node
  RCCHECK(rclc_node_init_default(&node, "microros_rover", "", &support));
  Serial.println("Node initialized");

  // create magPublisher
  RCCHECK(rclc_publisher_init_best_effort(
      &magPublisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_mag_data"));
  RCCHECK(rclc_publisher_init_best_effort(
      &gyroPublisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_gyro_data"));
  RCCHECK(rclc_publisher_init_best_effort(
      &accelPublisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_accel_data"));

  Serial.println("Publishers initialized");

  //   vehicle_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Serial.println("Publisher created successfully");

  //   vehicle =
  //       new Vehicle(UROS_L298N_ENA_PIN, UROS_L298N_IN1_PIN,
  //       UROS_L298N_IN2_PIN,
  //                   UROS_L298N_ENB_PIN, UROS_L298N_IN3_PIN,
  //                   UROS_L298N_IN4_PIN, UROS_BUZZER_PIN);
  //   Serial.println("Vehicle object created successfully");

  // Create task handles
  // xTaskCreatePinnedToCore(vehicle_task, "vehicle_task", 4096, nullptr, 1,
  //                           nullptr, 0);
}

void loop() {
  RCSOFTCHECK(rcl_publish(&magPublisher, &vehicle_mag_data, NULL));
  RCSOFTCHECK(rcl_publish(&gyroPublisher, &vehicle_gyro_data, NULL));
  RCSOFTCHECK(rcl_publish(&accelPublisher, &vehicle_accel_data, NULL));

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

  delay(200);
}
