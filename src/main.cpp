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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "",
                                 &support));
  Serial.println("Node initialized");

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "uros_data"));
  Serial.println("Publisher initialized");

  vehicle_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Serial.println("Publisher created successfully");

  vehicle =
      new Vehicle(UROS_L298N_ENA_PIN, UROS_L298N_IN1_PIN, UROS_L298N_IN2_PIN,
                  UROS_L298N_ENB_PIN, UROS_L298N_IN3_PIN, UROS_L298N_IN4_PIN,
                  UROS_BUZZER_PIN);
    Serial.println("Vehicle object created successfully");

    // Create task handles
    xTaskCreate(comm_task, "comm_task", 2048, NULL, 1, &comm_task_handle);
    xTaskCreate(vehicle_task, "vehicle_task", 2048, NULL, 1, &vehicle_task_handle);
}

void loop() {
//   RCSOFTCHECK(rcl_publish(&publisher, &vehicle_data, NULL));

//   // randomize vehicle data
//   vehicle_data.mag_x = random(-100, 100) / 100.0;
//   vehicle_data.mag_y = random(-100, 100) / 100.0;
//   vehicle_data.mag_z = random(-100, 100) / 100.0;
//   vehicle_data.gyro_x = random(-100, 100) / 100.0;
//   vehicle_data.gyro_y = random(-100, 100) / 100.0;
//   vehicle_data.gyro_z = random(-100, 100) / 100.0;
}
