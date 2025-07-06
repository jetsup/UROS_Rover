#include "main.hpp"

void setup() {
  delay(1000);

  Serial.begin(115200);
  if (!Wire.begin(UROS_I2C_SDA_PIN, UROS_I2C_SCL_PIN)) {
    Serial.println("Could not initialize the I2C bus!");
    while (true);
  }
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
  rcl_init_options_t urosOptions = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&urosOptions, allocator);
  rcl_init_options_set_domain_id(&urosOptions, UROS_ROS_DOMAIN_ID);

  Serial.println("Init options created");

  //   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  //   Serial.println("Support initialized");
  // Initialize the support structure
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &urosOptions,
                                         &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "uros_rover", "", &support));

  Serial.println("Node initialized");

  // Create Publishers
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
  RCCHECK(rclc_publisher_init_best_effort(
      &orientationPublisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_orientation_data"));
  RCCHECK(rclc_publisher_init_best_effort(
      &proximityPublisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_proximity_data"));

  Serial.println("Publishers initialized");

  // Initialize the timer
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100),
                                  timer_callback));
  Serial.println("Timer initialized");

  // Create Subscribers
  RCCHECK(rclc_subscription_init_best_effort(
      &leftMotorControlSubscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_left_motor_control"));
  RCCHECK(rclc_subscription_init_best_effort(
      &rightMotorControlSubscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_right_motor_control"));
  RCCHECK(rclc_subscription_init_best_effort(
      &vehicleControlSubscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
      "uros_vehicle_control"));

  // Change the number of handles in the executor to match the number of
  // subscriptions and timers used
  rclc_executor_init(&executor, &support.context, 8, &allocator);
  rclc_executor_add_subscription(&executor, &leftMotorControlSubscriber,
                                 &leftMotorControlData,
                                 &leftMotorSubscriptionCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &rightMotorControlSubscriber,
                                 &rightMotorControlData,
                                 &rightMotorSubscriptionCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(
      &executor, &vehicleControlSubscriber, &vehicleControlData,
      &vehicleControlSubscriptionCallback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);
  //   vehicle_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Serial.println("Subscribers initialized");

  vehicle =
      new Vehicle(UROS_L298N_ENA_PIN, UROS_L298N_IN1_PIN, UROS_L298N_IN2_PIN,
                  UROS_L298N_ENB_PIN, UROS_L298N_IN3_PIN, UROS_L298N_IN4_PIN,
                  UROS_BUZZER_PIN);
  vehicleSensors = new VehicleSensors(UROS_MPU6500_ADDRESS);
  Serial.println("Vehicle object and sensors created successfully");

  // Create task handles
  xTaskCreatePinnedToCore(vehicle_task, "vehicle_task", 4096, nullptr, 1,
                          nullptr, 1);
}

void loop() { rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); }
