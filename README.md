# Micro ROS Rover

This is a project for a rover using the micro ros. It should send data and receive commands from a [ROS2](https://docs.ros.org/en/jazzy/index.html) Node. The project is based on the [Arduino](https://www.arduino.cc/) framework and uses the micro-ROS library for communication with ROS2.

## Setup

Before you proceed with the setup, make sure you go through the [Custom Messages](CustomMessages.md) section to understand how to create and use custom messages in this project.

1. Install [PlatformIO](https://platformio.org/install) and flush the firmware to the board. Remember to update the WiFi [credentials](include/credentials.h) before uploading the code.

   ```bash
   cp include/credentials.example.h include/credentials.h
   ```

   Build and upload the firmware using PlatformIO:

   ```bash
   platformio run --target upload
   ```

2. Install the [micro-ROS Agent](https://micro.ros.org/docs/tutorials/core/first_application_linux/) on your computer.

3. Make sure your firewall allows for port 8888, which is used by the micro-ROS Agent for communication. You can check this by running:

   ```bash
   sudo ufw status
   ```

   If the port is not allowed, you can allow it with the following command:

   ```bash
   sudo ufw allow 8888/udp
   ```

4. Run the micro-ROS Agent on your computer with the following command:

   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

5. Open the [ROS2 CLI](https://docs.ros.org/en/jazzy/Command-Line-Interface.html) and run the following command to check if the rover is connected:

   ```bash
   ros2 topic list
   ```

6. If everything runs, your `rqt_graph` should look like this:

   ![rqt_graph](resources/rqt_communication.png)

   The `rover_controller` node is found in the [uros_rover_controller](https://github.com/jetsup/uros_rover_controller.git) repository.

### Troubleshooting

   If you do not see your topic, check and make sure that the port is not being blocked by a firewall or other software. You can allow the port in your terminal with the following command:

   ```bash
   sudo ufw allow 8888/udp
   ```
