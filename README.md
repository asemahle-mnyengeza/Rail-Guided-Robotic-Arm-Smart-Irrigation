# Rail-Guided Robotic Arm for Crop Monitoring and Precision Irrigation

## About the Project

This project involves the development of an autonomous rail-guided robotic arm designed to monitor crop conditions and provide automated irrigation.

The robotic arm moves along a fixed rail to different crop stations, where it performs soil moisture measurements and waters crops when necessary.

The system combines robotics, embedded systems, sensors and automation to improve crop monitoring and irrigation efficiency.

## Main Features

- Automated rail movement using a stepper motor.
- Four-servo robotic arm.
- Soil moisture monitoring.
- Automatic water pump control.
- Ultrasonic obstacle detection.

## Hardware Components

- Arduino Uno
- PCA9685 servo driver
- MG996R servo motors
- Stepper motor and A4988 driver
- Soil moisture sensor
- HC-SR04 ultrasonic sensor
- Relay module
- Water pump

## Programming Tools

- Arduino IDE
- C/C++

## Project Images

![Full System Project](Full%20system%20project%20image.jpg)

## Source Code

The complete source code for the rail-guided robotic arm and automated irrigation system is provided in the Arduino sketch below.

**Main Source Code:**
-[ FInal_code.ino ](FInal_code.ino)

The program contains the control logic developed for the integrated robotic system, including its movement, monitoring and irrigation functions.

## How the System Works

The rail-guided robotic arm operates through an automated sequence to monitor and service crops at different stations.

1. **Rail Movement:** The stepper motor moves the robotic arm along the rail to the designated crop station.
2. **Arm Positioning:** The four-servo robotic arm moves into position to perform crop monitoring.
3. **Soil Monitoring:** The soil moisture sensor measures the moisture level of the soil.
4. **Irrigation:** If the soil moisture level falls below the set threshold of 50%, the relay activates the water pump to supply water.
5. **Repeat Operation:** The robotic arm retracts, and the rail moves to the next crop station. The process repeats until all designated stations have been serviced.

## Future Improvements

- Implement QR-code-based crop identification using the ESP32-CAM.
- Improve coordination between the rail and robotic arm.
- Enhance autonomous crop monitoring and irrigation.
