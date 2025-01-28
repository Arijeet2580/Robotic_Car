# 🤖 Obstacle Avoidance Robot Car

## Project Overview
An autonomous robot car that can navigate through its environment while avoiding obstacles using ultrasonic sensors and Arduino microcontroller. The car features both autonomous and manual control modes, making it suitable for various applications and learning purposes.


## 🌟 Features
- **Autonomous Navigation**: Self-driving capability with smart obstacle detection
- **Real-time Obstacle Detection**: Uses ultrasonic sensors for accurate distance measurement
- **Multiple Control Modes**: 
  - Autonomous mode for self-navigation
  - Manual control via Bluetooth
    
- **Intelligent Maneuvers**: Smart turning and path-finding algorithms
- **Variable Speed Control**: Adjustable speed settings for different situations
- **Status Indicators**: LED indicators for different states and operations

## 🛠️ Hardware Requirements
- Arduino Uno/Mega
- L298N Motor Driver Module
- HC-SR04 Ultrasonic Sensor
- SG90 Servo Motor
- 4x DC Motors with Wheels
- Chassis Kit
- 12V Battery Pack
- Jumper Wires
- Bluetooth Module HC-05

## 📋 Pin Configuration
```
// Motor Driver Pins (L298N)
Motor A (Left):
- ENA: Pin 3
- IN1: Pin 4
- IN2: Pin 5

Motor B (Right):
- ENB: Pin 9
- IN3: Pin 6
- IN4: Pin 7

// Sensors
Ultrasonic Sensor:
- TRIG: Pin 12
- ECHO: Pin 11

Servo Motor:
- Signal: Pin 10
```

## 🔧 Assembly Instructions
1. **Chassis Assembly**
   - Mount the DC motors to the chassis
   - Attach wheels to the motors
   - Install the battery holder

2. **Electronics Setup**
   - Mount Arduino on the chassis
   - Install L298N motor driver
   - Connect motors to the driver
   - Mount ultrasonic sensor on servo
   - Install servo on the front

3. **Wiring**
   - Connect components according to pin configuration
   - Double-check all connections
   - Ensure proper power distribution

## 💻 Software Setup
1. **Required Libraries**
   ```cpp
   #include <Servo.h>
   ```

2. **Installation**
   - Download the repository
   - Install required libraries in Arduino IDE
   - Upload the code to Arduino

3. **Configuration**
   - Adjust motor speed constants if needed
   - Modify sensor thresholds as required
   - Set turning delays according to your chassis

## 🎮 Operation Guide
### Autonomous Mode
- Power on the robot
- The car will automatically start in autonomous mode
- It will scan for obstacles and navigate accordingly

### Manual Control (via Bluetooth)
Commands:
- 'F': Move Forward
- 'B': Move Backward
- 'L': Turn Left
- 'R': Turn Right
- 'S': Stop
- 'A': Switch to Autonomous Mode
- 'M': Switch to Manual Mode

## ⚙️ Customization
You can modify these parameters in the code:
```cpp
#define MOTOR_SPEED    180  // Default speed (0-255)
#define TURN_SPEED     160  // Turning speed
#define MIN_DISTANCE   20   // Minimum obstacle distance (cm)
#define TURN_TIME      600  // Turn duration (ms)
```

## 🔍 Troubleshooting
Common issues and solutions:

1. **Car Not Moving**
   - Check battery voltage
   - Verify motor connections
   - Ensure proper code upload

2. **Erratic Movement**
   - Calibrate motor speeds
   - Check wheel alignment
   - Verify sensor readings

3. **Poor Obstacle Detection**
   - Clean ultrasonic sensor
   - Adjust MIN_DISTANCE value
   - Check sensor mounting angle

## 🛡️ Safety Guidelines
- Always operate on a flat surface
- Keep batteries properly charged
- Avoid wet conditions
- Monitor motor temperature
- Keep clear of obstacles during testing

## 🔄 Future Improvements
- [ ] Add GPS navigation
- [ ] Implement machine learning for better path finding
- [ ] Add camera for visual navigation
- [ ] Create mobile app interface
- [ ] Add mapping capability

