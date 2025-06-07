/*
 * Enhanced Autonomous Robot Car using L298N Motor Driver
 * Features:
 * - Variable speed control based on distance
 * - Multiple ultrasonic sensors with sensor fusion
 * - Advanced navigation with wall-following and path memory
 * - Enhanced communication with command queuing
 */

#include <Servo.h>

// Motor A (Left) pins
#define IN1 4  // Motor A direction control 1
#define IN2 5  // Motor A direction control 2
#define ENA 3  // Motor A speed control (PWM)

// Motor B (Right) pins
#define IN3 6  // Motor B direction control 1
#define IN4 7  // Motor B direction control 2
#define ENB 9  // Motor B speed control (PWM)

// Multiple Ultrasonic sensor pins
#define TRIG_FRONT 12
#define ECHO_FRONT 11
#define TRIG_LEFT  A0
#define ECHO_LEFT  A1
#define TRIG_RIGHT A2
#define ECHO_RIGHT A3

// Servo pin
#define SERVO_PIN 10

// Speed Control Constants
#define MAX_SPEED       255  // Maximum motor speed
#define MIN_SPEED       80   // Minimum operational speed
#define CRUISE_SPEED    180  // Normal cruising speed
#define TURN_SPEED      120  // Speed during turns
#define SLOW_SPEED      100  // Speed when approaching obstacles

// Distance Constants
#define SAFE_DISTANCE     25   // Safe following distance (cm)
#define MIN_DISTANCE      10   // Emergency stop distance (cm)
#define WALL_FOLLOW_DIST  15   // Preferred wall following distance (cm)
#define SENSOR_TIMEOUT    30000 // Sensor timeout in microseconds

// Navigation Constants
#define SERVO_CENTER   90   // Center position for servo
#define SERVO_LEFT     180  // Leftmost servo angle
#define SERVO_RIGHT    0    // Rightmost servo angle
#define TURN_TIME      400  // Base time for turning (ms)
#define ACCEL_STEP     5    // Speed change per acceleration step
#define ACCEL_DELAY    20   // Delay between acceleration steps (ms)

// Path Memory Constants
#define MAX_PATH_MEMORY 20  // Maximum path history to remember
#define STUCK_THRESHOLD 5   // Number of similar positions before considering stuck

// Command Queue Constants
#define QUEUE_SIZE 10       // Maximum queued commands

// Create servo object
Servo servoHead;

// Speed Control Variables
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
unsigned long lastAccelTime = 0;

// Sensor Variables
struct SensorData {
  int front;
  int left;
  int right;
  bool frontValid;
  bool leftValid;
  bool rightValid;
  unsigned long lastUpdate;
};
SensorData sensors;

// Navigation Variables
enum NavigationMode {
  AUTONOMOUS,
  WALL_FOLLOW_LEFT,
  WALL_FOLLOW_RIGHT,
  ESCAPE_MODE
};
NavigationMode navMode = AUTONOMOUS;

// Path Memory System
struct PathPoint {
  int frontDist;
  int leftDist;
  int rightDist;
  unsigned long timestamp;
};
PathPoint pathMemory[MAX_PATH_MEMORY];
int pathIndex = 0;
int stuckCounter = 0;

// Communication System
struct Command {
  char cmd;
  int param1;
  int param2;
  unsigned long executeTime;
};
Command commandQueue[QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

// Status Reporting
unsigned long lastStatusReport = 0;
#define STATUS_INTERVAL 1000  // Report status every 1 second

void setup() {
  Serial.begin(9600);
  
  // Configure motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configure ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Initialize the servo and set to center position
  servoHead.attach(SERVO_PIN);
  servoHead.write(SERVO_CENTER);
  delay(500);

  // Initialize variables
  stopMotors();
  initializeSensors();
  initializePathMemory();
  
  Serial.println("Enhanced Robot Car Ready!");
  Serial.println("Commands: F(orward), B(ack), L(eft), R(ight), S(top)");
  Serial.println("Advanced: W(all follow), A(utonomous), Q(ueue), V(speed), D(iagonal)");
  printStatus();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Process serial commands
  if (Serial.available() > 0) {
    handleSerialInput();
  }
  
  // Execute queued commands
  processCommandQueue();
  
  // Update sensors
  updateAllSensors();
  
  // Main navigation logic
  if (queueCount == 0) {  // Only run autonomous mode if no commands queued
    executeNavigationMode();
  }
  
  // Smooth speed control
  updateMotorSpeeds();
  
  // Status reporting
  if (currentTime - lastStatusReport >= STATUS_INTERVAL) {
    printStatus();
    lastStatusReport = currentTime;
  }
  
  // Update path memory
  updatePathMemory();
  
  delay(50);  // Main loop delay
}

// ============= SPEED CONTROL SYSTEM =============

void setTargetSpeed(int leftSpeed, int rightSpeed) {
  targetLeftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  targetRightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
}

void updateMotorSpeeds() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastAccelTime >= ACCEL_DELAY) {
    // Smooth acceleration for left motor
    if (currentLeftSpeed < targetLeftSpeed) {
      currentLeftSpeed = min(currentLeftSpeed + ACCEL_STEP, targetLeftSpeed);
    } else if (currentLeftSpeed > targetLeftSpeed) {
      currentLeftSpeed = max(currentLeftSpeed - ACCEL_STEP, targetLeftSpeed);
    }
    
    // Smooth acceleration for right motor
    if (currentRightSpeed < targetRightSpeed) {
      currentRightSpeed = min(currentRightSpeed + ACCEL_STEP, targetRightSpeed);
    } else if (currentRightSpeed > targetRightSpeed) {
      currentRightSpeed = max(currentRightSpeed - ACCEL_STEP, targetRightSpeed);
    }
    
    // Apply speeds to motors
    applyMotorSpeeds(currentLeftSpeed, currentRightSpeed);
    lastAccelTime = currentTime;
  }
}

void applyMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor (Motor A)
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, abs(leftSpeed));
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(leftSpeed));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
  
  // Right motor (Motor B)
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, abs(rightSpeed));
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(rightSpeed));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

int calculateSpeedFromDistance(int distance) {
  if (distance <= MIN_DISTANCE) {
    return 0;  // Stop
  } else if (distance <= SAFE_DISTANCE) {
    // Linear interpolation between MIN_SPEED and CRUISE_SPEED
    return map(distance, MIN_DISTANCE, SAFE_DISTANCE, MIN_SPEED, CRUISE_SPEED);
  } else {
    return CRUISE_SPEED;  // Full speed
  }
}

// ============= SENSOR SYSTEM =============

void initializeSensors() {
  sensors.front = 0;
  sensors.left = 0;
  sensors.right = 0;
  sensors.frontValid = false;
  sensors.leftValid = false;
  sensors.rightValid = false;
  sensors.lastUpdate = 0;
}

void updateAllSensors() {
  sensors.front = getDistanceWithTimeout(TRIG_FRONT, ECHO_FRONT);
  sensors.frontValid = (sensors.front > 0 && sensors.front < 400);
  
  sensors.left = getDistanceWithTimeout(TRIG_LEFT, ECHO_LEFT);
  sensors.leftValid = (sensors.left > 0 && sensors.left < 400);
  
  sensors.right = getDistanceWithTimeout(TRIG_RIGHT, ECHO_RIGHT);
  sensors.rightValid = (sensors.right > 0 && sensors.right < 400);
  
  sensors.lastUpdate = millis();
}

int getDistanceWithTimeout(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
  
  if (duration == 0) {
    return -1;  // Timeout or no echo
  }
  
  return duration * 0.034 / 2;
}

int getFusedFrontDistance() {
  // Use sensor fusion for more reliable front distance
  if (sensors.frontValid) {
    return sensors.front;
  } else {
    // Fallback: estimate from side sensors if available
    if (sensors.leftValid && sensors.rightValid) {
      return min(sensors.left, sensors.right);
    } else if (sensors.leftValid) {
      return sensors.left;
    } else if (sensors.rightValid) {
      return sensors.right;
    }
  }
  return SAFE_DISTANCE;  // Safe default
}

// ============= NAVIGATION SYSTEM =============

void executeNavigationMode() {
  switch (navMode) {
    case AUTONOMOUS:
      autonomousNavigation();
      break;
    case WALL_FOLLOW_LEFT:
      wallFollowLeft();
      break;
    case WALL_FOLLOW_RIGHT:
      wallFollowRight();
      break;
    case ESCAPE_MODE:
      escapeMode();
      break;
  }
}

void autonomousNavigation() {
  int frontDist = getFusedFrontDistance();
  
  if (frontDist > SAFE_DISTANCE) {
    // Clear path - move forward with speed based on distance
    int speed = calculateSpeedFromDistance(frontDist);
    setTargetSpeed(speed, speed);
  } else if (frontDist > MIN_DISTANCE) {
    // Close to obstacle - slow down and prepare to navigate
    setTargetSpeed(SLOW_SPEED, SLOW_SPEED);
    
    // Check if we should switch to wall following
    if (sensors.leftValid && sensors.rightValid) {
      if (sensors.left < sensors.right && sensors.right > WALL_FOLLOW_DIST) {
        navMode = WALL_FOLLOW_RIGHT;
      } else if (sensors.left > WALL_FOLLOW_DIST) {
        navMode = WALL_FOLLOW_LEFT;
      } else {
        handleObstacleWithGradient();
      }
    } else {
      handleObstacleWithGradient();
    }
  } else {
    // Too close - stop and handle obstacle
    setTargetSpeed(0, 0);
    handleObstacleWithGradient();
  }
}

void wallFollowLeft() {
  if (!sensors.leftValid) {
    navMode = AUTONOMOUS;
    return;
  }
  
  int frontDist = getFusedFrontDistance();
  int leftDist = sensors.left;
  
  if (frontDist <= MIN_DISTANCE) {
    // Wall ahead - turn right
    gradientTurn(false);
    navMode = AUTONOMOUS;
  } else {
    // Adjust steering based on left wall distance
    int baseSpeed = calculateSpeedFromDistance(frontDist);
    int speedDiff = (leftDist - WALL_FOLLOW_DIST) * 2;  // Proportional control
    
    int leftSpeed = baseSpeed - speedDiff;
    int rightSpeed = baseSpeed + speedDiff;
    
    leftSpeed = constrain(leftSpeed, MIN_SPEED/2, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED/2, MAX_SPEED);
    
    setTargetSpeed(leftSpeed, rightSpeed);
  }
  
  // Exit wall following if no wall detected
  if (leftDist > SAFE_DISTANCE * 2) {
    navMode = AUTONOMOUS;
  }
}

void wallFollowRight() {
  if (!sensors.rightValid) {
    navMode = AUTONOMOUS;
    return;
  }
  
  int frontDist = getFusedFrontDistance();
  int rightDist = sensors.right;
  
  if (frontDist <= MIN_DISTANCE) {
    // Wall ahead - turn left
    gradientTurn(true);
    navMode = AUTONOMOUS;
  } else {
    // Adjust steering based on right wall distance
    int baseSpeed = calculateSpeedFromDistance(frontDist);
    int speedDiff = (rightDist - WALL_FOLLOW_DIST) * 2;  // Proportional control
    
    int leftSpeed = baseSpeed + speedDiff;
    int rightSpeed = baseSpeed - speedDiff;
    
    leftSpeed = constrain(leftSpeed, MIN_SPEED/2, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED/2, MAX_SPEED);
    
    setTargetSpeed(leftSpeed, rightSpeed);
  }
  
  // Exit wall following if no wall detected
  if (rightDist > SAFE_DISTANCE * 2) {
    navMode = AUTONOMOUS;
  }
}

void handleObstacleWithGradient() {
  // Stop and look around
  setTargetSpeed(0, 0);
  delay(200);
  
  // Move back slightly
  setTargetSpeed(-SLOW_SPEED, -SLOW_SPEED);
  delay(300);
  setTargetSpeed(0, 0);
  
  // Look left and right
  servoHead.write(SERVO_LEFT);
  delay(500);
  int leftDist = getDistanceWithTimeout(TRIG_FRONT, ECHO_FRONT);
  
  servoHead.write(SERVO_RIGHT);
  delay(500);
  int rightDist = getDistanceWithTimeout(TRIG_FRONT, ECHO_FRONT);
  
  servoHead.write(SERVO_CENTER);
  delay(300);
  
  // Decide direction based on gradient
  if (leftDist > rightDist && leftDist > MIN_DISTANCE) {
    gradientTurn(true);  // Turn left
  } else if (rightDist > MIN_DISTANCE) {
    gradientTurn(false); // Turn right
  } else {
    // Both sides blocked - activate escape mode
    navMode = ESCAPE_MODE;
  }
}

void gradientTurn(bool turnLeft) {
  int outerSpeed = TURN_SPEED;
  int innerSpeed = TURN_SPEED / 3;
  
  if (turnLeft) {
    setTargetSpeed(-innerSpeed, outerSpeed);
  } else {
    setTargetSpeed(outerSpeed, -innerSpeed);
  }
  
  delay(TURN_TIME);
  setTargetSpeed(0, 0);
}

void escapeMode() {
  // 180-degree turn and switch back to autonomous
  setTargetSpeed(-TURN_SPEED, TURN_SPEED);
  delay(TURN_TIME * 2);
  setTargetSpeed(0, 0);
  navMode = AUTONOMOUS;
}

// ============= PATH MEMORY SYSTEM =============

void initializePathMemory() {
  for (int i = 0; i < MAX_PATH_MEMORY; i++) {
    pathMemory[i].frontDist = 0;
    pathMemory[i].leftDist = 0;
    pathMemory[i].rightDist = 0;
    pathMemory[i].timestamp = 0;
  }
  pathIndex = 0;
  stuckCounter = 0;
}

void updatePathMemory() {
  pathMemory[pathIndex].frontDist = sensors.front;
  pathMemory[pathIndex].leftDist = sensors.left;
  pathMemory[pathIndex].rightDist = sensors.right;
  pathMemory[pathIndex].timestamp = millis();
  
  pathIndex = (pathIndex + 1) % MAX_PATH_MEMORY;
  
  // Check if stuck in similar positions
  checkIfStuck();
}

void checkIfStuck() {
  int similarPositions = 0;
  int currentFront = sensors.front;
  int currentLeft = sensors.left;
  int currentRight = sensors.right;
  
  for (int i = 0; i < MAX_PATH_MEMORY; i++) {
    if (pathMemory[i].timestamp > 0) {
      int frontDiff = abs(pathMemory[i].frontDist - currentFront);
      int leftDiff = abs(pathMemory[i].leftDist - currentLeft);
      int rightDiff = abs(pathMemory[i].rightDist - currentRight);
      
      if (frontDiff < 5 && leftDiff < 5 && rightDiff < 5) {
        similarPositions++;
      }
    }
  }
  
  if (similarPositions >= STUCK_THRESHOLD) {
    stuckCounter++;
    if (stuckCounter > 2) {
      navMode = ESCAPE_MODE;
      stuckCounter = 0;
      Serial.println("STUCK DETECTED - Activating escape mode");
    }
  } else {
    stuckCounter = max(0, stuckCounter - 1);
  }
}

// ============= COMMUNICATION SYSTEM =============

void handleSerialInput() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  
  if (input.length() == 0) return;
  
  char cmd = input.charAt(0);
  
  // Parse parameters if available
  int param1 = 0, param2 = 0;
  if (input.length() > 1) {
    int spaceIndex = input.indexOf(' ', 1);
    if (spaceIndex > 0) {
      param1 = input.substring(1, spaceIndex).toInt();
      param2 = input.substring(spaceIndex + 1).toInt();
    } else {
      param1 = input.substring(1).toInt();
    }
  }
  
  processCommand(cmd, param1, param2);
}

void processCommand(char cmd, int param1, int param2) {
  switch (cmd) {
    case 'F': // Forward
      setTargetSpeed(param1 > 0 ? param1 : CRUISE_SPEED, param1 > 0 ? param1 : CRUISE_SPEED);
      break;
    case 'B': // Backward
      setTargetSpeed(param1 > 0 ? -param1 : -CRUISE_SPEED, param1 > 0 ? -param1 : -CRUISE_SPEED);
      break;
    case 'L': // Left
      gradientTurn(true);
      break;
    case 'R': // Right
      gradientTurn(false);
      break;
    case 'S': // Stop
      setTargetSpeed(0, 0);
      break;
    case 'W': // Wall follow mode
      if (param1 == 1) navMode = WALL_FOLLOW_LEFT;
      else if (param1 == 2) navMode = WALL_FOLLOW_RIGHT;
      else navMode = AUTONOMOUS;
      Serial.println("Navigation mode changed");
      break;
    case 'A': // Autonomous mode
      navMode = AUTONOMOUS;
      Serial.println("Autonomous mode activated");
      break;
    case 'V': // Variable speed
      if (param1 > 0 && param1 <= 255) {
        setTargetSpeed(param1, param2 > 0 ? param2 : param1);
      }
      break;
    case 'D': // Diagonal movement
      if (param1 == 1) setTargetSpeed(CRUISE_SPEED * 0.7, CRUISE_SPEED); // Forward-right
      else if (param1 == 2) setTargetSpeed(CRUISE_SPEED, CRUISE_SPEED * 0.7); // Forward-left
      else if (param1 == 3) setTargetSpeed(-CRUISE_SPEED * 0.7, -CRUISE_SPEED); // Back-right
      else if (param1 == 4) setTargetSpeed(-CRUISE_SPEED, -CRUISE_SPEED * 0.7); // Back-left
      break;
    case 'Q': // Queue command
      queueCommand(param1, param2, 0, millis() + 1000);
      Serial.println("Command queued");
      break;
    case '?': // Status request
      printDetailedStatus();
      break;
    default:
      Serial.println("Unknown command");
      break;
  }
}

void queueCommand(char cmd, int param1, int param2, unsigned long executeTime) {
  if (queueCount < QUEUE_SIZE) {
    commandQueue[queueTail].cmd = cmd;
    commandQueue[queueTail].param1 = param1;
    commandQueue[queueTail].param2 = param2;
    commandQueue[queueTail].executeTime = executeTime;
    
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount++;
  }
}

void processCommandQueue() {
  if (queueCount > 0 && millis() >= commandQueue[queueHead].executeTime) {
    Command cmd = commandQueue[queueHead];
    processCommand(cmd.cmd, cmd.param1, cmd.param2);
    
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount--;
  }
}

// ============= STATUS REPORTING =============

void printStatus() {
  Serial.print("Status: Front=");
  Serial.print(sensors.front);
  Serial.print("cm Left=");
  Serial.print(sensors.left);
  Serial.print("cm Right=");
  Serial.print(sensors.right);
  Serial.print("cm Speed=");
  Serial.print(currentLeftSpeed);
  Serial.print(",");
  Serial.print(currentRightSpeed);
  Serial.print(" Mode=");
  Serial.print(navMode);
  Serial.print(" Queue=");
  Serial.println(queueCount);
}

void printDetailedStatus() {
  Serial.println("=== DETAILED STATUS ===");
  Serial.print("Sensors - Front: ");
  Serial.print(sensors.front);
  Serial.print("cm (");
  Serial.print(sensors.frontValid ? "OK" : "FAIL");
  Serial.print(") Left: ");
  Serial.print(sensors.left);
  Serial.print("cm (");
  Serial.print(sensors.leftValid ? "OK" : "FAIL");
  Serial.print(") Right: ");
  Serial.print(sensors.right);
  Serial.print("cm (");
  Serial.print(sensors.rightValid ? "OK" : "FAIL");
  Serial.println(")");
  
  Serial.print("Motors - Current: ");
  Serial.print(currentLeftSpeed);
  Serial.print(",");
  Serial.print(currentRightSpeed);
  Serial.print(" Target: ");
  Serial.print(targetLeftSpeed);
  Serial.print(",");
  Serial.println(targetRightSpeed);
  
  Serial.print("Navigation - Mode: ");
  Serial.print(navMode);
  Serial.print(" Stuck Counter: ");
  Serial.println(stuckCounter);
  
  Serial.print("Queue - Count: ");
  Serial.print(queueCount);
  Serial.print(" Head: ");
  Serial.print(queueHead);
  Serial.print(" Tail: ");
  Serial.println(queueTail);
  Serial.println("========================");
}

// Legacy functions for compatibility
void stopMotors() {
  setTargetSpeed(0, 0);
}

void moveForward() {
  setTargetSpeed(CRUISE_SPEED, CRUISE_SPEED);
}

void moveBackward() {
  setTargetSpeed(-CRUISE_SPEED, -CRUISE_SPEED);
}

void turnLeft() {
  gradientTurn(true);
}

void turnRight() {
  gradientTurn(false);
}
