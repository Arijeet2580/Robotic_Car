/*
 * Autonomous Robot Car using L298N Motor Driver
 * Features:
 * - Obstacle avoidance
 * - Ultrasonic distance sensing
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

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 11

// Servo pin
#define SERVO_PIN 10

// Constants
#define MOTOR_SPEED    180  // Default motor speed (0-255)
#define TURN_SPEED     160  // Speed during turns
#define MIN_DISTANCE   10   // Minimum safe distance (cm)
#define SERVO_CENTER   90   // Center position for servo
#define SERVO_LEFT     180  // Leftmost servo angle
#define SERVO_RIGHT    0    // Rightmost servo angle
#define TURN_TIME      600  // Time for turning (ms)
#define RETRY_LIMIT    3    // Maximum retries for obstacle avoidance

// Create servo object
Servo servoHead;

// Variables
int currentDistance = 0;
int leftDistance = 0;
int rightDistance = 0;
char command;

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Configure motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configure ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the servo and set to center position
  servoHead.attach(SERVO_PIN);
  servoHead.write(SERVO_CENTER);

  // Ensure motors are stopped at startup
  stopMotors();

  Serial.println("Robot Car Ready!");
}

void loop() {
  if (Serial.available() > 0) {
    // Read command from Serial (Bluetooth or USB)
    command = Serial.read();
    handleCommand(command);
  } else {
    // Autonomous mode
    autonomousMode();
  }
}

void autonomousMode() {
  currentDistance = getDistance();

  if (currentDistance > MIN_DISTANCE) {
    moveForward();
  } else {
    handleObstacle();
  }
}

void handleObstacle() {
  stopMotors();
  moveBackward();
  delay(500);
  stopMotors();

  for (int retry = 0; retry < RETRY_LIMIT; retry++) {
    // Look left
    servoHead.write(SERVO_LEFT);
    delay(500);
    leftDistance = getDistance();

    // Look right
    servoHead.write(SERVO_RIGHT);
    delay(500);
    rightDistance = getDistance();

    // Return to center
    servoHead.write(SERVO_CENTER);
    delay(500);

    if (leftDistance > rightDistance && leftDistance > MIN_DISTANCE) {
      turnLeft();
      return;
    } else if (rightDistance > MIN_DISTANCE) {
      turnRight();
      return;
    }
  }

  // If retries exhausted, turn around
  turnLeft();
  turnLeft();
}

// Motor Control Functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  delay(TURN_TIME);
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  delay(TURN_TIME);
  stopMotors();
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Sensor Functions
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10μs pulse to trigger pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo time
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert to distance in cm
  return duration * 0.034 / 2;
}

// Command Handler
void handleCommand(char cmd) {
  switch (cmd) {
    case 'F': // Move forward
      moveForward();
      break;
    case 'B': // Move backward
      moveBackward();
      break;
    case 'L': // Turn left
      turnLeft();
      break;
    case 'R': // Turn right
      turnRight();
      break;
    case 'S': // Stop
    default:
      stopMotors();
      break;
  }
}
