/*
 * Autonomous Robot Car using L298N Motor Driver
 * Features:
 * - Obstacle avoidance
 * - Ultrasonic distance sensing
 */

#include <Servo.h>
#include <SoftwareSerial.h>

//Define RX and TX pins
#define BT_RX_PIN 2
#define BT_TX_PIN 3

// Create SoftwareSerial object for Bluetooth communication
SoftwareSerial bluetoothSerial(BT_RX_PIN, BT_TX_PIN); 
// Buffer for receiving data
const int BUFFER_SIZE = 64;
char dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;

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
#define TURN_TIME     600   // Time for turning (ms)

// Create servo object
Servo servoHead;

// Variables
int currentDistance = 0;
int leftDistance = 0;
int rightDistance = 0;
char recievedChar;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  //Initialise Bluetooth Serial
  bluetoothSerial.begin(9600);
  Serial.println("Bluetooth Communication Ready for Communication");
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
  
  // Initialize servo
  servoHead.attach(SERVO_PIN);
  servoHead.write(SERVO_CENTER);
  
  // Ensure motors are stopped at startup
  stopMotors();
  
  Serial.println("Robot Car Ready!");
}

void loop() {
  if(bluetoothSerial.available() >0){
    char recievedchar=bluetoothSerial.read();
    processmsg(recievedchar);
    delay(20);
  }
  else{
    autonomousMode();
  }
}

void processmsg(char recievedchar){
  if(recievedChar == '\n'){
    dataBuffer[bufferIndex]='\0'; //terminating the char array

    if(bufferIndex >0){
      handleCommand(dataBuffer);
    }

    bufferIndex=0; //Reseting the Buffer Index
  }
  else if(bufferIndex < BUFFER_SIZE-1){
    dataBuffer[bufferIndex]=recievedChar;
    bufferIndex++;
  }
}
void autonomousMode() {
  currentDistance = getDistance();
  
  if (currentDistance > MIN_DISTANCE) {
    moveForward();
  } else {
    // Obstacle detected - find best path
    handleObstacle();
  }
}

void handleObstacle() {
  stopMotors();
  moveBackward();
  delay(500);
  stopMotors();
  
  // Look left
  servoHead.write(180);
  delay(500);
  leftDistance = getDistance();
  
  // Look right
  servoHead.write(0);
  delay(500);
  rightDistance = getDistance();
  
  // Return to center
  servoHead.write(SERVO_CENTER);
  delay(500);
  
  // Choose better path
  if (leftDistance > rightDistance && leftDistance > MIN_DISTANCE) {
    turnLeft();
  } else if (rightDistance > MIN_DISTANCE) {
    turnRight();
  } else {
    // Both sides blocked - turn around
    turnLeft();
    turnLeft();
  }
}

// Motor Control Functions
void moveForward() {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void moveBackward() {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set speed
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnLeft() {
  // Left motor backward, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set turn speed
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  
  delay(TURN_TIME);
  stopMotors();
}

void turnRight() {
  // Left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Set turn speed
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  
  delay(TURN_TIME);
  stopMotors();
}

void stopMotors() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Sensor Functions
int getDistance() {
  // Clear trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10μs pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse duration
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Convert to distance in cm
  return duration * 0.034 / 2;
}

// Command Handler
void handleCommand(char cmd) {
  switch(cmd) {
    case 'F': // Forward
      moveForward();
      break;
    
    case 'B': // Backward
      moveBackward();
      break;
    
    case 'L': // Left
      turnLeft();
      break;
    
    case 'R': // Right
      turnRight();
      break;
    
    case 'S': // Stop
      stopMotors();
      break;
    
    default:
      stopMotors();
      break;
  }
}