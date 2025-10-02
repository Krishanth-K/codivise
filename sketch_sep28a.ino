// Pin definitions for motor controllers (no PWM, enable pins jumpered)
// Controller 1 (pins 14, 27, 26, 25) now controls right side
// Controller 2 (pins 22, 23, 19, 18) now controls left side
#define IN1_RIGHT 27  // Direction pin 1 for right motor 1 (Controller 1)
#define IN2_RIGHT 14  // Direction pin 2 for right motor 1 (Controller 1)
#define IN3_RIGHT 25  // Direction pin 1 for right motor 2 (Controller 1)
#define IN4_RIGHT 26  // Direction pin 2 for right motor 2 (Controller 1)
#define IN1_LEFT 33   // Direction pin 1 for left motor 1 (Controller 2)
#define IN2_LEFT 32   // Direction pin 2 for left motor 1 (Controller 2)
#define IN3_LEFT 22   // Direction pin 1 for left motor 2 (Controller 2)
#define IN4_LEFT 23   // Direction pin 2 for left motor 2 (Controller 2)

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(IN3_RIGHT, OUTPUT);
  pinMode(IN4_RIGHT, OUTPUT);
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN3_LEFT, OUTPUT);
  pinMode(IN4_LEFT, OUTPUT);

  // Stop motors initially
  stopMotors();
  Serial.println("Starting autonomous car sequence...");
}

void loop() {
  // Move forward for 2 seconds
  Serial.println("Moving Forward");
  moveForward();
  delay(2000);

  // Turn right for 1 second
  Serial.println("Turning Right");
  turnRight();
  delay(1000);

  // Turn left for 1 second
  Serial.println("Turning Left");
  turnLeft();
  delay(1000);

  // Move backward for 2 seconds
  Serial.println("Moving Backward");
  moveBackward();
  delay(2000);

  // Stop motors
  Serial.println("Stopped");
  stopMotors();

  // Pause for 2 seconds before repeating
  delay(2000);
}

// Motor control functions
void moveForward() {
  // Left motors forward (Controller 2)
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_LEFT, HIGH);
  digitalWrite(IN4_LEFT, LOW);

  // Right motors forward (Controller 1)
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN3_RIGHT, HIGH);
  digitalWrite(IN4_RIGHT, LOW);
}

void moveBackward() {
  // Left motors backward (Controller 2)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN3_LEFT, LOW);
  digitalWrite(IN4_LEFT, HIGH);

  // Right motors backward (Controller 1)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, HIGH);
}

void turnLeft() {
  // Left motors backward (Controller 2)
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, HIGH);
  digitalWrite(IN3_LEFT, LOW);
  digitalWrite(IN4_LEFT, HIGH);

  // Right motors forward (Controller 1)
  digitalWrite(IN1_RIGHT, HIGH);
  digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN3_RIGHT, HIGH);
  digitalWrite(IN4_RIGHT, LOW);
}

void turnRight() {
  // Left motors forward (Controller 2)
  digitalWrite(IN1_LEFT, HIGH);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_LEFT, HIGH);
  digitalWrite(IN4_LEFT, LOW);

  // Right motors backward (Controller 1)
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, HIGH);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, HIGH);
}

void stopMotors() {
  // Stop all motors
  digitalWrite(IN1_LEFT, LOW);
  digitalWrite(IN2_LEFT, LOW);
  digitalWrite(IN3_LEFT, LOW);
  digitalWrite(IN4_LEFT, LOW);
  digitalWrite(IN1_RIGHT, LOW);
  digitalWrite(IN2_RIGHT, LOW);
  digitalWrite(IN3_RIGHT, LOW);
  digitalWrite(IN4_RIGHT, LOW);
}