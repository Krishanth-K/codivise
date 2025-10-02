// Pin definitions for motor controllers (no PWM, enable pins jumpered)
// Controller 1 (pins 14, 27, 26, 25) controls right side
// Controller 2 (pins 22, 23, 19, 18) controls left side
#define IN1_RIGHT 27  // Direction pin 1 for right motor 1 (Controller 1)
#define IN2_RIGHT 14  // Direction pin 2 for right motor 1 (Controller 1)
#define IN3_RIGHT 25  // Direction pin 1 for right motor 2 (Controller 1)
#define IN4_RIGHT 26  // Direction pin 2 for right motor 2 (Controller 1)
#define IN1_LEFT 33   // Direction pin 1 for left motor 1 (Controller 2)
#define IN2_LEFT 32   // Direction pin 2 for left motor 1 (Controller 2)
#define IN3_LEFT 22   // Direction pin 1 for left motor 2 (Controller 2)
#define IN4_LEFT 23   // Direction pin 2 for left motor 2 (Controller 2)

// Pin definitions for ultrasonic sensors (two front, one back)
#define TRIG_FRONT_LEFT 2   // Front-left sensor trigger pin
#define ECHO_FRONT_LEFT 4   // Front-left sensor echo pin
#define TRIG_FRONT_RIGHT 13 // Front-right sensor trigger pin
#define ECHO_FRONT_RIGHT 12 // Front-right sensor echo pin
#define TRIG_BACK 18        // Back sensor trigger pin
#define ECHO_BACK 19         // Back sensor echo pin

// Obstacle detection threshold (in cm)
#define OBSTACLE_THRESHOLD 20

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Seed random number generator for random turns
  randomSeed(analogRead(0));

  // Set motor control pins as outputs
  pinMode(IN1_RIGHT, OUTPUT);
  pinMode(IN2_RIGHT, OUTPUT);
  pinMode(IN3_RIGHT, OUTPUT);
  pinMode(IN4_RIGHT, OUTPUT);
  pinMode(IN1_LEFT, OUTPUT);
  pinMode(IN2_LEFT, OUTPUT);
  pinMode(IN3_LEFT, OUTPUT);
  pinMode(IN4_LEFT, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_FRONT_LEFT, OUTPUT);
  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(TRIG_FRONT_RIGHT, OUTPUT);
  pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  // Stop motors initially
  stopMotors();
  Serial.println("Autonomous car obstacle avoidance started...");
}

void loop() {
  // Read distances from all sensors
  long distanceFrontLeft = getDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
  delay(10); // Small delay to reduce crosstalk
  long distanceFrontRight = getDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
  delay(10);
  long distanceBack = getDistance(TRIG_BACK, ECHO_BACK);

  // Print distances to Serial Monitor
  Serial.print("Front-Left: ");
  Serial.print(distanceFrontLeft);
  Serial.print(" cm | Front-Right: ");
  Serial.print(distanceFrontRight);
  Serial.print(" cm | Back: ");
  Serial.print(distanceBack);
  Serial.println(" cm");

  // Obstacle avoidance logic
  if (distanceFrontLeft > OBSTACLE_THRESHOLD && distanceFrontRight > OBSTACLE_THRESHOLD) {
    // No obstacle in front: move forward
    Serial.println("Moving Forward");
    moveForward();
  } else if (distanceFrontLeft <= OBSTACLE_THRESHOLD && distanceFrontRight > OBSTACLE_THRESHOLD) {
    // Obstacle on left: turn right
    Serial.println("Turning Right");
    turnRight();
    delay(1000); // Turn for 1 second
    stopMotors();
  } else if (distanceFrontRight <= OBSTACLE_THRESHOLD && distanceFrontLeft > OBSTACLE_THRESHOLD) {
    // Obstacle on right: turn left
    Serial.println("Turning Left");
    turnLeft();
    delay(1000); // Turn for 1 second
    stopMotors();
  } else if (distanceFrontLeft <= OBSTACLE_THRESHOLD && distanceFrontRight <= OBSTACLE_THRESHOLD) {
    // Obstacle directly in front: check back sensor
    if (distanceBack > OBSTACLE_THRESHOLD) {
      // Back is clear: move backward, then random turn
      Serial.println("Obstacle in front, moving backward");
      moveBackward();
      delay(1000); // Move backward for 1 second
      stopMotors();

      // Randomly choose left or right turn
      if (random(2) == 0) {
        Serial.println("Trying Left Turn");
        turnLeft();
      } else {
        Serial.println("Trying Right Turn");
        turnRight();
      }
      delay(1000); // Turn for 1 second
      stopMotors();
    } else {
      // Back obstacle detected: stop to avoid collision
      Serial.println("Obstacle in front and back, stopping");
      stopMotors();
    }
  }

  // Delay before next sensor reading
  delay(500);
}

// Function to measure distance using ultrasonic sensor
long getDistance(int trigPin, int echoPin) {
  // Send 10us pulse to trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse duration
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (~5m)

  // Calculate distance in cm (speed of sound = 343 m/s, or 0.0343 cm/us)
  if (duration == 0) {
    return -1; // Invalid reading (timeout or error)
  }
  long distance = duration * 0.0343 / 2;
  return distance;
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
