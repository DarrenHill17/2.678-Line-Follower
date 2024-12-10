const int PWMA = 11;  // Pololu drive A
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;  // Pololu drive B
const int BIN2 = 6;
const int PWMB = 5;

const int defaultSpeed = 150;  // Base speed for straights
const int maxSpeed = 255;      // Maximum motor speed for straights
const int turnSpeed = 100;     // Reduced speed for tight turns

float Kp = 1.5;                // Proportional constant
float Ki = 0.01;               // Integral constant
float Kd = 0.7;                // Derivative constant

int previousError = 0;         // Error from the previous loop
float integral = 0;            // Integral accumulator
unsigned long lastInfiniteLoopCheck = 0;  // Timer for detecting infinite loops
int infiniteLoopCounter = 0;   // Counter for infinite loop detection

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);  // Enable motor driver
  Serial.begin(9600);
}

void loop() {
  int sensorReadings[3];
  getSensorReadings(sensorReadings);

  int error = calculateError(sensorReadings);
  int correction = calculatePIDCorrection(error);

  // Adjust speed based on error magnitude
  int speedAdjustment = max(0, defaultSpeed - abs(error) * 2);
  int leftSpeed = defaultSpeed - correction - speedAdjustment;
  int rightSpeed = defaultSpeed + correction - speedAdjustment;

  // Drive motors with adjusted speeds
  drive(leftSpeed, rightSpeed);

  // Handle infinite loops
  handleInfiniteLoops();

  delay(10);  // 100 updates per second
}

// Get sensor readings
void getSensorReadings(int readings[]) {
  readings[0] = analogRead(A3);  // Left sensor
  readings[1] = analogRead(A4);  // Center sensor
  readings[2] = analogRead(A5);  // Right sensor
}

// Calculate the error for PID control
int calculateError(int sensorReadings[]) {
  int threshold = 700;  // Adjustable threshold for light/dark classification
  bool leftDark = sensorReadings[0] > threshold;
  bool centerDark = sensorReadings[1] > threshold;
  bool rightDark = sensorReadings[2] > threshold;

  // Offset line-following strategy (favor the right edge of the line)
  if (rightDark) {
    return 10;  // Right sensor sees the line
  } else if (centerDark) {
    return 0;   // Center sensor aligned
  } else if (leftDark) {
    return -10; // Left sensor sees the line
  } else {
    return 0;   // All sensors white, assume straight line
  }
}

// Calculate PID correction
int calculatePIDCorrection(int error) {
  float dt = 0.01;  // Time step (10 ms)
  integral += error * dt;
  float derivative = (error - previousError) / dt;

  int correction = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  return correction;
}

// Handle infinite loops by detecting lack of progress
void handleInfiniteLoops() {
  unsigned long currentTime = millis();
  if (currentTime - lastInfiniteLoopCheck > 2000) {  // Check every 2 seconds
    infiniteLoopCounter++;
    if (infiniteLoopCounter > 3) {  // Trigger corrective action after 3 loops
      drive(-turnSpeed, turnSpeed);  // Sharp turn to escape loop
      delay(500);  // Wait to complete turn
      infiniteLoopCounter = 0;  // Reset counter
    }
    lastInfiniteLoopCheck = currentTime;
  }
}

// Drive the robot using left and right motor speeds
void drive(int speedL, int speedR) {
  motorWrite(speedL, AIN1, AIN2, PWMA);  // Left motor
  motorWrite(speedR, BIN1, BIN2, PWMB);  // Right motor
}

// Control motor speed and direction
void motorWrite(int spd, int pin_IN1, int pin_IN2, int pin_PWM) {
  if (spd < 0) {
    digitalWrite(pin_IN1, HIGH);  // Go one way
    digitalWrite(pin_IN2, LOW);
  } else {
    digitalWrite(pin_IN1, LOW);  // Go the other way
    digitalWrite(pin_IN2, HIGH);
  }
  analogWrite(pin_PWM, abs(spd));  // Set speed
}
