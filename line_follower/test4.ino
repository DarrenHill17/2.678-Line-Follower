const int PWMA = 11;  // Pololu drive A
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;   // Pololu drive B
const int BIN2 = 6;
const int PWMB = 5;

const int defaultSpeed = 50;

int previousSensor[3] = {0, 0, 0}; // Initialize previous sensor readings

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH); // Enable the motor driver

  Serial.begin(9600);
}

void loop() {
  int sensorReadings[3];

  // Read sensor values
  getSensorReadings(sensorReadings);

  // Calculate correction
  int correction = getCorrection(sensorReadings, previousSensor);

  // Drive the robot based on correction
  drive(-(defaultSpeed + correction), defaultSpeed - correction);

  // Update previousSensor with current sensor readings
  for (int i = 0; i < 3; i++) {
    previousSensor[i] = sensorReadings[i];
  }

  delay(50); // 20 updates per second
}

// Get correction for line following
int getCorrection(int sensorReadings[], int previousSensor[]) {
  float Kp = 1;       // Proportional constant
  int threshold = 700; // Adjustable threshold for light vs dark

  // Determine if sensors detect light or dark
  bool leftLight = sensorReadings[0] <= threshold;
  bool centerLight = sensorReadings[1] <= threshold;
  bool rightLight = sensorReadings[2] <= threshold;

  int error = 0;

  if ((sensorReadings[0] == 1 && sensorReadings[1] == 1 && sensorReadings[2] == 1) ||
      (sensorReadings[0] == 0 && sensorReadings[1] == 0 && sensorReadings[2] == 0)) {
    error = 0; // All sensors detect the same, no correction
  } 
  else if (previousSensor[0] == 1 && previousSensor[1] == 1 && previousSensor[2] == 0 &&
           sensorReadings[0] == 1 && sensorReadings[1] == 0 && sensorReadings[2] == 0) {
    error = 100; // Steer right
  } 
  else if (previousSensor[0] == 0 && previousSensor[1] == 1 && previousSensor[2] == 1 &&
           sensorReadings[0] == 0 && sensorReadings[1] == 0 && sensorReadings[2] == 1) {
    error = -100; // Steer left
  } else if (centerLight) {
    error = 0; // Drive straight if center sensor detects light
  }

  return Kp * error; // Return correction
}

// Get sensor readings (fill the provided array)
void getSensorReadings(int readings[]) {
  readings[0] = analogRead(A3) > 700 ? 1 : 0; // Left sensor
  readings[1] = analogRead(A4) > 700 ? 1 : 0; // Center sensor
  readings[2] = analogRead(A5) > 700 ? 1 : 0; // Right sensor
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
  analogWrite(pin_PWM, abs(spd)); // Set speed
}

// Drive the robot using left and right motor speeds
void drive(int speedL, int speedR) {
  motorWrite(speedL, AIN1, AIN2, PWMA); // Left motor
  motorWrite(speedR, BIN1, BIN2, PWMB); // Right motor
}
