const int PWMA = 11;  // Pololu drive A
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;  // Pololu drive B
const int BIN2 = 6;
const int PWMB = 5;

const int defaultSpeed = 100;

const int rememberedReadings = 20; // Number of sensor readings used to calculate the I term for PID

int previousSensorReadings[rememberedReadings][3]; // Store previous sensor readings
int numRows = 0; // Current number of rows in the readings

int previousSensor[3];

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
  getSensorReadings(sensorReadings);

  // Update previous sensor readings if needed
  updatePreviousSensorReadings(sensorReadings, 3);

  // Line Following
  int correction = getCorrection(sensorReadings);
  drive(-(defaultSpeed + correction), defaultSpeed - correction);

  delay(10); // 100 updates per second
}

// Get correction for line following
int getCorrection(int sensorReadings[]) {
  float Kp = 1; // Proportional constant
  int threshold = 700; // Adjustable threshold for classifying light vs dark

  // Determine if the sensors detect light or dark
  bool leftDark = sensorReadings[0] <= threshold;
  bool centerDark = sensorReadings[1] <= threshold;
  bool rightDark = sensorReadings[2] <= threshold;

  int values[] = {(int) leftDark, (int) centerDark, (int) rightDark};

  int error = 0;

  if ((values[0] == 1 && values[1] == 1 && values[2] == 1) ||
      (values[0] == 0 && values[1] == 0 && values[2] == 0)) {
    error = 0; // All sensors detect the same, no correction
  } 
  else if (previousSensorReadings[0] == 1 && previousSensorReadings[1] == 1 && previousSensorReadings[2] == 0 &&
           values[0] == 1 && values[1] == 0 && values[2] == 0) {
    error = 100; // Steer right
  } 
  else if (previousSensorReadings[0] == 0 && previousSensorReadings[1] == 1 && previousSensorReadings[2] == 1 &&
           values[0] == 0 && values[1] == 0 && values[2] == 1) {
    error = -100; // Steer left
  }

  for (int i = 0; i < 3; i++) {
    previousSensor[i] = values[i];
  }

  return Kp * error; // Return the correction value
}

// Get sensor readings (fill the provided array)
void getSensorReadings(int readings[]) {
  readings[0] = analogRead(A3); // Left sensor
  readings[1] = analogRead(A4); // Center sensor
  readings[2] = analogRead(A5); // Right sensor
}

// Update previous sensor readings for PID or tracking
void updatePreviousSensorReadings(int sensorReadings[], int rowSize) {
  if (numRows < rememberedReadings) {
    // Add a new row
    for (int j = 0; j < rowSize; j++) {
      previousSensorReadings[numRows][j] = sensorReadings[j];
    }
    numRows++;
  } else {
    // Shift rows to make space for the new one
    for (int i = 1; i < rememberedReadings; i++) {
      for (int j = 0; j < rowSize; j++) {
        previousSensorReadings[i - 1][j] = previousSensorReadings[i][j];
      }
    }
    // Add the new row at the end
    for (int j = 0; j < rowSize; j++) {
      previousSensorReadings[rememberedReadings - 1][j] = sensorReadings[j];
    }
  }
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
