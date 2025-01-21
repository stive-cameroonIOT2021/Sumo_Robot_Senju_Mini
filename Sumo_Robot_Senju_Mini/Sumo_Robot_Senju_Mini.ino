#define leftMotorPWM1Pin 3
#define leftMotorPWM2Pin 11
#define rightMotorPWM1Pin 9
#define rightMotorPWM2Pin 10

#define leftLineSensorPin A5
#define rightLineSensorPin A4
#define leftObstacleSensorPin A3
#define rightObstacleSensorPin A2

#define ledBuzzerPin 13
#define startButtonPin 6
#define potentiometerPin A6
#define irSensorPin 2

#define modeSwitch1Pin 12
#define modeSwitch2Pin 8
#define modeSwitch3Pin 7

int lineSensorThreshold = 100;
int defaultSpeed = 20;
int irSensorState = 0;
int maximumMotorSpeed = 102;
int lastDirection = 5;

void setup() {
  Serial.begin(9600);

  pinMode(leftMotorPWM1Pin, OUTPUT);
  pinMode(leftMotorPWM2Pin, OUTPUT);
  pinMode(rightMotorPWM1Pin, OUTPUT);
  pinMode(rightMotorPWM2Pin, OUTPUT);

  pinMode(leftObstacleSensorPin, INPUT_PULLUP);
  pinMode(rightObstacleSensorPin, INPUT_PULLUP);
  pinMode(leftLineSensorPin, INPUT_PULLUP);
  pinMode(rightLineSensorPin, INPUT_PULLUP);

  pinMode(ledBuzzerPin, OUTPUT);
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(potentiometerPin, INPUT_PULLUP);
  pinMode(irSensorPin, INPUT);

  pinMode(modeSwitch1Pin, INPUT_PULLUP);
  pinMode(modeSwitch2Pin, INPUT_PULLUP);
  pinMode(modeSwitch3Pin, INPUT_PULLUP);

  digitalWrite(ledBuzzerPin, LOW);
  digitalWrite(leftMotorPWM1Pin, LOW);
  digitalWrite(leftMotorPWM2Pin, LOW);
  digitalWrite(rightMotorPWM1Pin, LOW);
  digitalWrite(rightMotorPWM2Pin, LOW);

  tone(ledBuzzerPin, 200, 350);
  delay(300);
  noTone(ledBuzzerPin);

  tone(ledBuzzerPin, 300, 200);
  tone(ledBuzzerPin, 4000, 500);
  noTone(ledBuzzerPin);

  //calibrateLineSensors();
}

void loop() {
Wait:
  //lineSensorThreshold = analogRead(potentiometerPin);

  // Line detection and buzzer alert
  if (analogRead(leftLineSensorPin) < lineSensorThreshold || analogRead(rightLineSensorPin) < lineSensorThreshold) {
    tone(ledBuzzerPin, 300, 10);
  } else if (digitalRead(leftObstacleSensorPin) == LOW || digitalRead(rightObstacleSensorPin) == LOW) {
    digitalWrite(ledBuzzerPin, HIGH);
  } else {
    noTone(ledBuzzerPin);
    digitalWrite(ledBuzzerPin, LOW);
  }

  // Start via IR sensor
  if (digitalRead(irSensorPin) == 1) {
    irSensorState = 1;
    handleOperatingMode();
    noTone(ledBuzzerPin);
    goto Start;
  }

  // Start via button
  if (digitalRead(startButtonPin) == 0) {
    for (int i = 0; i < 6; i++) {
      controlMotors(0, 0, 1);
      tone(ledBuzzerPin, 523, 200);
      delay(200);
      noTone(ledBuzzerPin);
      delay(600);
    }
    handleOperatingMode();
    noTone(ledBuzzerPin);
    goto Start;
  }

  goto Wait;

Start:
  // Exit control
  if (digitalRead(irSensorPin) == 0 && irSensorState == 1) {
    controlMotors(0, 0, 1);
    irSensorState = 0;
    goto Wait;
  }

  if (analogRead(leftLineSensorPin) > 900 && analogRead(rightLineSensorPin) > 900) {
    // Stay inside control
    if (digitalRead(leftObstacleSensorPin) == LOW && digitalRead(rightObstacleSensorPin) == LOW) {
      controlMotors(maximumMotorSpeed, maximumMotorSpeed, 1);
    } else if (digitalRead(leftObstacleSensorPin) == LOW && digitalRead(rightObstacleSensorPin) == HIGH) {
      controlMotors(maximumMotorSpeed, maximumMotorSpeed, 1);
      lastDirection = 7;
    } else if (digitalRead(leftObstacleSensorPin) == HIGH && digitalRead(rightObstacleSensorPin) == LOW) {
      controlMotors(maximumMotorSpeed, maximumMotorSpeed, 1);
      lastDirection = 3;
    } else {
      if (lastDirection == 7) {
        controlMotors(-70, 70, 2);
      } else if (lastDirection == 3) {
        controlMotors(70, -70, 2);
      }
    }
  } else {
    if (analogRead(leftLineSensorPin) < lineSensorThreshold && analogRead(rightLineSensorPin) > 900) {
      controlMotors(-102, -102, 150);
      controlMotors(102, -102, 100);
    } else if (analogRead(leftLineSensorPin) > 900 && analogRead(rightLineSensorPin) < lineSensorThreshold) {
      controlMotors(-102, -102, 150);
      controlMotors(-102, 102, 100);
    } else if (analogRead(leftLineSensorPin) < lineSensorThreshold && analogRead(rightLineSensorPin) < lineSensorThreshold) {
      if (lastDirection == 3) {
        controlMotors(-102, -102, 150);
        controlMotors(102, -102, 100);
      } else if (lastDirection == 7) {
        controlMotors(-102, -102, 150);
        controlMotors(-102, 102, 100);
      }
    }
  }

  goto Start;
}

void controlMotors(float leftMotorSpeed, float rightMotorSpeed, int duration) {
  leftMotorSpeed = leftMotorSpeed * 2.5;
  rightMotorSpeed = rightMotorSpeed * 2.5;

  if (leftMotorSpeed >= 0) {
    digitalWrite(leftMotorPWM1Pin, LOW);
    analogWrite(leftMotorPWM2Pin, leftMotorSpeed);
  } else {
    leftMotorSpeed = abs(leftMotorSpeed);
    analogWrite(leftMotorPWM1Pin, leftMotorSpeed);
    digitalWrite(leftMotorPWM2Pin, LOW);
  }

  if (rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWM1Pin, rightMotorSpeed);
    digitalWrite(rightMotorPWM2Pin, LOW);
  } else {
    rightMotorSpeed = abs(rightMotorSpeed);
    digitalWrite(rightMotorPWM1Pin, LOW);
    analogWrite(rightMotorPWM2Pin, rightMotorSpeed);
  }
  delay(duration);
}

void handleOperatingMode() {
  if (digitalRead(modeSwitch1Pin) == 0 && digitalRead(modeSwitch2Pin) == 1 && digitalRead(modeSwitch3Pin) == 1) {
    controlMotors(-40, 40, 120);
    controlMotors(40, 40, 20);
    lastDirection = 7;
  } else if (digitalRead(modeSwitch1Pin) == 0 && digitalRead(modeSwitch2Pin) == 0 && digitalRead(modeSwitch3Pin) == 1) {
    controlMotors(50, 80, 170);
    lastDirection = 7;
  } else if (digitalRead(modeSwitch1Pin) == 1 && digitalRead(modeSwitch2Pin) == 1 && digitalRead(modeSwitch3Pin) == 0) {
    controlMotors(100, -100, 120);
    controlMotors(80, 80, 20);
    lastDirection = 3;
  } else if (digitalRead(modeSwitch1Pin) == 1 && digitalRead(modeSwitch2Pin) == 0 && digitalRead(modeSwitch3Pin) == 0) {
    controlMotors(80, 80, 170);
    lastDirection = 3;
  }
}

void calibrateLineSensors() {
  int minSensorValue = 1023, maxSensorValue = 0;

  for (int i = 0; i < 100; i++) {
    int currentSensorValue = analogRead(leftLineSensorPin);
    if (currentSensorValue > maxSensorValue) maxSensorValue = currentSensorValue;
    if (currentSensorValue < minSensorValue) minSensorValue = currentSensorValue;
    delay(10);
  }

  lineSensorThreshold = (minSensorValue + maxSensorValue) / 2;
  Serial.print("Line Sensor Threshold Value: ");
  Serial.println(lineSensorThreshold);
}
