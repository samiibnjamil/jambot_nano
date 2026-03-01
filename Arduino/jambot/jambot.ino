#include <PinChangeInterrupt.h>
#include <PID_v1.h>

#include "MPU6050.h"


// test sigh var
long prevT = 0;
float eprev = 0;
float eintegral = 0;
// Low-Pass Filter variables

double v1Filt = 0;
double v1Prev = 0;
double v2Filt = 0;
double v2Prev = 0;

// Create MPU6050 and KalmanFilter objects
MPU6050 mpu;

// RGB Color lamp control pin
#define CLED A0
#define MLED A1
#define YLED A2
// The buzzer controls the pin
#define BUZZER 11


// MPU6050 Gyroscope control pin
#define MPU_SCL A5
#define MPU_SDA A4
// Encoder pins
#define MOTOR1_ENCA 2
#define MOTOR1_ENCB 5
#define MOTOR2_ENCA 4
#define MOTOR2_ENCB 3

// TB6612 Control Pins
#define TB6612_STBY 8
#define TB6612_PWMA 10
#define TB6612_PWMB 9
#define TB6612_AIN1 12
#define TB6612_AIN2 13
#define TB6612_BIN1 7
#define TB6612_BIN2 6

#define CPR 404
#define WHEEL_DIAMETER_MM 68
#define MM_PER_COUNT (PI * WHEEL_DIAMETER_MM / CPR)

volatile long encoder1Count = 0;
volatile int encoder1Direction = 0;
volatile long encoder2Count = 0;
volatile int encoder2Direction = 0;

long lastMotor1Count = 0;
long lastMotor2Count = 0;

double motor1Speed = 0, motor2Speed = 0;
double targetRPM1 = 0, targetRPM2 = 0;
double outputPWM1 = 0, outputPWM2 = 0;
//double KpD = 0.6, KiD = 1.7, KdD =0.22;
double Kp = 0.6, Ki = 1.7, Kd = 0.001, Ko = 1.0;  // Define Ko globally;double Kp = 0.4, Ki = 1.5, Kd = 0.005, Ko = 1.0;  // Define Ko globally;
//double Kp = 0.36, Ki = 0.82, Kd = 0.000;
//double KpD = 1000.0, KiD = 5.0, KdD = 4.000;
bool pidEnabled = false;  // Add this near the top of your code
// PID motor1PID(&motor1Speed, &outputPWM1, &targetRPM1, Kp, Ki, Kd, DIRECT);
// PID motor2PID(&motor2Speed, &outputPWM2, &targetRPM2, Kp, Ki, Kd, DIRECT);


PID motor1PID(&v1Filt, &outputPWM1, &targetRPM1, Kp, Ki, Kd, DIRECT);
PID motor2PID(&v2Filt, &outputPWM2, &targetRPM2, Kp, Ki, Kd, DIRECT);

// double setpoint1 = 0;
// double setpoint2 = 0;
// double encoder1CountDouble = 0;
// double encoder2CountDouble = 0;

// PID motor1PositionPID(&encoder1CountDouble, &outputPWM1, &setpoint1, KpD, KiD, KdD, DIRECT);
// PID motor2PositionPID(&encoder2CountDouble, &outputPWM2, &setpoint2, KpD, KiD, KdD, DIRECT);

unsigned long lastDataReceivedTime = 0;  // Variable to store the last time data was received
const unsigned long dataTimeout = 2000; // 2 seconds timeout

void setup() {
  setupEncoders();
  setupMotorDriver();
  setupBuzzerLED();
  initMPU6050();
  Serial.begin(115200);
  initializePIDControllers();
  Serial.println("ready");
}

void loop() {
  if (pidEnabled) {
    speedPID(0);  // Compute PID outputs
  }
  processMotorCommands();
  //testSin();
  //motorStay();

}
void setupBuzzerLED() {
  digitalWrite(CLED, HIGH);
  digitalWrite(MLED, HIGH);
  digitalWrite(YLED, HIGH);

  // Configure pins as OUTPUT
  pinMode(CLED, OUTPUT);
  pinMode(MLED, OUTPUT);
  pinMode(YLED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
}
void initMPU6050() {
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;  // Halt execution if the connection fails
  }
  delay(1000);  // Give the sensor some time to stabilize
}


void setupEncoders() {
  pinMode(MOTOR1_ENCA, INPUT);
  pinMode(MOTOR1_ENCB, INPUT);
  pinMode(MOTOR2_ENCA, INPUT);
  pinMode(MOTOR2_ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCA), encoder1A, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(MOTOR1_ENCB), encoder1B, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(MOTOR2_ENCA), encoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCB), encoder2B, CHANGE);
}

void setupMotorDriver() {
  pinMode(TB6612_STBY, OUTPUT);
  pinMode(TB6612_PWMA, OUTPUT);
  pinMode(TB6612_PWMB, OUTPUT);
  pinMode(TB6612_AIN1, OUTPUT);
  pinMode(TB6612_AIN2, OUTPUT);
  pinMode(TB6612_BIN1, OUTPUT);
  pinMode(TB6612_BIN2, OUTPUT);
  digitalWrite(TB6612_STBY, HIGH);
}

void initializePIDControllers() {
  motor1PID.SetMode(AUTOMATIC);
  motor2PID.SetMode(AUTOMATIC);
  motor1PID.SetOutputLimits(-255, 255);
  motor2PID.SetOutputLimits(-255, 255);

  // motor1PositionPID.SetMode(MANUAL);
  // motor2PositionPID.SetMode(MANUAL);
  // motor1PositionPID.SetOutputLimits(-255, 255);
  // motor2PositionPID.SetOutputLimits(-255, 255);
}


void testSin() {
  pidEnabled = true;
  targetRPM1 = 250 * sin(prevT / 1e6);
  targetRPM2 = 250 * sin(prevT / 1e6);
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;
}

void speedPID(int printFlag) {
  static unsigned long lastTime = 0;
  if (millis() - lastTime >= 50) {
    long motor1Pulses = encoder1Count - lastMotor1Count;
    long motor2Pulses = encoder2Count - lastMotor2Count;

    motor1Speed = (motor1Pulses * 1200.0) / CPR;
    motor2Speed = (motor2Pulses * 1200.0) / CPR;

    // Apply Low-Pass Filter to RPM 25hz cukoff
    v1Filt = 0.854 * v1Filt + 0.0728 * motor1Speed + 0.0728 * v1Prev;
    v1Prev = motor1Speed;
    v2Filt = 0.854 * v2Filt + 0.0728 * motor2Speed + 0.0728 * v2Prev;
    v2Prev = motor2Speed;


    motor1PID.Compute();
    motor2PID.Compute();

    lastMotor1Count = encoder1Count;
    lastMotor2Count = encoder2Count;
    lastTime = millis();
  }

  applyMotorSpeed(outputPWM1 * Ko, outputPWM2 * Ko);

  if (printFlag == 1) {
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 100) {
      Serial.print(targetRPM1);
      Serial.print("\t");  // Tab separation
      //Serial.print(motor1Speed);
      //Serial.print("\t");
      Serial.print(v1Filt);
      Serial.print("\t");
      Serial.print(targetRPM2);
      Serial.print("\t");
      Serial.println(v2Filt);
      // Serial.print("\t");
      //Serial.println(motor2Speed);  // Newline at the end
      lastPrintTime = millis();
    }
  }
}

unsigned long lastBlinkTime = 0;
bool ledState = false;
const long blinkInterval = 500;  // Blinking interval in milliseconds

void processMotorCommands() {

  // Check if 2 seconds have passed since the last data was received
  if (millis() - lastDataReceivedTime > dataTimeout) {
    // Stop the motors
    rampDownMotors();
    pidEnabled = false;
    motor1PID.SetMode(MANUAL);  // Disable speed PID
    motor2PID.SetMode(MANUAL);  // Disable speed PID

    // Blink the red LED every 500 milliseconds
    if (millis() - lastBlinkTime >= blinkInterval) {
      lastBlinkTime = millis();  // Update the last blink time
      ledState = !ledState;       // Toggle LED state

      setRGBColor(ledState ? 1 : 0, ledState ? 1 : 0, 0);  // Toggle yellow LED (on or off)

    }
    // Optionally, you can print a message to indicate the timeout
    Serial.println("No data received for 2 seconds. Motors stopped and LED turned red.");
  }

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    char cmd = command.charAt(0);
    // Update the last data received time
    lastDataReceivedTime = millis();

    switch (cmd) {
      case 'b':
        {
          measureBatteryVoltage();



          break;
        }
      case 'e':
        {
          // Respond with current encoder counts
          Serial.print("e");
          Serial.print(" ");
          Serial.print(encoder1Count);
          Serial.print(" ");
          Serial.println(encoder2Count);
          break;
        }
      case 'i':
        {
          // Respond with current imu data
          sendimu();
          break;
        }
      case 'l':
        {
          // Update the RGB LED color
          int rgbIndex = command.indexOf(' ');
          if (rgbIndex > 0) {
            int nextSpaceIndex = command.indexOf(' ', rgbIndex + 1);
            int rIndex = command.substring(rgbIndex + 1, nextSpaceIndex).toInt();
            int gIndex = command.substring(nextSpaceIndex + 1, command.lastIndexOf(' ')).toInt();
            int bIndex = command.substring(command.lastIndexOf(' ') + 1).toInt();

            setRGBColor(rIndex, gIndex, bIndex);
            Serial.println("OK");
          }
          break;
        }
      case 'o':
        {
          // Set the raw PWM speed
          int pwmIndex = command.indexOf(' ');
          if (pwmIndex > 0) {
            int outputPWM11 = command.substring(pwmIndex + 1).toInt();
            int outputPWM21 = command.substring(command.lastIndexOf(' ') + 1).toInt();
            pidEnabled = false;
            motor1PID.SetMode(MANUAL);  // Disable speed PID
            motor2PID.SetMode(MANUAL);  // Disable speed PID
            // motor1PositionPID.SetMode(MANUAL);  // Disable position PID
            // motor2PositionPID.SetMode(MANUAL);  // Disable position PID
            
            if (outputPWM11 == 0 && outputPWM21 == 0) {
              Serial.println("ramping down");
              rampDownMotors();
            } else {
              applyMotorSpeed(outputPWM11, outputPWM21);
              outputPWM1=outputPWM11;
              outputPWM2=outputPWM21;
            }
            encoder1Count= 0;
            encoder2Count=0;
          
            Serial.println("OK");
          }
          break;
        }
      case 'm':
        {
          // Set the closed-loop speed
          int spdIndex = command.indexOf(' ');
          if (spdIndex > 0) {
            targetRPM1 = command.substring(spdIndex + 1).toInt();
            targetRPM2 = command.substring(command.lastIndexOf(' ') + 1).toInt();

            if (targetRPM1 == 0 && targetRPM2 == 0) {
              // Stop motors immediately and disable PID
              pidEnabled = false;
              motor1PID.SetMode(MANUAL);  // Disable speed PID
              motor2PID.SetMode(MANUAL);  // Disable speed PID
              // motor1PositionPID.SetMode(MANUAL);  // Disable position PID
              // motor2PositionPID.SetMode(MANUAL);  // Disable position PID
              rampDownMotors();
            } else {
              // Enable PID and reset controllers
              pidEnabled = true;
              motor1PID.SetMode(AUTOMATIC);
              motor2PID.SetMode(AUTOMATIC);
              // motor1PositionPID.SetMode(MANUAL);
              // motor2PositionPID.SetMode(MANUAL);
            }
            Serial.println("OK");
          }
          break;
        }

      case 'n':
        {
          // buzzer
          int buzzerIndex = command.indexOf(' ');
          if (buzzerIndex > 0) {
            int buzzerState = command.substring(buzzerIndex + 1).toInt();
            if (buzzerState == 1) {
              digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
            } else {
              digitalWrite(BUZZER, LOW);  // Turn off the buzzer
            }
            Serial.println("OK");
          }
          break;
        }
      case 'p':
        {
          // Update the PID parameters
          int kpIndex = command.indexOf(' ');
          if (kpIndex > 0) {
            int kdIndex = command.indexOf(' ', kpIndex + 1);
            int kiIndex = command.indexOf(' ', kdIndex + 1);
            int koIndex = command.indexOf(' ', kiIndex + 1);

            if (kdIndex > 0 && kiIndex > 0 && koIndex > 0) {
              Kp = command.substring(kpIndex + 1, kdIndex).toFloat();
              Kd = command.substring(kdIndex + 1, kiIndex).toFloat();
              Ki = command.substring(kiIndex + 1, koIndex).toFloat();
              Ko = command.substring(koIndex + 1).toFloat();

              Serial.println("OK");
            }
          }
          break;
        }
      // case 's':  // Set position setpoints
      //   {
      //     int spIndex = command.indexOf(' ');
      //     if (spIndex > 0) {
      //       setpoint1 = command.substring(spIndex + 1).toInt();
      //       setpoint2 = command.substring(command.lastIndexOf(' ') + 1).toInt();
      //       pidEnabled = true;  // Disable speed PID
      //       motor1PID.SetMode(MANUAL);
      //       motor2PID.SetMode(MANUAL);
      //       motor1PositionPID.SetMode(AUTOMATIC);
      //       motor2PositionPID.SetMode(AUTOMATIC);
      //       Serial.println("OK");
      //     }
      //     break;
      //   }
      case 'r':
        {
          // Reset encoder values
          encoder1Count = 0;
          encoder2Count = 0;
          Serial.println("OK");
          break;
        }
      case 'x':
        {
          // Reset encoder values
          pidEnabled = !pidEnabled;
          if (pidEnabled) {
            // Enable speed PID and disable position PID
            motor1PID.SetMode(AUTOMATIC);
            motor2PID.SetMode(AUTOMATIC);
            // motor1PositionPID.SetMode(MANUAL);
            // motor2PositionPID.SetMode(MANUAL);
          } else {
            // Disable speed PID and enable position PID
            motor1PID.SetMode(MANUAL);
            // motor2PID.SetMode(MANUAL);
            // motor1PositionPID.SetMode(AUTOMATIC);
            // motor2PositionPID.SetMode(AUTOMATIC);
          }
          Serial.println("OK");
          break;
        }
      default:
        {

          // Unrecognized command
          Serial.println("Invalid command");
          break;
        }
    }
  }
}



void setRGBColor(int red, int green, int blue) {
  // Convert RGB to CMY
  int cyan = 1 - red;
  int magenta = 1 - green;
  int yellow = 1 - blue;

  // Write CMY values to the LED pins
  digitalWrite(CLED, cyan);     // Cyan channel
  digitalWrite(MLED, magenta);  // Magenta channel
  digitalWrite(YLED, yellow);   // Yellow channel
}

void encoder1A() {
  if (digitalRead(MOTOR1_ENCA) == digitalRead(MOTOR1_ENCB)) {
    encoder1Direction = 1;
  } else {
    encoder1Direction = -1;
  }
  encoder1Count += encoder1Direction;
}

void encoder1B() {
  if (digitalRead(MOTOR1_ENCA) == digitalRead(MOTOR1_ENCB)) {
    encoder1Direction = -1;
  } else {
    encoder1Direction = 1;
  }
  encoder1Count += encoder1Direction;
}

void encoder2A() {
  if (digitalRead(MOTOR2_ENCA) == digitalRead(MOTOR2_ENCB)) {
    encoder2Direction = -1;
  } else {
    encoder2Direction = 1;
  }
  encoder2Count += encoder2Direction;
}

void encoder2B() {
  if (digitalRead(MOTOR2_ENCA) == digitalRead(MOTOR2_ENCB)) {
    encoder2Direction = 1;
  } else {
    encoder2Direction = -1;
  }
  encoder2Count += encoder2Direction;
}

void applyMotorSpeed(int speedA, int speedB) {
  if (speedA >= 0) {
    digitalWrite(TB6612_AIN1, HIGH);
    digitalWrite(TB6612_AIN2, LOW);
    analogWrite(TB6612_PWMA, speedA);
  } else {
    digitalWrite(TB6612_AIN1, LOW);
    digitalWrite(TB6612_AIN2, HIGH);
    analogWrite(TB6612_PWMA, -speedA);
  }

  if (speedB >= 0) {
    digitalWrite(TB6612_BIN1, HIGH);
    digitalWrite(TB6612_BIN2, LOW);
    analogWrite(TB6612_PWMB, speedB);
  } else {
    digitalWrite(TB6612_BIN1, LOW);
    digitalWrite(TB6612_BIN2, HIGH);
    analogWrite(TB6612_PWMB, -speedB);
  }
}

// void motorStay() {
//   motor1PositionPID.Compute();
//   motor2PositionPID.Compute();
//   applyMotorSpeed(outputPWM1 * Ko, outputPWM2 * Ko);
// }

void printDist() {
  float distance1 = encoder1Count * MM_PER_COUNT;
  float distance2 = encoder2Count * MM_PER_COUNT;
  Serial.print("  E1: ");
  Serial.print(encoder1Count);
  Serial.print(" Dir: ");
  Serial.print(encoder1Direction == -1 ? "FWD" : "REV");
  Serial.print(" Dist: ");
  Serial.print(distance1, 2);
  Serial.print("mm || ");
  Serial.print("E2: ");
  Serial.print(encoder2Count);
  Serial.print(" Dir: ");
  Serial.print(encoder2Direction == -1 ? "FWD" : "REV");
  Serial.print(" Dist: ");
  Serial.print(distance2, 2);
  Serial.println("mm");
}

void sendimu() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Create a buffer to hold the formatted string
  char buffer[50];  // Adjust size based on your data length

  // Format the data into the buffer
  sprintf(buffer, "i %d %d %d %d %d %d", ax, ay, az, gx, gy, gz);

  // Send the buffer over serial
  Serial.println(buffer);
}
// Variables for LED blinking
unsigned long lastBlinkTime2 = 0;
bool ledState2 = false;
const long blinkInterval2 = 200;  // Blinking interval in milliseconds

void measureBatteryVoltage() {
  const int analogPin = A7;  // Analog pin connected to the voltage divider
  const float R1 = 9500.0;   // Resistance of R1 in ohms
  const float R2 = 5200.0;   // Resistance of R2 in ohms
  const float vRef = 5.0;    // Reference voltage of the Arduino (5V)
  const int adcMax = 1023;   // Maximum value for the ADC (10-bit resolution)

  int analogValue = analogRead(analogPin);     // Read the analog value
  float vOut = (analogValue * vRef) / adcMax;  // Calculate the voltage at the analog pin
  float vIn = vOut * (R1 + R2) / R2;           // Calculate the battery voltage using the voltage divider formula
   // If battery percentage is below 10%, blink red LED
  if (vIn <= 9.9) {
    // Blink the red LED
    if (millis() - lastBlinkTime2 >= blinkInterval2) {
      lastBlinkTime2 = millis();  // Update the last blink time
      ledState2 = !ledState2;       // Toggle LED state
      setRGBColor(ledState2 ? 1 : 0, 0, 0);  // Blink red LED (on or off)
    }
  }
  
  Serial.print("b ");
  Serial.println(vIn);
}


void rampDownMotors() {
   Serial.print(outputPWM1);
   Serial.println(outputPWM2);
  int currentPWM1 = outputPWM1;
  int currentPWM2 = outputPWM2;
  while (currentPWM1 != 0 || currentPWM2 != 0) {
    if (currentPWM1 > 0) currentPWM1--;
    if (currentPWM1 < 0) currentPWM1++;
    if (currentPWM2 > 0) currentPWM2--;
    if (currentPWM2 < 0) currentPWM2++;
    applyMotorSpeed(currentPWM1, currentPWM2);
      Serial.print(currentPWM1);
    Serial.println(currentPWM2);
    delay(5);  // Adjust delay for smoother ramp-down
  }
   outputPWM1 = currentPWM1;
   outputPWM2 = currentPWM2;
}


void buzzerSound(int soundType) {
  switch (soundType) {
    case 1:  // Single short beep (fastest)
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(150);  // Short pause after the beep
      break;

    case 2:  // Double beep (slightly longer)
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(100);  // Short pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(200);  // Longer pause after the double beep
      break;

    case 3:  // Triple beep (even longer)
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(100);  // Short pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(100);  // Short pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(300);  // Longer pause after the triple beep
      break;

    case 4:  // Short beep with longer pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Beep for 100ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(500);  // Longer pause after the beep
      break;

    case 5:  // Beep pattern (short-long-short)
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Short beep
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(100);  // Short pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(300);  // Long beep
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(100);  // Short pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(100);  // Short beep
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(500);  // Longer pause after the pattern
      break;

    case 6:  // Slow beep pattern (long pauses)
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(200);  // Beep for 200ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(500);  // Long pause
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(200);  // Beep for 200ms
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(500);  // Long pause
      break;

    case 7:  // Very short beeps (rapid fire)
      for (int i = 0; i < 5; i++) {
        digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
        delay(50);  // Very short beep
        digitalWrite(BUZZER, LOW);  // Turn off the buzzer
        delay(50);  // Very short pause
      }
      delay(300);  // Longer pause after the rapid fire
      break;

    case 8:  // Single long beep (1 second)
      digitalWrite(BUZZER, HIGH);  // Turn on the buzzer
      delay(1000);  // Beep for 1 second
      digitalWrite(BUZZER, LOW);  // Turn off the buzzer
      delay(500);  // Longer pause after the long beep
      break;

    default:  // Invalid sound type
      Serial.println("Invalid sound type");
      break;
  }
}

// Function to implement different light patterns
void lightPattern(String pattern) {
  if (pattern == "breathing") {
    for (int i = 0; i < 5; i++) {  // Repeat 5 times
      // Breath in (turn LEDs on, then off in sequence)
      setRGBColor(1, 0, 0);  // Example: Red breathing (on)
      delay(1000);            // Wait for 1 second
      setRGBColor(0, 0, 0);  // Turn off LEDs (off)
      delay(1000);
    }
  }
  else if (pattern == "flashing") {
    for (int i = 0; i < 10; i++) {
      // Flash the red color quickly
      setRGBColor(1, 0, 0);  // Red color (on)
      delay(200);
      setRGBColor(0, 0, 0);  // Turn off LEDs (off)
      delay(200);
    }
  }
  else if (pattern == "fading") {
    for (int i = 0; i < 5; i++) {
      // Fade in and out with blue LEDs
      setRGBColor(0, 0, 1);  // Blue color (on)
      delay(1000);
      setRGBColor(0, 0, 0);  // Turn off LEDs (off)
      delay(1000);
    }
  }
  else {
    // Default: Set to solid white (all LEDs on)
    setRGBColor(1, 1, 1);  // All LEDs on
  }
}
