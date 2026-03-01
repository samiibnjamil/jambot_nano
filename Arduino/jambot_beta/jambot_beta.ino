#include <PinChangeInterrupt.h>
#include <PID_v1.h>
#include <stdlib.h>
#include <string.h>

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
// Non-beta tuned PID defaults (kept fixed in beta).
double Kp = 0.6, Ki = 1.7, Kd = 0.001, Ko = 1.0;
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
unsigned long lastMotionCommandTime = 0;
const unsigned long motionTimeout = 500;

struct RGBColor {
  int r;
  int g;
  int b;
};

// Add this near your other state variables =====================
enum LedState {
  LED_BOOTING,         // Solid blue while initializing
  LED_READY_LISTENING, // Solid green: firmware ready, waiting for host commands
  LED_DISCONNECTED,    // Blink yellow
  LED_CONNECTED,       // Accept external commands
  LED_LOW_BATTERY,     // Blink red
  LED_AUTONOMOUS_MODE, // Solid blue
  LED_CUSTOM_1,
  LED_CUSTOM_2
};

LedState currentLedState = LED_BOOTING;
unsigned long lastLedUpdate = 0;
bool ledBlinkState = false;
bool wasDisconnected = true;
RGBColor currentColor = {0, 0, 0};  // Track current color
bool hasReceivedCommand = false;

bool lowBatteryActive = false;
bool autonomousModeEnabled = false;
float lastBatteryVoltage = 0.0;
unsigned long lastBatteryCheck = 0;
const unsigned long batteryCheckInterval = 1000;
const float lowBatteryEnterVoltage = 9.9;
const float lowBatteryExitVoltage = 10.3;

bool timeoutActive = false;

bool rampDownActive = false;
int rampPWM1 = 0;
int rampPWM2 = 0;
unsigned long lastRampStepTime = 0;
const unsigned long rampStepIntervalMs = 5;

int activeBuzzerType = 0;
int buzzerStepIndex = 0;
unsigned long lastBuzzerStepTime = 0;
unsigned long buzzerStepIntervalMs = 50;
bool buzzerOutputHigh = false;

int activeLightPattern = 0;
int lightStepIndex = 0;
int lightRepeatCount = 0;
unsigned long lastLightStepTime = 0;
// ==============================================================

bool readCommandLine(char* outBuffer, size_t outSize);
int tokenizeCommand(char* line, char* tokens[], int maxTokens);
bool parseLongValue(const char* text, long* value);
bool parseDoubleValue(const char* text, double* value);
void startRampDown();
void updateRampDown();
void updateBuzzerPattern();
void updateLightPattern();
void applyLightStep(int patternId, int step);








void setup() {
  setupEncoders();
  setupMotorDriver();
  setupBuzzerLED();
  currentLedState = LED_BOOTING;
  setRGBColor(0, 0, 1);  // Show boot-in-progress immediately.
  initMPU6050();
  Serial.begin(115200);
  lastDataReceivedTime = millis();
  lastMotionCommandTime = millis();
  initializePIDControllers();
  currentLedState = LED_READY_LISTENING;
  updateLedState();
  Serial.println("ready");
}

void loop() {
  updateRampDown();
  updateBuzzerPattern();
  updateLightPattern();
  if (pidEnabled) {
    speedPID(0);  // Compute PID outputs
  }
  processMotorCommands();
  //testSin();
  //motorStay();

}
void setupBuzzerLED() {
  // Configure pins as OUTPUT
  pinMode(CLED, OUTPUT);
  pinMode(MLED, OUTPUT);
  pinMode(YLED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(CLED, HIGH);
  digitalWrite(MLED, HIGH);
  digitalWrite(YLED, HIGH);
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
  motor1PID.SetSampleTime(50);
  motor2PID.SetSampleTime(50);
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
    long encoder1Snapshot = 0;
    long encoder2Snapshot = 0;
    noInterrupts();
    encoder1Snapshot = encoder1Count;
    encoder2Snapshot = encoder2Count;
    interrupts();

    long motor1Pulses = encoder1Snapshot - lastMotor1Count;
    long motor2Pulses = encoder2Snapshot - lastMotor2Count;

    motor1Speed = (motor1Pulses * 1200.0) / CPR;
    motor2Speed = (motor2Pulses * 1200.0) / CPR;

    // Apply Low-Pass Filter to RPM 25hz cukoff
    v1Filt = 0.854 * v1Filt + 0.0728 * motor1Speed + 0.0728 * v1Prev;
    v1Prev = motor1Speed;
    v2Filt = 0.854 * v2Filt + 0.0728 * motor2Speed + 0.0728 * v2Prev;
    v2Prev = motor2Speed;


    motor1PID.Compute();
    motor2PID.Compute();

    lastMotor1Count = encoder1Snapshot;
    lastMotor2Count = encoder2Snapshot;
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


void processMotorCommands() {

  // Check connection state first
  checkConnectionState();

  // Keep battery state updated continuously
  updateBatteryState(false);

  // Update LED state machine
  updateLedState();


  // Motion watchdog: stop if motion commands stop arriving.
  if (millis() - lastMotionCommandTime > motionTimeout) {
    if (!timeoutActive) {
      timeoutActive = true;
      // Stop the motors once when timeout starts.
      rampDownMotors();
      pidEnabled = false;
      motor1PID.SetMode(MANUAL);  // Disable speed PID
      motor2PID.SetMode(MANUAL);  // Disable speed PID
    }
  
  }

  char commandLine[96];
  if (readCommandLine(commandLine, sizeof(commandLine))) {
    if (commandLine[0] == '\0') {
      Serial.println("ERR line too long");
      return;
    }
    char* tokens[8];
    int tokenCount = tokenizeCommand(commandLine, tokens, 8);
    if (tokenCount <= 0) {
      Serial.println("ERR empty cmd");
      return;
    }
    char cmd = tokens[0][0];
    // Update the last data received time
    lastDataReceivedTime = millis();
    hasReceivedCommand = true;

    switch (cmd) {
      case 'b':
        {
          measureBatteryVoltage();
          break;
        }
      case 'e':
        {
          // Respond with current encoder counts
          long encoder1Snapshot = 0;
          long encoder2Snapshot = 0;
          noInterrupts();
          encoder1Snapshot = encoder1Count;
          encoder2Snapshot = encoder2Count;
          interrupts();

          Serial.print("e");
          Serial.print(" ");
          Serial.print(encoder1Snapshot);
          Serial.print(" ");
          Serial.println(encoder2Snapshot);
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
          long red = 0;
          long green = 0;
          long blue = 0;
          if (tokenCount == 4 && parseLongValue(tokens[1], &red) && parseLongValue(tokens[2], &green) && parseLongValue(tokens[3], &blue)) {
            if ((red < 0 || red > 1) || (green < 0 || green > 1) || (blue < 0 || blue > 1)) {
              Serial.println("ERR range");
              break;
            }
            setRGBColor((int)red, (int)green, (int)blue);
            currentColor = {(int)red, (int)green, (int)blue};  // Store color
            Serial.println("OK");
          } else {
            Serial.println("ERR bad args");
          }
          break;
        }
      case 'o':
        {
          // Set the raw PWM speed
          long outputPWM11 = 0;
          long outputPWM21 = 0;
          if (tokenCount == 3 && parseLongValue(tokens[1], &outputPWM11) && parseLongValue(tokens[2], &outputPWM21)) {
            if ((outputPWM11 < -255 || outputPWM11 > 255) || (outputPWM21 < -255 || outputPWM21 > 255)) {
              Serial.println("ERR range");
              break;
            }
            lastMotionCommandTime = millis();
            timeoutActive = false;
            pidEnabled = false;
            motor1PID.SetMode(MANUAL);  // Disable speed PID
            motor2PID.SetMode(MANUAL);  // Disable speed PID
            // motor1PositionPID.SetMode(MANUAL);  // Disable position PID
            // motor2PositionPID.SetMode(MANUAL);  // Disable position PID
            
            if (outputPWM11 == 0 && outputPWM21 == 0) {
              rampDownMotors();
            } else {
              applyMotorSpeed((int)outputPWM11, (int)outputPWM21);
              rampDownActive = false;
              outputPWM1=outputPWM11;
              outputPWM2=outputPWM21;
            }
            noInterrupts();
            encoder1Count= 0;
            encoder2Count=0;
            interrupts();
          
            Serial.println("OK");
          } else {
            Serial.println("ERR bad args");
          }
          break;
        }
      case 'm':
        {
          // Set the closed-loop speed
          long rpm1 = 0;
          long rpm2 = 0;
          if (tokenCount == 3 && parseLongValue(tokens[1], &rpm1) && parseLongValue(tokens[2], &rpm2)) {
            targetRPM1 = rpm1;
            targetRPM2 = rpm2;
            lastMotionCommandTime = millis();
            timeoutActive = false;

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
              rampDownActive = false;
              // motor1PositionPID.SetMode(MANUAL);
              // motor2PositionPID.SetMode(MANUAL);
            }
            Serial.println("OK");
          } else {
            Serial.println("ERR bad args");
          }
          break;
        }

      case 'n':
        {
          // buzzer
          long buzzerState = 0;
          if (tokenCount == 2 && parseLongValue(tokens[1], &buzzerState)) {
            if (buzzerState < 0 || buzzerState > 8) {
              Serial.println("ERR range");
              break;
            }
            if (buzzerState == 0) {
              activeBuzzerType = 0;
              digitalWrite(BUZZER, LOW);  // Turn off the buzzer
            } else {
              buzzerSound((int)buzzerState);
            }
            Serial.println("OK");
          } else {
            Serial.println("ERR bad args");
          }
          break;
        }
      case 'a':
        {
          // Toggle autonomous LED mode: a 1 (enable), a 0 (disable)
          long modeState = 0;
          if (tokenCount == 2 && parseLongValue(tokens[1], &modeState)) {
            if (modeState != 0 && modeState != 1) {
              Serial.println("ERR range");
              break;
            }
            autonomousModeEnabled = (modeState == 1);

            if (autonomousModeEnabled && !lowBatteryActive) {
              currentLedState = LED_AUTONOMOUS_MODE;
            } else if (lowBatteryActive) {
              currentLedState = LED_LOW_BATTERY;
            } else {
              bool isConnected = (millis() - lastDataReceivedTime) < dataTimeout;
              currentLedState = isConnected ? LED_CONNECTED : LED_DISCONNECTED;
              if (currentLedState == LED_CONNECTED) {
                wasDisconnected = true;
              }
            }
            Serial.println("OK");
          } else {
            Serial.println("ERR bad args");
          }
          break;
        }
      case 'p':
        {
          // Keep PID fixed to the non-beta tuned values.
          // Accept command for protocol compatibility but ignore values.
          Serial.println("OK");
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
          noInterrupts();
          encoder1Count = 0;
          encoder2Count = 0;
          interrupts();
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
            motor2PID.SetMode(MANUAL);
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

bool readCommandLine(char* outBuffer, size_t outSize) {
  static char lineBuffer[96];
  static size_t lineLength = 0;
  static bool overflowed = false;

  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      if (!overflowed && lineLength == 0) {
        continue;
      }
      if (overflowed) {
        outBuffer[0] = '\0';
        overflowed = false;
        lineLength = 0;
        return true;
      }
      lineBuffer[lineLength] = '\0';
      strncpy(outBuffer, lineBuffer, outSize - 1);
      outBuffer[outSize - 1] = '\0';
      lineLength = 0;
      return true;
    }

    if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = ch;
    } else {
      // Drop oversized command and wait for end-of-line.
      overflowed = true;
      lineLength = 0;
    }
  }
  return false;
}

int tokenizeCommand(char* line, char* tokens[], int maxTokens) {
  int count = 0;
  char* savePtr = NULL;
  char* token = strtok_r(line, " \t", &savePtr);
  while (token != NULL && count < maxTokens) {
    tokens[count++] = token;
    token = strtok_r(NULL, " \t", &savePtr);
  }
  return count;
}

bool parseLongValue(const char* text, long* value) {
  char* endPtr = NULL;
  long parsed = strtol(text, &endPtr, 10);
  if (endPtr == text || *endPtr != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

bool parseDoubleValue(const char* text, double* value) {
  char* endPtr = NULL;
  double parsed = strtod(text, &endPtr);
  if (endPtr == text || *endPtr != '\0') {
    return false;
  }
  *value = parsed;
  return true;
}

void startRampDown() {
  rampPWM1 = (int)outputPWM1;
  rampPWM2 = (int)outputPWM2;
  rampDownActive = true;
  lastRampStepTime = 0;
}

void updateRampDown() {
  if (!rampDownActive) {
    return;
  }
  if (millis() - lastRampStepTime < rampStepIntervalMs) {
    return;
  }
  lastRampStepTime = millis();

  if (rampPWM1 > 0) rampPWM1--;
  else if (rampPWM1 < 0) rampPWM1++;
  if (rampPWM2 > 0) rampPWM2--;
  else if (rampPWM2 < 0) rampPWM2++;

  applyMotorSpeed(rampPWM1, rampPWM2);
  outputPWM1 = rampPWM1;
  outputPWM2 = rampPWM2;

  if (rampPWM1 == 0 && rampPWM2 == 0) {
    rampDownActive = false;
  }
}

// New LED control functions ====================================
void updateLedState() {
  switch(currentLedState) {
    case LED_BOOTING:
      setRGBColor(0, 0, 1);
      break;

    case LED_READY_LISTENING:
      // Firmware initialized and waiting for host commands.
      setRGBColor(0, 1, 0);
      break;

    case LED_DISCONNECTED:
      // Fast yellow blink (250ms)
      if (millis() - lastLedUpdate >= 250) {
        lastLedUpdate = millis();
        ledBlinkState = !ledBlinkState;
        setRGBColor(ledBlinkState, ledBlinkState, 0);
      }
      break;

    case LED_CONNECTED:
      if (wasDisconnected) {
        // Restore last user-selected color when reconnecting.
        setRGBColor(currentColor.r, currentColor.g, currentColor.b);
        wasDisconnected = false;
      }
      // Otherwise, maintain last set color
      break;

    case LED_LOW_BATTERY:
      // Slow red blink (1000ms)
      if (millis() - lastLedUpdate >= 1000) {
        lastLedUpdate = millis();
        ledBlinkState = !ledBlinkState;
        setRGBColor(ledBlinkState, 0, 0);
      }
      break;

    case LED_AUTONOMOUS_MODE:
      // Solid blue override
      setRGBColor(0, 0, 1);
      break;
  }
}

void checkConnectionState() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    bool isConnected = (millis() - lastDataReceivedTime) < dataTimeout;

    // Do not override critical/explicit modes.
    if (currentLedState == LED_CONNECTED ||
        currentLedState == LED_DISCONNECTED ||
        currentLedState == LED_READY_LISTENING) {

      if (!hasReceivedCommand) {
        currentLedState = LED_READY_LISTENING;
      } else if (isConnected && currentLedState != LED_CONNECTED) {
        currentLedState = LED_CONNECTED;
        wasDisconnected = true;
      } else if (!isConnected && currentLedState != LED_DISCONNECTED) {
        currentLedState = LED_DISCONNECTED;
      }
    }
  }
}
// ==============================================================

float readBatteryVoltage() {
  const int analogPin = A7;  // Analog pin connected to the voltage divider
  const float R1 = 9500.0;   // Resistance of R1 in ohms
  const float R2 = 5200.0;   // Resistance of R2 in ohms
  const float vRef = 5.0;    // Reference voltage of the Arduino (5V)
  const int adcMax = 1023;   // Maximum value for the ADC (10-bit resolution)

  int analogValue = analogRead(analogPin);
  float vOut = (analogValue * vRef) / adcMax;
  return vOut * (R1 + R2) / R2;
}

void updateBatteryState(bool forceRead) {
  if (!forceRead && (millis() - lastBatteryCheck < batteryCheckInterval)) {
    return;
  }
  lastBatteryCheck = millis();
  lastBatteryVoltage = readBatteryVoltage();

  if (!lowBatteryActive && lastBatteryVoltage <= lowBatteryEnterVoltage) {
    lowBatteryActive = true;
    currentLedState = LED_LOW_BATTERY;
  } else if (lowBatteryActive && lastBatteryVoltage >= lowBatteryExitVoltage) {
    lowBatteryActive = false;
    if (currentLedState == LED_LOW_BATTERY) {
      if (autonomousModeEnabled) {
        currentLedState = LED_AUTONOMOUS_MODE;
      } else {
        if (!hasReceivedCommand) {
          currentLedState = LED_READY_LISTENING;
        } else {
          bool isConnected = (millis() - lastDataReceivedTime) < dataTimeout;
          currentLedState = isConnected ? LED_CONNECTED : LED_DISCONNECTED;
          if (currentLedState == LED_CONNECTED) {
            wasDisconnected = true;
          }
        }
      }
    }
  }
}

// Modified setRGBColor() ========================================
void setRGBColor(int red, int green, int blue) {
  // Allow updates from all LED states so state-machine patterns render correctly.
  if (currentLedState == LED_CONNECTED || 
      currentLedState == LED_BOOTING ||
      currentLedState == LED_READY_LISTENING ||
      currentLedState == LED_DISCONNECTED ||
      currentLedState == LED_LOW_BATTERY ||
      currentLedState == LED_AUTONOMOUS_MODE) {
        
    // Convert to CMY
    int cyan = 1 - red;
    int magenta = 1 - green;
    int yellow = 1 - blue;

    // Update hardware
    digitalWrite(CLED, cyan);
    digitalWrite(MLED, magenta);
    digitalWrite(YLED, yellow);

    // Store color unless in override state
    if (currentLedState == LED_CONNECTED) {
      currentColor = {red, green, blue};
    }
  }
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
  long encoder1Snapshot = 0;
  long encoder2Snapshot = 0;
  int encoder1DirSnapshot = 0;
  int encoder2DirSnapshot = 0;
  noInterrupts();
  encoder1Snapshot = encoder1Count;
  encoder2Snapshot = encoder2Count;
  encoder1DirSnapshot = encoder1Direction;
  encoder2DirSnapshot = encoder2Direction;
  interrupts();

  float distance1 = encoder1Snapshot * MM_PER_COUNT;
  float distance2 = encoder2Snapshot * MM_PER_COUNT;
  Serial.print("  E1: ");
  Serial.print(encoder1Snapshot);
  Serial.print(" Dir: ");
  Serial.print(encoder1DirSnapshot == -1 ? "FWD" : "REV");
  Serial.print(" Dist: ");
  Serial.print(distance1, 2);
  Serial.print("mm || ");
  Serial.print("E2: ");
  Serial.print(encoder2Snapshot);
  Serial.print(" Dir: ");
  Serial.print(encoder2DirSnapshot == -1 ? "FWD" : "REV");
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
  snprintf(buffer, sizeof(buffer), "i %d %d %d %d %d %d", ax, ay, az, gx, gy, gz);

  // Send the buffer over serial
  Serial.println(buffer);
}
void measureBatteryVoltage() {
  updateBatteryState(true);
  Serial.print("b ");
  Serial.println(lastBatteryVoltage);
}


void rampDownMotors() {
  startRampDown();
}


void buzzerSound(int soundType) {
  if (soundType < 1 || soundType > 8) {
    Serial.println("Invalid sound type");
    return;
  }
  activeBuzzerType = soundType;
  buzzerStepIndex = 0;
  lastBuzzerStepTime = 0;
  buzzerStepIntervalMs = 0;
  buzzerOutputHigh = false;
}

void updateBuzzerPattern() {
  if (activeBuzzerType == 0) {
    return;
  }
  if (lastBuzzerStepTime != 0 && millis() - lastBuzzerStepTime < buzzerStepIntervalMs) {
    return;
  }

  bool stepHigh = false;
  bool validStep = true;
  int stepLimit = 0;
  switch (activeBuzzerType) {
    case 1: stepLimit = 2; break;
    case 2: stepLimit = 4; break;
    case 3: stepLimit = 6; break;
    case 4: stepLimit = 2; break;
    case 5: stepLimit = 6; break;
    case 6: stepLimit = 4; break;
    case 7: stepLimit = 10; break;
    case 8: stepLimit = 2; break;
    default: validStep = false; break;
  }

  if (!validStep || buzzerStepIndex >= stepLimit) {
    activeBuzzerType = 0;
    digitalWrite(BUZZER, LOW);
    return;
  }

  stepHigh = (buzzerStepIndex % 2 == 0);
  digitalWrite(BUZZER, stepHigh ? HIGH : LOW);
  buzzerOutputHigh = stepHigh;
  buzzerStepIndex++;
  lastBuzzerStepTime = millis();
  switch (activeBuzzerType) {
    case 1: buzzerStepIntervalMs = buzzerOutputHigh ? 100 : 150; break;
    case 2: buzzerStepIntervalMs = 100; break;
    case 3: buzzerStepIntervalMs = buzzerOutputHigh ? 100 : 100; break;
    case 4: buzzerStepIntervalMs = buzzerOutputHigh ? 100 : 500; break;
    case 5:
      buzzerStepIntervalMs = (buzzerStepIndex == 3 && buzzerOutputHigh) ? 300 : 100;
      break;
    case 6: buzzerStepIntervalMs = buzzerOutputHigh ? 200 : 500; break;
    case 7: buzzerStepIntervalMs = 50; break;
    case 8: buzzerStepIntervalMs = buzzerOutputHigh ? 1000 : 500; break;
    default: buzzerStepIntervalMs = 100; break;
  }
}

// Function to implement different light patterns
void lightPattern(String pattern) {
  activeLightPattern = 0;
  lightStepIndex = 0;
  lightRepeatCount = 0;
  lastLightStepTime = 0;

  if (pattern == "breathing") {
    activeLightPattern = 1;
  }
  else if (pattern == "flashing") {
    activeLightPattern = 2;
  }
  else if (pattern == "fading") {
    activeLightPattern = 3;
  }
  else {
    // Default: Set to solid white (all LEDs on)
    setRGBColor(1, 1, 1);  // All LEDs on
  }
}

void updateLightPattern() {
  if (activeLightPattern == 0) {
    return;
  }
  unsigned long interval = (activeLightPattern == 2) ? 200 : 1000;
  if (lastLightStepTime != 0 && millis() - lastLightStepTime < interval) {
    return;
  }
  lastLightStepTime = millis();

  applyLightStep(activeLightPattern, lightStepIndex % 2);
  lightStepIndex++;

  int maxRepeats = (activeLightPattern == 2) ? 10 : 5;
  if (lightStepIndex % 2 == 0) {
    lightRepeatCount++;
  }
  if (lightRepeatCount >= maxRepeats) {
    activeLightPattern = 0;
    lightStepIndex = 0;
    lightRepeatCount = 0;
  }
}

void applyLightStep(int patternId, int step) {
  if (patternId == 1) {
    setRGBColor(step == 0 ? 1 : 0, 0, 0);
  } else if (patternId == 2) {
    setRGBColor(step == 0 ? 1 : 0, 0, 0);
  } else if (patternId == 3) {
    setRGBColor(0, 0, step == 0 ? 1 : 0);
  }
}
