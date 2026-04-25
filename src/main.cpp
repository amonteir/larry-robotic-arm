#include <Arduino.h>
#include <TMCStepper.h>

#define EN_PIN 25
#define DIR_PIN 26
#define STEP_PIN 27

#define RXD2 16
#define TXD2 17

#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f

constexpr int MOTOR_FULL_STEPS_PER_REV = 200;  // 1.8 degree motor
constexpr int MICROSTEPS = 16;
constexpr float GEAR_RATIO = 20.0f;  // 20 motor turns for 1 output turn
constexpr uint16_t MOTOR_CURRENT_MA = 1200;
constexpr int STEP_HIGH_US = 20;
constexpr int STEP_LOW_US = 500;
constexpr float MIN_INPUT_ANGLE_DEG = -360.0f;
constexpr float MAX_INPUT_ANGLE_DEG = 360.0f;
constexpr size_t INPUT_BUFFER_SIZE = 32;

HardwareSerial TMCSerial(2);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);
char inputBuffer[INPUT_BUFFER_SIZE];
size_t inputLen = 0;

long angleToSteps(float angleDeg) {
  const float stepsPerDegree =
      (MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0f;
  return lroundf(fabsf(angleDeg) * stepsPerDegree);
}

void pulseStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_HIGH_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_LOW_US);
}

void moveSteps(long steps, bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  delay(5);

  Serial.print("Direction: ");
  Serial.print(direction ? "HIGH" : "LOW");
  Serial.print(" | Steps: ");
  Serial.println(steps);

  for (long i = 0; i < steps; i++) {
    pulseStep();
  }
}

void moveAngle(float angleDeg) {
  const bool direction = angleDeg >= 0.0f;
  const long steps = angleToSteps(angleDeg);

  Serial.print("Commanded output angle: ");
  Serial.print(angleDeg);
  Serial.print(" deg | Gear ratio: ");
  Serial.print(GEAR_RATIO);
  Serial.print(":1 | Calculated motor steps: ");
  Serial.println(steps);

  moveSteps(steps, direction);
}

void printPrompt() {
  Serial.println();
  Serial.println("Enter angle from -360 to 360, then press Enter:");
}

void handleAngleCommand(float angleDeg) {
  if (angleDeg < MIN_INPUT_ANGLE_DEG || angleDeg > MAX_INPUT_ANGLE_DEG) {
    Serial.println("Angle out of range. Use a value from -360 to 360.");
    printPrompt();
    return;
  }

  if (fabsf(angleDeg) < 0.001f) {
    Serial.println("Angle is 0. Nothing to move.");
    printPrompt();
    return;
  }

  moveAngle(angleDeg);
  printPrompt();
}

void processInputLine() {
  inputBuffer[inputLen] = '\0';

  char *endPtr = nullptr;
  const float angleDeg = strtof(inputBuffer, &endPtr);

  while (endPtr != nullptr && *endPtr == ' ') {
    endPtr++;
  }

  if (inputLen == 0 || endPtr == inputBuffer ||
      (endPtr != nullptr && *endPtr != '\0')) {
    Serial.println();
    Serial.println("Invalid input. Example: 90, -45, 180");
    inputLen = 0;
    printPrompt();
    return;
  }

  inputLen = 0;
  Serial.println();
  handleAngleCommand(angleDeg);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\b' || c == 127) {
      if (inputLen > 0) {
        inputLen--;
        Serial.print("\b \b");
      }
      continue;
    }

    if (c == '\n') {
      processInputLine();
      continue;
    }

    if (inputLen < INPUT_BUFFER_SIZE - 1) {
      inputBuffer[inputLen++] = c;
      Serial.print(c);
    } else {
      Serial.println();
      Serial.println("Input too long. Try a shorter number.");
      inputLen = 0;
      printPrompt();
    }
  }
}

void setupDriver() {
  TMCSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  driver.begin();

  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.mstep_reg_select(true);
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(MOTOR_CURRENT_MA);
  driver.microsteps(MICROSTEPS);
  driver.en_spreadCycle(true);
  driver.ihold(16);
  driver.irun(24);
  driver.iholddelay(8);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(EN_PIN, LOW);

  setupDriver();
  delay(100);

  Serial.println();
  Serial.println("TMC2209 STEP/DIR interactive angle control");
  Serial.print("STEP pin: ");
  Serial.println(STEP_PIN);
  Serial.print("DIR pin: ");
  Serial.println(DIR_PIN);
  Serial.print("EN pin: ");
  Serial.println(EN_PIN);
  Serial.print("Microsteps: ");
  Serial.println(MICROSTEPS);
  Serial.print("Gear ratio: ");
  Serial.println(GEAR_RATIO);
  Serial.print("Current mA RMS: ");
  Serial.println(MOTOR_CURRENT_MA);
  printPrompt();
}

void loop() {
  readSerialCommands();
}
