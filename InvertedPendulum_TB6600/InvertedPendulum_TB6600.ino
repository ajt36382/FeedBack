#include <Wire.h>
#include <PID_v1.h>

// === MPU6050 Setup ===
#define MPU_ADDR 0x68

// === Stepper Motor Pins (TB6600) ===
#define STEP_PIN 3
#define DIR_PIN 4

float angle = 0.0;
unsigned long lastUpdate = 0;
float alpha = 0.98;

// PID
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, 5, 1, 1, DIRECT);

// Stepper Motor Control
unsigned long lastStepMicros = 0;
int pulseDelayMicros = 1000; // initial default
bool stepHigh = false;

void setup() {
  Wire.begin();
  Wire.setClock(80000);
  Serial.begin(115200);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Setpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  // update angle every ~5ms
  if (micros() - lastUpdate > 2000) {
    lastUpdate = micros();
    updateAngle();
    Input = angle;
    myPID.Compute();

    Serial.print("Angle: ");
    Serial.print(angle, 2);
    Serial.print(" | PID Output: ");
    Serial.println(Output);

    // Set direction
    if (Output > 5) {
      digitalWrite(DIR_PIN, HIGH);
      pulseDelayMicros = map(Output, 5, 255, 3000, 500); // slower to faster
    } else if (Output < -5) {
      digitalWrite(DIR_PIN, LOW);
      pulseDelayMicros = map(-Output, 5, 255, 3000, 500);
    } else {
      pulseDelayMicros = 0; // stop stepping if very small output
    }
  }

  // Step the motor
  if (pulseDelayMicros > 0) {
    manageMotorStepping();
  }
}

void manageMotorStepping() {
  unsigned long now = micros();
  if (now - lastStepMicros >= pulseDelayMicros) {
    lastStepMicros = now;

    // Decide how many steps to move based on Output strength
    int steps = map(abs(Output), 5, 255, 1, 5);  // move 1 to 5 steps each time
    steps = constrain(steps, 1, 5);

    for (int i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(2);
    }
  }
}

void updateAngle() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  float ax = ax_raw / 16384.0;
  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;
  float gyroX = gx_raw / 131.0;

  float accAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  
  accAngle = accAngle + 90.0;
  if (ay < 0) {
    accAngle = -1 * accAngle;
  }

  static float lastTime = millis();
  float nowTime = millis();
  float dt = (nowTime - lastTime) / 1000.0;
  lastTime = nowTime;

  angle = alpha * (angle + gyroX * dt) + (1 - alpha) * accAngle;
}