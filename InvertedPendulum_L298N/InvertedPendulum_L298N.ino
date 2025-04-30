#include <Wire.h>
#include <PID_v1.h>

// === MPU6050 Setup ===
#define MPU_ADDR 0x68
float angle = 0.0;
float lastTimeIMU = 0;
float alpha = 0.98;

// === Stepper Motor Pins (L298N OUT1~4) ===
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

const int stepCount = 4;
int stepIndex = 0;
const int stepTable[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

// === PID Variables ===
double setpoint = 0.0;  // Target upright position
double input = 0.0;     // Measured tilt angle
double output = 0.0;    // PID controller output

// PID tuning parameters
double Kp = 3.0;
double Ki = 0.5;
double Kd = 1.0;

// Create PID controller
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Wire.begin(); // Default I2C pins (A4=SDA, A5=SCL)

  Serial.begin(9600);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Motor control range

  lastTimeIMU = millis();

  Serial.println("PID_v1 Control System Ready (fixed version)");
}

void loop() {
  updateAngle();

  input = angle;    // Update PID input
  myPID.Compute();  // Run PID calculation

  Serial.print("Angle: "); Serial.print(angle, 2);
  Serial.print(" | PID Output: "); Serial.println(output, 2);

  int stepsToMove = map(abs(output), 0, 255, 0, 100); // Max 10 steps
  stepsToMove = constrain(stepsToMove, 0, 100);

  if (abs(input) > 1.0) { // Deadzone
    for (int i = 0; i < stepsToMove; i++) {
      if (output > 0) {
        stepClockwise();
      } else {
        stepCounterClockwise();
      }
      delay(1); // Tiny step delay
    }
  }

  delay(5); // Small stability delay
}

void stepClockwise() {
  stepIndex = (stepIndex + 1) % stepCount;
  applyStep();
}

void stepCounterClockwise() {
  stepIndex = (stepIndex - 1 + stepCount) % stepCount;
  applyStep();
}

void applyStep() {
  digitalWrite(IN1, stepTable[stepIndex][0]);
  digitalWrite(IN2, stepTable[stepIndex][1]);
  digitalWrite(IN3, stepTable[stepIndex][2]);
  digitalWrite(IN4, stepTable[stepIndex][3]);
}

// === Complementary Filter for Angle Estimation ===
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
  float gyroY = gy_raw / 131.0; // <<< FIXED! Use gyY, not gx

  float accAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  
  accAngle += 90.0;
  if (ay < 0) {
    accAngle = -accAngle;
  }

  float nowTime = millis();
  float dt = (nowTime - lastTimeIMU) / 1000.0;
  lastTimeIMU = nowTime;

  angle = alpha * (angle + gyroY * dt) + (1 - alpha) * accAngle;
}
