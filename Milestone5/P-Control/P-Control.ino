#include <Wire.h>

#define MPU_ADDR 0x68
float angle = 0.0;
float lastTime = 0;
float alpha = 0.98;

const float Kp = 0.5;

#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

#define JOY_SW A0

#define SDA A4
#define SCL A5

const int stepCount = 4;
int stepIndex = 0;
const int stepTable[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

void setup() {
  
  Wire.begin(SDA, SCL);
  
  Serial.begin(9600);

  // Set up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(JOY_SW, INPUT_PULLUP);
  lastTime = millis();
  
  Serial.println("P-Control System Ready — Press Joystick to Enable");
}

void loop() {
  bool controlEnabled = digitalRead(JOY_SW) == LOW;

  updateAngle(); // Update angle from MPU6050

  Serial.print("Angle: "); Serial.print(angle, 2);

  if (controlEnabled) {
    
    float error = 0.0 - angle;
    float P = Kp * abs(error);   
    float absError = abs(error);

    int delayTime = map(P, 0, 45, 50, 2);
    delayTime = constrain(delayTime, 2, 50);

    if (absError > 1.0) {
      if (error > 0) {
        stepClockwise();
      } else {
        stepCounterClockwise();
      }
      delay(delayTime);
    }
  }

  delay(10);
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

void updateAngle() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  float ax = ax_raw / 16384.0;
  float ay = ay_raw / 16384.0;
  float az = az_raw / 16384.0;
  float gyroY = gy_raw / 131.0;

  float accAngle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  float now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  angle = alpha * (angle + gyroY * dt) + (1 - alpha) * accAngle;
}
