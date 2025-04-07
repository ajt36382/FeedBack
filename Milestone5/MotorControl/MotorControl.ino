// === Stepper Motor Control Pins ===
#define IN1 8   // A+
#define IN2 9   // A-
#define IN3 10  // B+
#define IN4 11  // B-

// === Joystick X axis ===
#define JOY_X A0

const int deadZone = 30;
const int joyCenter = 512;
const int stepCount = 4;
int stepIndex = 0;

// Full-step drive sequence
const int stepTable[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(JOY_X, INPUT);

  Serial.begin(9600);
}

void loop() {
  int joyVal = analogRead(JOY_X);         // 0–1023
  int offset = joyVal - joyCenter;        // -512 to +511
  Serial.print("X offset: "); Serial.print(offset);
  Serial.println();
  if (abs(offset) < deadZone) {
    return;
  }

  int delayTime = map(abs(offset), deadZone, 511, 50, 0.5);

  if (offset > 0) {
    stepClockwise();
  } else {
    stepCounterClockwise();
  }

  delay(delayTime);
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
