#include <Arduino.h>

// ---------------- LEFT MOTOR ----------------
#define L_PWM 5
#define L_DIR 4
#define L_ENCA 2
#define L_ENCB 3

// ---------------- RIGHT MOTOR ----------------
#define R_PWM 6
#define R_DIR 7
#define R_ENCA 18
#define R_ENCB 19

// ---------------- ENCODER VARIABLES ----------------
volatile long leftTicks = 0;
volatile long rightTicks = 0;

const int ticks_per_rev = 683;
const float wheel_diameter = 0.115;
const float wheel_circumference = 3.1416 * wheel_diameter;

// ---------------- TIMING VARIABLE ----------------
unsigned long last_enc_time = 0;

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // CRITICAL: Prevents 1-second freezing on bad serial packets

  pinMode(L_PWM, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  pinMode(L_ENCA, INPUT);
  pinMode(L_ENCB, INPUT);
  pinMode(R_ENCA, INPUT);
  pinMode(R_ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(L_ENCA), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), rightEncoderISR, RISING);

  digitalWrite(L_DIR, HIGH);
  digitalWrite(R_DIR, HIGH);
}

// ---------------- LOOP ----------------
void loop() {

  // 1. NON-BLOCKING READ: Process ALL available serial data instantly
  while (Serial.available()) {
    String data = Serial.readStringUntil('\n');

    if (data.startsWith("CMD:")) {
      int commaIndex = data.indexOf(',');
      
      // CRITICAL: Only parse if the string isn't corrupted (contains a comma)
      if (commaIndex != -1) {
        float leftPWM = data.substring(4, commaIndex).toFloat();
        float rightPWM = data.substring(commaIndex + 1).toFloat();

        digitalWrite(L_DIR, leftPWM >= 0 ? HIGH : LOW);
        digitalWrite(R_DIR, rightPWM >= 0 ? HIGH : LOW);

        analogWrite(L_PWM, abs(leftPWM));
        analogWrite(R_PWM, abs(rightPWM));
      }
    }
  }

  // 2. NON-BLOCKING DELAY: Send encoder data every 20ms WITHOUT stopping the loop
  if (millis() - last_enc_time >= 20) {
    last_enc_time = millis(); // Reset timer

    Serial.print("ENC:");
    Serial.print(leftTicks);
    Serial.print(",");
    Serial.println(rightTicks);
  }
}

// ---------------- ENCODER ISRs ----------------
void leftEncoderISR() {
  if (digitalRead(L_ENCB))
    leftTicks++;
  else
    leftTicks--;
}

void rightEncoderISR() {
  if (digitalRead(R_ENCB))
    rightTicks++;
  else
    rightTicks--;
}