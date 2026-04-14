#include <Servo.h>

Servo servo1;

const int ldrPin = A0;
const int threshold = 100;

bool hasMoved = false;   // Prevent repeated movement

void setup() {
  Serial.begin(9600);

  servo1.attach(D0);   // GPIO14
  servo1.write(90);    // Neutral position
}

void loop() {
  int ldrValue = analogRead(ldrPin);
  Serial.println(ldrValue);

  if (ldrValue < threshold) {   // Dim condition
    if (!hasMoved) {
      moveServoOnce();   // Move only once
      hasMoved = true;   // Block further movement
    }
  } else {
    // Bright condition → reset
    hasMoved = false;
    servo1.write(90);   // Return to neutral (optional)
  }

  delay(10);
}

// 🔁 Move only once
void moveServoOnce() {
  for (int pos = 0; pos <= 90; pos += 5) {
    servo1.write(pos);
    delay(15);
  }

  for (int pos = 90; pos >= 0; pos -= 5) {
    servo1.write(pos);
    delay(15);
  }
}
