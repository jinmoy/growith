#include <Arduino.h>
#include <Servo.h>


#define ESC_PIN1 10
#define ESC_PIN2 11

Servo esc1;
Servo esc2;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // 시리얼 포트가 연결될 때까지 대기
  }

  Serial.println("Motor Test Program");
  Serial.println("Enter 'f' for forward");
  Serial.println("Enter 'b' for backward");
  Serial.println("Enter 'l' for left");
  Serial.println("Enter 'r' for right");
  Serial.println("Enter 's' for stop");

  esc1.attach(ESC_PIN1, 1000, 2000);
  esc2.attach(ESC_PIN2, 1000, 2000);
  esc1.writeMicroseconds(1500);  // 초기화 - 정지 상태
  esc2.writeMicroseconds(1500);  // 초기화 - 정지 상태
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    switch (input) {
      case 'f': // Forward
        esc1.writeMicroseconds(1700);
        esc2.writeMicroseconds(1700);
        Serial.println("Moving Forward");
        break;
      case 'b': // Backward
        esc1.writeMicroseconds(1300);
        esc2.writeMicroseconds(1300);
        Serial.println("Moving Backward");
        break;
      case 'l': // Left
        esc1.writeMicroseconds(1300);
        esc2.writeMicroseconds(1700);
        Serial.println("Turning Left");
        break;
      case 'r': // Right
        esc1.writeMicroseconds(1700);
        esc2.writeMicroseconds(1300);
        Serial.println("Turning Right");
        break;
      case 's': // Stop
        esc1.writeMicroseconds(1500);
        esc2.writeMicroseconds(1500);
        Serial.println("Stopping");
        break;
      default:
        Serial.println("Invalid Command");
        break;
    }
  }
}
