#line 1 "C:\\Microsoft VS Code\\growith\\arduino\\arduino.ino"
#include <DallasTemperature.h>
#include <Arduino.h>
#include <OneWire.h>
#include <Servo.h>
#include <WiFiS3.h>
#include <SoftwareSerial.h>

#define ONE_WIRE_BUS 2
#define ESC_PIN1 10
#define ESC_PIN2 11
#define Vref 4.95

const char* ssid = "YourHotspotSSID";  // 노트북 핫스팟의 SSID
const char* password = "YourHotspotPassword";  // 노트북 핫스팟의 비밀번호

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Servo esc1;
Servo esc2;
unsigned long int avgValue;

SoftwareSerial BluetoothSerial(2, 3); // RX, TX 핀 설정

// 함수 선언
float readPH();
void sendDataToServer(float temp, float do_value, float ph);

void setup() {
  Serial.begin(9600);
  BluetoothSerial.begin(9600);

  // WiFi 연결
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  sensors.begin();
  esc1.attach(ESC_PIN1, 1000, 2000);
  esc2.attach(ESC_PIN2, 1000, 2000);
  esc1.writeMicroseconds(1500);
  esc2.writeMicroseconds(1500);
}

void loop() {
  // 센서 데이터 읽기
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  int doSensorValue = analogRead(A0);
  float doVoltage = doSensorValue * (5.0 / 1023.0);
  float DOValue = (doVoltage / 3.0) * 20.0;
  float pHValue = readPH();

  // 시리얼 포트로 데이터 전송
  Serial.print("Temperature: ");
  Serial.println(temperatureC);
  Serial.print("DO: ");
  Serial.println(DOValue);
  Serial.print("pH: ");
  Serial.println(pHValue);

  // 블루투스 입력으로 모터 제어
  if (BluetoothSerial.available()) {
    char input = BluetoothSerial.read();
    switch (input) {
      case 'f': // Forward
        esc1.writeMicroseconds(1700);
        esc2.writeMicroseconds(1700);
        break;
      case 'b': // Backward
        esc1.writeMicroseconds(1300);
        esc2.writeMicroseconds(1300);
        break;
      case 'l': // Left
        esc1.writeMicroseconds(1300);
        esc2.writeMicroseconds(1700);
        break;
      case 'r': // Right
        esc1.writeMicroseconds(1700);
        esc2.writeMicroseconds(1300);
        break;
      case 's': // Stop
        esc1.writeMicroseconds(1500);
        esc2.writeMicroseconds(1500);
        break;
    }
  }

  delay(5000);  // 5초 간격으로 데이터 전송
}

float readPH() {
  int buf[10];
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(A1);
    delay(10);
  }
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        int temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;
  for (int i = 2; i < 8; i++) {
    avgValue += buf[i];
  }
  float sensorValue = avgValue / 6.0;
  float phVoltage = sensorValue * (Vref / 1023.0);
  float scale = -3.5;
  float pHValue = 7.0 + (phVoltage - 2.5) * scale;
  return pHValue;
}
#line 1 "C:\\Microsoft VS Code\\growith\\arduino\\test.ino"
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

