# 1 "C:\\Microsoft VS Code\\growith\\arduino\\arduino.ino"
# 2 "C:\\Microsoft VS Code\\growith\\arduino\\arduino.ino" 2


# 5 "C:\\Microsoft VS Code\\growith\\arduino\\arduino.ino" 2
# 6 "C:\\Microsoft VS Code\\growith\\arduino\\arduino.ino" 2
# 7 "C:\\Microsoft VS Code\\growith\\arduino\\arduino.ino" 2

#define ONE_WIRE_BUS 2
#define ESC_PIN1 10
#define ESC_PIN2 11
#define Vref 4.95

const char* ssid = "YourHotspotSSID"; // 노트북 핫스팟의 SSID
const char* password = "YourHotspotPassword"; // 노트북 핫스팟의 비밀번호

OneWire oneWire(2);
DallasTemperature sensors(&oneWire);
Servo esc1;
Servo esc2;
unsigned long int avgValue;

SoftwareSerial BluetoothSerial(2, 3); // RX, TX 핀 설정

// 함수 선언
float readPH();
void sendDataToServer(float temp, float do_value, float ph);

void setup() {
  _UART1_.begin(9600);
  BluetoothSerial.begin(9600);

  // WiFi 연결
  _UART1_.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    _UART1_.print(".");
    delay(1000);
  }
  _UART1_.println("\nConnected to WiFi");
  _UART1_.print("IP address: ");
  _UART1_.println(WiFi.localIP());

  sensors.begin();
  esc1.attach(10, 1000, 2000);
  esc2.attach(11, 1000, 2000);
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
  _UART1_.print("Temperature: ");
  _UART1_.println(temperatureC);
  _UART1_.print("DO: ");
  _UART1_.println(DOValue);
  _UART1_.print("pH: ");
  _UART1_.println(pHValue);

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

  delay(5000); // 5초 간격으로 데이터 전송
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
  float phVoltage = sensorValue * (4.95 / 1023.0);
  float scale = -3.5;
  float pHValue = 7.0 + (phVoltage - 2.5) * scale;
  return pHValue;
}
# 1 "C:\\Microsoft VS Code\\growith\\arduino\\test.ino"
# 2 "C:\\Microsoft VS Code\\growith\\arduino\\test.ino" 2

#define ESC_PIN1 10
#define ESC_PIN2 11

Servo esc1;
Servo esc2;

void setup() {
  _UART1_.begin(9600);
  while (!_UART1_) {
    ; // 시리얼 포트가 연결될 때까지 대기
  }

  _UART1_.println("Motor Test Program");
  _UART1_.println("Enter 'f' for forward");
  _UART1_.println("Enter 'b' for backward");
  _UART1_.println("Enter 'l' for left");
  _UART1_.println("Enter 'r' for right");
  _UART1_.println("Enter 's' for stop");

  esc1.attach(10, 1000, 2000);
  esc2.attach(11, 1000, 2000);
  esc1.writeMicroseconds(1500); // 초기화 - 정지 상태
  esc2.writeMicroseconds(1500); // 초기화 - 정지 상태
}

void loop() {
  if (_UART1_.available() > 0) {
    char input = _UART1_.read();
    switch (input) {
      case 'f': // Forward
        esc1.writeMicroseconds(1700);
        esc2.writeMicroseconds(1700);
        _UART1_.println("Moving Forward");
        break;
      case 'b': // Backward
        esc1.writeMicroseconds(1300);
        esc2.writeMicroseconds(1300);
        _UART1_.println("Moving Backward");
        break;
      case 'l': // Left
        esc1.writeMicroseconds(1300);
        esc2.writeMicroseconds(1700);
        _UART1_.println("Turning Left");
        break;
      case 'r': // Right
        esc1.writeMicroseconds(1700);
        esc2.writeMicroseconds(1300);
        _UART1_.println("Turning Right");
        break;
      case 's': // Stop
        esc1.writeMicroseconds(1500);
        esc2.writeMicroseconds(1500);
        _UART1_.println("Stopping");
        break;
      default:
        _UART1_.println("Invalid Command");
        break;
    }
  }
}
