#include <Wire.h>

const uint8_t MPU_ADDRESS = 0x68;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // 실제 배선
  // SDA = GPIO 27
  // SCL = GPIO 26
  Wire.begin(27, 26);

  // 느리고 안정적인 I2C 속도
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  // MPU 절전모드 해제
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);

  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("WAKE FAILED");

    while (true) {
      delay(1000);
    }
  }

  delay(100);

  Serial.println("MPU START");
}

void loop() {
  // 가속도 데이터가 시작되는 레지스터 선택
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    Serial.println("REGISTER ERROR");
    delay(1000);
    return;
  }

  // X, Y, Z 가속도 총 6바이트 요청
  int received = Wire.requestFrom(
    MPU_ADDRESS,
    (uint8_t)6,
    true
  );

  if (received != 6) {
    Serial.print("READ FAILED: ");
    Serial.println(received);

    delay(1000);
    return;
  }

  // 상위 바이트와 하위 바이트 결합
  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // 기본 ±2g 범위를 가정해 g 단위로 변환
  float x = rawX / 16384.0;
  float y = rawY / 16384.0;
  float z = rawZ / 16384.0;

  Serial.print("X:");
  Serial.print(x, 2);

  Serial.print(" Y:");
  Serial.print(y, 2);

  Serial.print(" Z:");
  Serial.println(z, 2);

  // 1초에 한 번만 측정
  delay(1000);
}