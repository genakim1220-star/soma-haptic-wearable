#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

const uint8_t MPU_ADDRESS = 0x68;

// ESP-NOW 브로드캐스트 주소
uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

struct MotionPacket {
  uint32_t sequence;
  float movement;
  uint8_t intensity;
};

MotionPacket packet;

float previousX = 0;
float previousY = 0;
float previousZ = 0;

bool firstReading = true;

bool readAcceleration(
  float &x,
  float &y,
  float &z
) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return false;
  }

  int received = Wire.requestFrom(
    MPU_ADDRESS,
    (uint8_t)6,
    true
  );

  if (received != 6) {
    return false;
  }

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  x = rawX / 16384.0;
  y = rawY / 16384.0;
  z = rawZ / 16384.0;

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // MPU 통신 시작
  Wire.begin(27, 26);
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  // MPU 절전모드 해제
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    Serial.println("MPU WAKE FAILED");

    while (true) {
      delay(1000);
    }
  }

  delay(100);

  // ESP-NOW 준비
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");

    while (true) {
      delay(1000);
    }
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );

  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");

    while (true) {
      delay(1000);
    }
  }

  packet.sequence = 0;

  Serial.println("SENDER READY");
}

void loop() {
  float x;
  float y;
  float z;

  if (!readAcceleration(x, y, z)) {
    Serial.println("MPU READ FAILED");
    delay(100);
    return;
  }

  if (firstReading) {
    previousX = x;
    previousY = y;
    previousZ = z;

    firstReading = false;

    delay(50);
    return;
  }

  // 직전 측정값과 현재 측정값의 차이
  float differenceX = abs(x - previousX);
  float differenceY = abs(y - previousY);
  float differenceZ = abs(z - previousZ);

  // 세 축의 변화량을 더해 움직임 세기로 사용
  float movement =
    differenceX +
    differenceY +
    differenceZ;

  previousX = x;
  previousY = y;
  previousZ = z;

  // 작은 센서 흔들림은 무시
  const float MIN_MOVEMENT = 0.03;

  // 이 정도 이상의 변화는 최대 진동
  const float MAX_MOVEMENT = 1.00;

  int intensity = 0;

  if (movement > MIN_MOVEMENT) {
    float normalized =
      (movement - MIN_MOVEMENT) /
      (MAX_MOVEMENT - MIN_MOVEMENT);

    normalized = constrain(normalized, 0.0, 1.0);

    // DRV2605 실시간 진동 범위: 0~127
    intensity = normalized * 127;
  }

  packet.sequence++;
  packet.movement = movement;
  packet.intensity = intensity;

  esp_err_t result = esp_now_send(
    broadcastAddress,
    (uint8_t *)&packet,
    sizeof(packet)
  );

  // 시리얼 출력량을 줄이기 위해 10회마다 출력
  if (packet.sequence % 10 == 0) {
    Serial.print("Movement: ");
    Serial.print(movement, 3);

    Serial.print(" | Intensity: ");
    Serial.print(intensity);

    Serial.print(" | Send: ");
    Serial.println(
      result == ESP_OK ? "OK" : "FAILED"
    );
  }

  // 초당 약 20회 측정·전송
  delay(50);
}