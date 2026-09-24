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


// 자이로 이전 값
float previousGX = 0;
float previousGY = 0;
float previousGZ = 0;

bool firstReading = true;


// ================================
// 자이로 읽기
// ================================
bool readGyroscope(
  float &gx,
  float &gy,
  float &gz
) {

  Wire.beginTransmission(MPU_ADDRESS);

  // 자이로 데이터 시작 레지스터
  Wire.write(0x43);

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

  int16_t rawGX =
    (Wire.read() << 8) | Wire.read();

  int16_t rawGY =
    (Wire.read() << 8) | Wire.read();

  int16_t rawGZ =
    (Wire.read() << 8) | Wire.read();


  // 기본 설정 ±250 °/s
  // 감도 = 131 LSB / (°/s)

  gx = rawGX / 131.0;
  gy = rawGY / 131.0;
  gz = rawGZ / 131.0;

  return true;
}


void setup() {

  Serial.begin(115200);
  delay(2000);


  // ================================
  // MPU 통신 시작
  // ================================

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


  // ================================
  // ESP-NOW 준비
  // ================================

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_channel(
    1,
    WIFI_SECOND_CHAN_NONE
  );


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


  if (
    esp_now_add_peer(&peerInfo)
    != ESP_OK
  ) {

    Serial.println("PEER ADD FAILED");

    while (true) {
      delay(1000);
    }
  }


  packet.sequence = 0;

  Serial.println("GYRO SENDER READY");
}


void loop() {

  float gx;
  float gy;
  float gz;


  // ================================
  // 자이로 데이터 읽기
  // ================================

  if (!readGyroscope(gx, gy, gz)) {

    Serial.println("GYRO READ FAILED");

    delay(100);

    return;
  }


  // ================================
  // 첫 번째 측정값 저장
  // ================================

  if (firstReading) {

    previousGX = gx;
    previousGY = gy;
    previousGZ = gz;

    firstReading = false;

    delay(50);

    return;
  }


  // ================================
  // 현재 자이로 값의 크기 계산
  // ================================

  float gyroMagnitude =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );


  // ================================
  // 회전 속도 → 진동 세기
  // ================================

  const float MIN_GYRO = 5.0;
  const float MAX_GYRO = 250.0;

  int intensity = 0;


  if (gyroMagnitude > MIN_GYRO) {

    float normalized =
      (gyroMagnitude - MIN_GYRO) /
      (MAX_GYRO - MIN_GYRO);

    normalized =
      constrain(
        normalized,
        0.0,
        1.0
      );

    intensity =
      normalized * 127;
  }


  // ================================
  // ESP-NOW 전송
  // ================================

  packet.sequence++;

  packet.movement =
    gyroMagnitude;

  packet.intensity =
    intensity;


  esp_err_t result =
    esp_now_send(
      broadcastAddress,
      (uint8_t *)&packet,
      sizeof(packet)
    );


  // ================================
  // Serial Monitor
  // ================================

  if (packet.sequence % 10 == 0) {

    Serial.print("GX: ");
    Serial.print(gx, 2);

    Serial.print(" | GY: ");
    Serial.print(gy, 2);

    Serial.print(" | GZ: ");
    Serial.print(gz, 2);

    Serial.print(" | Gyro: ");
    Serial.print(gyroMagnitude, 2);

    Serial.print(" deg/s");

    Serial.print(" | Intensity: ");
    Serial.print(intensity);

    Serial.print(" | Send: ");

    Serial.println(
      result == ESP_OK
      ? "OK"
      : "FAILED"
    );
  }


  // 초당 약 20회 측정
  delay(50);
}