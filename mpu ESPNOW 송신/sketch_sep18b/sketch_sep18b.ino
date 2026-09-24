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


// ================================
// 전송 데이터
// ================================

struct MotionPacket {
  uint32_t sequence;
  float movement;
  uint8_t intensity;
};

MotionPacket packet;


// ================================
// 가속도 이전 값
// ================================

float previousX = 0;
float previousY = 0;
float previousZ = 0;

bool firstReading = true;


// ================================
// MPU-6050 가속도 + 자이로 읽기
// ================================

bool readMotion(
  float &x,
  float &y,
  float &z,
  float &gx,
  float &gy,
  float &gz
) {

  // MPU 데이터 시작 주소
  // 0x3B부터 시작하면
  // ACC X/Y/Z + 온도 + GYRO X/Y/Z를
  // 한 번에 읽을 수 있음

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return false;
  }


  // 총 14바이트
  // ACC 6
  // TEMP 2
  // GYRO 6

  int received = Wire.requestFrom(
    MPU_ADDRESS,
    (uint8_t)14,
    true
  );

  if (received != 14) {
    return false;
  }


  // ================================
  // 가속도 RAW 값
  // ================================

  int16_t rawX =
    (Wire.read() << 8) | Wire.read();

  int16_t rawY =
    (Wire.read() << 8) | Wire.read();

  int16_t rawZ =
    (Wire.read() << 8) | Wire.read();


  // ================================
  // 온도 데이터
  // 사용하지 않음
  // ================================

  Wire.read();
  Wire.read();


  // ================================
  // 자이로 RAW 값
  // ================================

  int16_t rawGX =
    (Wire.read() << 8) | Wire.read();

  int16_t rawGY =
    (Wire.read() << 8) | Wire.read();

  int16_t rawGZ =
    (Wire.read() << 8) | Wire.read();


  // ================================
  // 실제 단위로 변환
  // ================================

  // 가속도
  // 기본 설정 ±2g
  x = rawX / 16384.0;
  y = rawY / 16384.0;
  z = rawZ / 16384.0;


  // 자이로
  // 기본 설정 ±250 °/s
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


  // ================================
  // MPU 절전모드 해제
  // ================================

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

  Serial.println("ACC + GYRO SENDER READY");
}


void loop() {

  float x;
  float y;
  float z;

  float gx;
  float gy;
  float gz;


  // ================================
  // 가속도 + 자이로 읽기
  // ================================

  if (
    !readMotion(
      x,
      y,
      z,
      gx,
      gy,
      gz
    )
  ) {

    Serial.println("MPU READ FAILED");

    delay(100);

    return;
  }


  // ================================
  // 첫 번째 측정값 저장
  // ================================

  if (firstReading) {

    previousX = x;
    previousY = y;
    previousZ = z;

    firstReading = false;

    delay(50);

    return;
  }


  // ==================================================
  // 1. 가속도 변화량 계산
  // ==================================================

  float differenceX =
    abs(x - previousX);

  float differenceY =
    abs(y - previousY);

  float differenceZ =
    abs(z - previousZ);


  // 기존 코드와 동일한 방식
  float accelerationMovement =
    differenceX +
    differenceY +
    differenceZ;


  previousX = x;
  previousY = y;
  previousZ = z;


  // ==================================================
  // 2. 자이로 회전 속도 계산
  // ==================================================

  float gyroMagnitude =
    sqrt(
      gx * gx +
      gy * gy +
      gz * gz
    );


  // ==================================================
  // 3. 가속도 정규화
  // ==================================================

  const float MIN_ACCEL = 0.03;
  const float MAX_ACCEL = 1.00;

  float accelerationNormalized = 0;


  if (accelerationMovement > MIN_ACCEL) {

    accelerationNormalized =
      (accelerationMovement - MIN_ACCEL) /
      (MAX_ACCEL - MIN_ACCEL);

    accelerationNormalized =
      constrain(
        accelerationNormalized,
        0.0,
        1.0
      );
  }


  // ==================================================
  // 4. 자이로 정규화
  // ==================================================

  const float MIN_GYRO = 5.0;
  const float MAX_GYRO = 250.0;

  float gyroNormalized = 0;


  if (gyroMagnitude > MIN_GYRO) {

    gyroNormalized =
      (gyroMagnitude - MIN_GYRO) /
      (MAX_GYRO - MIN_GYRO);

    gyroNormalized =
      constrain(
        gyroNormalized,
        0.0,
        1.0
      );
  }


  // ==================================================
  // 5. 가속도 + 자이로 결합
  // ==================================================

  // 현재는 50 : 50
  float combinedMovement =
    (accelerationNormalized * 0.5) +
    (gyroNormalized * 0.5);


  // ==================================================
  // 6. 진동 세기로 변환
  // ==================================================

  int intensity =
    combinedMovement * 127;


  intensity =
    constrain(
      intensity,
      0,
      127
    );


  // ==================================================
  // 7. ESP-NOW 전송
  // ==================================================

  packet.sequence++;

  packet.movement =
    combinedMovement;

  packet.intensity =
    intensity;


  esp_err_t result =
    esp_now_send(
      broadcastAddress,
      (uint8_t *)&packet,
      sizeof(packet)
    );


  // ==================================================
  // 8. Serial Monitor
  // ==================================================

  if (packet.sequence % 10 == 0) {

    Serial.print("ACC: ");
    Serial.print(accelerationMovement, 3);

    Serial.print(" | ACC Norm: ");
    Serial.print(accelerationNormalized, 2);


    Serial.print(" | GYRO: ");
    Serial.print(gyroMagnitude, 2);

    Serial.print(" | GYRO Norm: ");
    Serial.print(gyroNormalized, 2);


    Serial.print(" | Combined: ");
    Serial.print(combinedMovement, 2);


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