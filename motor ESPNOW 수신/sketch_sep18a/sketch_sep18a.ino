#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_DRV2605.h>
#include <esp_arduino_version.h>

Adafruit_DRV2605 drv;


// ==================================================
// 송신기와 동일한 데이터 구조
// ==================================================

struct MotionPacket {
  uint32_t sequence;
  float movement;
  uint8_t intensity;
};


// ==================================================
// ⭐ 3초 딜레이 설정
// ==================================================

const uint32_t DELAY_MS = 3000;


// ==================================================
// ⭐ 딜레이 버퍼
// ==================================================

// 송신기가 약 50ms마다 데이터를 보내므로
// 3초 동안 약 60개의 데이터를 저장

const int BUFFER_SIZE = 70;

struct DelayedPacket {
  uint8_t intensity;
  uint32_t receiveTime;
};

DelayedPacket buffer[BUFFER_SIZE];

int writeIndex = 0;


// ==================================================
// 버튼
// ==================================================

const int BUTTON_PIN = 25;

bool motorEnabled = true;
bool previousButtonState = HIGH;


// ==================================================
// ESP-NOW 데이터 수신
// ==================================================

#if ESP_ARDUINO_VERSION_MAJOR >= 3

void onDataReceived(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int length
) {

  if (length != sizeof(MotionPacket)) {
    return;
  }

  MotionPacket packet;

  memcpy(
    &packet,
    data,
    sizeof(packet)
  );


  // ==================================================
  // ⭐ 받은 intensity와 받은 시간을 버퍼에 저장
  // ==================================================

  buffer[writeIndex].intensity = packet.intensity;
  buffer[writeIndex].receiveTime = millis();

  writeIndex++;

  if (writeIndex >= BUFFER_SIZE) {
    writeIndex = 0;
  }
}

#else

void onDataReceived(
  const uint8_t *macAddress,
  const uint8_t *data,
  int length
) {

  if (length != sizeof(MotionPacket)) {
    return;
  }

  MotionPacket packet;

  memcpy(
    &packet,
    data,
    sizeof(packet)
  );


  // ==================================================
  // ⭐ 받은 intensity와 받은 시간을 버퍼에 저장
  // ==================================================

  buffer[writeIndex].intensity = packet.intensity;
  buffer[writeIndex].receiveTime = millis();

  writeIndex++;

  if (writeIndex >= BUFFER_SIZE) {
    writeIndex = 0;
  }
}

#endif


// ==================================================
// SETUP
// ==================================================

void setup() {

  Serial.begin(115200);

  delay(2000);


  // ==================================================
  // 버튼
  // ==================================================

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );


  // ==================================================
  // DRV2605L I2C
  // ==================================================

  Wire.begin(21, 22);

  Wire.setClock(100000);


  if (!drv.begin()) {

    Serial.println(
      "DRV2605L NOT FOUND"
    );

    while (true) {
      delay(1000);
    }
  }


  drv.useERM();


  // 실시간 진동 세기 모드
  drv.setMode(
    DRV2605_MODE_REALTIME
  );

  drv.setRealtimeValue(0);


  // ==================================================
  // ESP-NOW
  // ==================================================

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();


  esp_wifi_set_channel(
    1,
    WIFI_SECOND_CHAN_NONE
  );


  if (
    esp_now_init()
    != ESP_OK
  ) {

    Serial.println(
      "ESP-NOW INIT FAILED"
    );

    while (true) {
      delay(1000);
    }
  }


  esp_now_register_recv_cb(
    onDataReceived
  );


  Serial.println(
    "RECEIVER READY"
  );

  Serial.println(
    "3 SECOND DELAY MODE"
  );
}


// ==================================================
// LOOP
// ==================================================

void loop() {


  // ==================================================
  // 1. 버튼 확인
  // ==================================================

  bool currentButtonState =
    digitalRead(BUTTON_PIN);


  if (
    previousButtonState == HIGH &&
    currentButtonState == LOW
  ) {

    motorEnabled =
      !motorEnabled;


    Serial.print(
      "MOTOR: "
    );

    Serial.println(
      motorEnabled
      ? "ON"
      : "OFF"
    );


    delay(30);
  }


  previousButtonState =
    currentButtonState;


  // ==================================================
  // 2. 현재 시간
  // ==================================================

  uint32_t now = millis();


  // ==================================================
  // 3. 3초 전에 받은 데이터를 찾음
  // ==================================================

  uint8_t outputIntensity = 0;


  if (motorEnabled) {

    for (int i = 0; i < BUFFER_SIZE; i++) {

      uint32_t age =
        now - buffer[i].receiveTime;


      // ⭐ 정확히 3초 전의 데이터
      if (
        age >= DELAY_MS &&
        age < DELAY_MS + 100
      ) {

        outputIntensity =
          buffer[i].intensity;

        break;
      }
    }
  }


  // ==================================================
  // 4. 모터 출력
  // ==================================================

  drv.setRealtimeValue(
    outputIntensity
  );


  // ==================================================
  // 5. Serial Monitor
  // ==================================================

  static uint32_t previousPrintTime = 0;


  if (
    millis() - previousPrintTime >= 500
  ) {

    previousPrintTime =
      millis();


    Serial.print(
      "Delayed Intensity: "
    );

    Serial.print(
      outputIntensity
    );


    Serial.print(
      " | Motor: "
    );

    Serial.println(
      motorEnabled
      ? "ON"
      : "OFF"
    );
  }


  delay(10);
}