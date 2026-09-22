#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_DRV2605.h>
#include <esp_arduino_version.h>

Adafruit_DRV2605 drv;

// 송신기와 동일한 데이터 구조
struct MotionPacket {
  uint32_t sequence;
  float movement;
  uint8_t intensity;
};

volatile uint8_t receivedIntensity = 0;
volatile uint32_t lastPacketTime = 0;

const int BUTTON_PIN = 25;

bool motorEnabled = true;
bool previousButtonState = HIGH;

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
  memcpy(&packet, data, sizeof(packet));

  receivedIntensity = packet.intensity;
  lastPacketTime = millis();
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
  memcpy(&packet, data, sizeof(packet));

  receivedIntensity = packet.intensity;
  lastPacketTime = millis();
}

#endif

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // DRV2605L I2C 시작
  Wire.begin(27, 26);
  Wire.setClock(100000);

  if (!drv.begin()) {
    Serial.println("DRV2605L NOT FOUND");

    while (true) {
      delay(1000);
    }
  }

  drv.useERM();

  // 숫자로 진동 세기를 직접 지정하는 모드
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(0);

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

  esp_now_register_recv_cb(onDataReceived);

  Serial.println("RECEIVER READY");
}

void loop() {
  // 버튼 상태 확인
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // 버튼을 누르는 순간 ON/OFF 전환
  if (
    previousButtonState == HIGH &&
    currentButtonState == LOW
  ) {
    motorEnabled = !motorEnabled;

    Serial.print("MOTOR: ");
    Serial.println(motorEnabled ? "ON" : "OFF");

    delay(30);
  }

  previousButtonState = currentButtonState;

  uint8_t outputIntensity = 0;

  // 마지막 수신 후 500ms 이내이고 모터가 ON일 때만 진동
  if (
    motorEnabled &&
    millis() - lastPacketTime < 500
  ) {
    outputIntensity = receivedIntensity;
  }

  drv.setRealtimeValue(outputIntensity);

  static uint32_t previousPrintTime = 0;

  if (millis() - previousPrintTime >= 500) {
    previousPrintTime = millis();

    Serial.print("Intensity: ");
    Serial.print(outputIntensity);

    Serial.print(" | Motor: ");
    Serial.println(motorEnabled ? "ON" : "OFF");
  }

  delay(10);
}