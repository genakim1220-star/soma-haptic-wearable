#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <Adafruit_DRV2605.h>
#include <esp_arduino_version.h>


// =========================
// DRV2605L
// =========================

Adafruit_DRV2605 drv;


// ========================= 
// BUTTON 
// =========================

const int BUTTON_PIN = 25; 

bool motorEnabled = true;
bool previousButtonState = HIGH;


// =========================
// PACKET
// =========================

struct MotionPacket {

  uint32_t sequence;

  float movement;

  uint8_t intensity;

  uint8_t sensorMode;

  uint16_t delayMs;
};


// =========================
// HISTORY BUFFER
// =========================

const int BUFFER_SIZE = 256;

struct HistoryPacket {

  uint32_t receiveTime;

  uint8_t intensity;

};

HistoryPacket history[BUFFER_SIZE];

volatile int writeIndex = 0;

volatile bool hasData = false;


// =========================
// LAST SETTINGS
// =========================

volatile uint16_t currentDelayMs = 0;

volatile uint8_t latestIntensity = 0;

volatile uint32_t lastPacketTime = 0;


// =========================
// ESP-NOW RECEIVE
// =========================

#if ESP_ARDUINO_VERSION_MAJOR >= 3

void onDataReceived(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len
) {

#else

void onDataReceived(
  const uint8_t *mac,
  const uint8_t *incomingData,
  int len
) {

#endif

  if (
    len != sizeof(MotionPacket)
  ) {
    return;
  }


  MotionPacket packet;

  memcpy(
    &packet,
    incomingData,
    sizeof(packet)
  );


  uint32_t now =
    millis();


  history[writeIndex].receiveTime =
    now;

  history[writeIndex].intensity =
    packet.intensity;


  writeIndex++;

  if (
    writeIndex >= BUFFER_SIZE
  ) {

    writeIndex = 0;
  }


  latestIntensity =
    packet.intensity;

  currentDelayMs =
    packet.delayMs;

  lastPacketTime =
    now;

  hasData =
    true;
}


// =========================
// FIND DELAYED VALUE
// =========================

uint8_t getDelayedIntensity(
  uint16_t delayMs
) {

  if (!hasData) {

    return 0;
  }


  uint32_t now =
    millis();


  // -------------------------
  // IMMEDIATE
  // -------------------------

  if (delayMs == 0) {

    return latestIntensity;
  }


  // -------------------------
  // TARGET TIME
  // -------------------------

  uint32_t targetTime =
    now - delayMs;


  uint8_t selectedIntensity = 0;

  uint32_t closestTime = 0;

  bool found = false;


  // Search history
  for (int i = 0; i < BUFFER_SIZE; i++) {

    uint32_t timestamp =
      history[i].receiveTime;

    uint8_t intensity =
      history[i].intensity;


    if (timestamp == 0)
      continue;


    // packet must be older than target
    if (timestamp <= targetTime) {

      if (
        !found ||
        timestamp > closestTime
      ) {

        closestTime =
          timestamp;

        selectedIntensity =
          intensity;

        found =
          true;
      }
    }
  }


  if (found) {

    return selectedIntensity;
  }


  return 0;
}


// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);


  // =========================
  // I2C
  // =========================

  Wire.begin(21, 22);


  // =========================
  // DRV2605L
  // =========================

  if (!drv.begin()) {

    Serial.println(
      "DRV2605L NOT FOUND"
    );

    while (1) {

      delay(100);
    }
  }


  drv.useERM();

  drv.setMode(
    DRV2605_MODE_REALTIME
  );

  drv.setRealtimeValue(0);


  // =========================
  // BUTTON
  // =========================

  pinMode(
    BUTTON_PIN,
    INPUT_PULLUP
  );


  // =========================
  // WIFI
  // =========================

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  esp_wifi_set_channel(
    1,
    WIFI_SECOND_CHAN_NONE
  );


  // =========================
  // ESP-NOW
  // =========================

  if (
    esp_now_init()
    != ESP_OK
  ) {

    Serial.println(
      "ESP-NOW INIT FAILED"
    );

    while (1) {

      delay(100);
    }
  }


  esp_now_register_recv_cb(
    onDataReceived
  );


  Serial.println();
  Serial.println(
    "SOMA HAPTIC RECEIVER READY"
  );
}


// =========================
// LOOP
// =========================

void loop() {


  // =========================
  // BUTTON
  // =========================

  bool buttonState =
    digitalRead(BUTTON_PIN);


  if (
    previousButtonState == HIGH &&
    buttonState == LOW
  ) {

    motorEnabled =
      !motorEnabled;


    Serial.print(
      "Motor: "
    );

    Serial.println(
      motorEnabled
      ? "ON"
      : "OFF"
    );


    delay(200);
  }


  previousButtonState =
    buttonState;


  // =========================
  // PACKET TIMEOUT
  // =========================

  if (
    millis() - lastPacketTime
    > 1000
  ) {

    drv.setRealtimeValue(0);

    return;
  }


  // =========================
  // DELAYED OUTPUT
  // =========================

  uint8_t outputIntensity =
    getDelayedIntensity(
      currentDelayMs
    );


  if (!motorEnabled) {

    outputIntensity = 0;
  }


  drv.setRealtimeValue(
    outputIntensity
  );


  // =========================
  // SERIAL
  // =========================

  static uint32_t lastPrint = 0;


  if (
    millis() - lastPrint
    > 500
  ) {

    lastPrint =
      millis();


    Serial.print(
      "Delay: "
    );

    Serial.print(
      currentDelayMs
    );

    Serial.print(
      " ms | Output: "
    );

    Serial.println(
      outputIntensity
    );
  }


  delay(5);
}