#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <Adafruit_DRV2605.h>
#include <esp_arduino_version.h>
#include <string.h>  


// =========================
// DRV2605L
// ========================= 

Adafruit_DRV2605 drv;


// 마지막으로 I2C 에 실제로 써서 사용한 값.
// 같은 값을 반복해서 쓰지 않기 위한 비교용.

uint8_t lastWrittenRealtimeValue = 255;


// 송신 펌웨어와 맞아야 하는 프로토콜 버전.
// 어긋나면 조용히 무동작이 되므로
// 한 번이라도 다르면 경고를 찍는다.

const uint8_t PROTOCOL_VERSION = 2;

bool versionMismatchReported = false;


// 패킷 유실 / 지연 확인용 계기.

uint32_t receivedPackets = 0;
uint32_t droppedPackets = 0;
uint32_t lastSequence = 0;

bool haveLastSequence = false;


// =========================
// BUTTON
// =========================

const int BUTTON_PIN = 25;

bool motorEnabled = true;

// TRUE 이면 "이번 눌림에 대해 이미 토글했다".
// 길게 누를 때 중복 토글을 막는다.
// 초기값은 FALSE (아직 안 누름).

bool buttonToggleDone = false;


// =========================
// PACKET
// =========================

struct MotionPacket {

  uint32_t sequence;

  float movement;

  uint8_t intensity;

  uint8_t sensorMode;

  uint16_t delayMs;

  uint8_t protocolVersion;
};


// =========================
// HISTORY BUFFER
// =========================

// 버터 크기는 "최대 지연 시간"을 담을 수 있어야 한다.
//
// 5초 지연 + 샘플레이트 100Hz 를 쓰려면 최소 500칸이 필요하다.
// 여유를 두어 2048칸.
// (HistoryPacket 8바이트 * 2048 = 16KB, ESP32 에서 무리 없다.)

const int BUFFER_SIZE = 2048;

struct HistoryPacket {

  uint32_t receiveTime;

  uint8_t intensity;

};

HistoryPacket history[BUFFER_SIZE];

volatile int writeIndex = 0;

volatile bool hasData = false;


// 버퍼가 처음 채워졌는가.
// receiveTime == 0 을 "빈 칸" 표시로 쓰면
// millis() 가 우연히 0인 순간과 구분되지 않는다.

volatile bool bufferFilled = false;


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

  if (len != sizeof(MotionPacket)) {

    Serial.println(
      "PACKET SIZE MISMATCH"
    );

    Serial.print("  expected: ");
    Serial.println(sizeof(MotionPacket));

    Serial.print("  received: ");
    Serial.println(len);

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


  // -------------------------
  // PROTOCOL VERSION CHECK
  // -------------------------

  // 구조체 크기가 달라지면 len 검사만으로는
  // 못 잡는 경우(필드 순서 변경 등)가 있다.
  // 버전이 다르면 진동을 내지 않는다.
  // 조용히 이상하게 작동하는 게 실험에서는 더 위험하다.

  if (
    packet.protocolVersion
    != PROTOCOL_VERSION
  ) {

    if (!versionMismatchReported) {

      versionMismatchReported = true;

      Serial.print(
        "PROTOCOL MISMATCH: expected "
      );

      Serial.print(PROTOCOL_VERSION);
      Serial.print(", got ");
      Serial.println(
        packet.protocolVersion
      );
    }

    return;
  }


  // -------------------------
  // PACKET LOSS
  // -------------------------

  // sequence 로 빠진 패킷 수를 센다.
  // 유실률이 높으면 촉각 자체의 문제가 아니라
  // RF 문제이므로 실험 설계 단계에서 다뤄야 한다.

  if (haveLastSequence) {

    uint32_t diff =
      packet.sequence - lastSequence;

    if (diff > 1 && diff < 1000) {

      droppedPackets += diff - 1;
    }
  }

  lastSequence = packet.sequence;
  haveLastSequence = true;
  receivedPackets++;


  // -------------------------
  // HISTORY
  // -------------------------

  history[writeIndex].receiveTime =
    now;

  history[writeIndex].intensity =
    packet.intensity;


  writeIndex++;

  if (
    writeIndex >= BUFFER_SIZE
  ) {

    writeIndex = 0;

    bufferFilled = true;
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

// targetTime 이하인 값 중 "가장 최근"의 값을 찾는다.
// 버퍼는 원형이고 시간 순서대로 채워지므로,
// 최신 항목부터 거꾸로 훑다가 처음 조건을 만족하는 칸이 답이다.
//
// 기존 코드는 2048칸 전체를 매 루프마다 훑었다.
// 그것도 newest 와 oldest 를 구분하지 못해서
// 버퍼가 절반만 차도 뻔한 배열을 골랐다.
// millis() 는 unsigned 이므로
// now - delayMs 가 언더플로우하면
// 아주 먼 과거로 돌아가 버퍼 전체를 스캔하게 된다.
//
// 역방향 + 조기 종료로 바꾸면
// 평균적으로 한두 칸만 읽는다.

uint8_t getDelayedIntensity(
  uint16_t delayMs
) {

  if (!hasData) {

    return 0;
  }


  // -------------------------
  // IMMEDIATE
  // -------------------------

  if (delayMs == 0) {

    return latestIntensity;
  }


  uint32_t now =
    millis();


  // -------------------------
  // TARGET TIME
  // -------------------------

  // delayMs 는 uint16 이므로 최대 65535ms.
  // millis() 는 49.7일마다 한 바퀴 도는 32비트 값이라
  // 뺄셈은 unsigned 연산으로 안전하게 돌아간다.

  uint32_t targetTime =
    now - (uint32_t) delayMs;


  // -------------------------
  // OLDEST AVAILABLE
  // -------------------------

  // 버퍼가 한 바퀴를 아직 다 안 채웠다면,
  // 가장 오래된 칸이 어디인지 계산해서
  // "그보다 과거는 데이터가 없다"를 판별한다.

  uint32_t oldestTime =
    history[
      (
        writeIndex - 1
        + BUFFER_SIZE
      )
      % BUFFER_SIZE
    ].receiveTime;


  if (
    !bufferFilled
    &&
    targetTime < oldestTime
  ) {

    // 요청한 지연 시점이 기록된 범위보다 과거다.
    // 지연 시간을 줄이거나 충분히 데이터를 모은다.

    return 0;
  }


  // -------------------------
  // BACKWARD SCAN
  // -------------------------

  int index =
    (
      writeIndex - 1
      + BUFFER_SIZE
    )
    % BUFFER_SIZE;


  // 아직 다 안 찼으면 유효한 칸은 writeIndex 개뿐이다.
  // 그 이상을 훑으면 아직 안 쓴 칸(0)을 읽게 되고
  // 의미 없는 시간이 많이 낭비된다.

  int maxScans =
    bufferFilled
      ? BUFFER_SIZE
      : writeIndex;


  for (int scanned = 0; scanned < maxScans; scanned++) {

    uint32_t timestamp =
      history[index].receiveTime;


    if (
      timestamp
      != 0
      &&
      timestamp
      <= targetTime
    ) {

      return history[index].intensity;
    }


    index--;

    if (index < 0) {

      index =
        BUFFER_SIZE - 1;
    }
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


  // =========================
  // ACTUATOR TYPE
  // =========================

  // LRA (Linear Resonant Actuator) 라면 반드시 useLRA() 다.
  //
  // useERM() 으로 LRA 를 몰면
  // 구동 주기가 어긋나서 진동이 약해지거나
  // 전혀 안 느껴진다. 되게 서서히 흔들리게 느껴진다.
  //
  // 지금 연결된 액추에이터가 뭔지 정확히 확인한 뒤
  // 아래 한 줄만 바꾸면 된다.

  drv.useERM();

  // LRA 를 쓰는 경우 위의 useERM() 을 지우고
  // 아래 한 줄만 남긴다.
  // drv.useLRA();


  // =========================
  // REALTIME MODE
  // =========================

  // RT 모드는 진폭만 직접 제어하는 모드다.
  //
  // 이 연구에서는 진폭을 움직임의 에너지에 매핑하므로
  // RT 모드가 맞다.
  // 주파수는 액추에이터 고유진동수로 고정되고,
  // AM 펄스 레이트로间接 조절하게 된다.

  drv.setMode(
    DRV2605_MODE_REALTIME
  );

  drv.setRealtimeValue(0);

  lastWrittenRealtimeValue = 0;


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


  // -------------------------
  // WIFI POWER / CHANNEL
  // -------------------------

  // 이 두 설정은 esp_now_init 이 초기화 상태를 덮어쓸 수 있으므로
  // 반드시 그 뒤에 둔다.
  //
  // PS 를 켜두면 수신 패킷이 늦게 도착한다.
  // DELAY 0 으로 실험할 때도
  // 실질적으로 tens of ms 만큼 늦게 진동이 나가므로
  // "즉시 조건"(100ms 이내)이 깨진다.

  esp_wifi_set_ps(
    WIFI_PS_NONE
  );

  esp_wifi_set_channel(
    1,
    WIFI_SECOND_CHAN_NONE
  );


  esp_now_register_recv_cb(
    onDataReceived
  );


  Serial.println();
  Serial.println(
    "SOMA HAPTIC RECEIVER READY"
  );

  Serial.print("Protocol version: ");
  Serial.println(PROTOCOL_VERSION);

  Serial.println(
    "Expected packet size: "
  );

  Serial.println(sizeof(MotionPacket));

  Serial.println(
    "Update both firmwares together."
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

  uint32_t nowMs =
    millis();


  // -------------------------
  // DEBOUNCE
  // -------------------------

  // 기계적 접점에는 채터링이 있어서
  // 한 번 눌렀는데 여러 번 눌린 것처럼 읽힌다.
  //
  // 기존 코드는 delay(200) 으로 덮었다.
  // 그 200ms 동안 버튼을 계속 눌러 있어도
  // 상태가 뒤집히지 않고,
  // 동시에 진동 갱신도 200ms 동안 멈춘다.
  // 진동 갱신이 멈추는 건 별견 달성 축(속도 재현)에서
  // 치명적이므로 버린다.
  //
  // 대신 LOW 가 50ms 이상 계속되면
  // "한 번 눌린 것"으로 확정한다.
  // 어느 시점에도 루프를 멈추지 않는다.

  const uint32_t BUTTON_DEBOUNCE_MS =
    50;

  static bool lowSinceActive =
    false;

  static uint32_t lowStartMs =
    0;


  if (buttonState == LOW) {

    if (!lowSinceActive) {

      lowSinceActive = true;
      lowStartMs = nowMs;

    } else if (
      nowMs - lowStartMs
      >= BUTTON_DEBOUNCE_MS
    ) {

      // 아직 토글하지 않은 상태에서만 발화한다.
      // 길게 누르고 있어도 한 번만 토글된다.

      if (!buttonToggleDone) {

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

        buttonToggleDone =
          true;
      }
    }

  } else {

    lowSinceActive = false;

    // 뗄 때까지 true 를 유지해야
    // 다음 눌림에서 다시 토글된다.

    buttonToggleDone =
      false;
  }


  // =========================
  // PACKET TIMEOUT
  // =========================

  if (
    nowMs - lastPacketTime
    > 1000
  ) {

    if (lastWrittenRealtimeValue != 0) {

      drv.setRealtimeValue(0);

      lastWrittenRealtimeValue = 0;
    }

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


  // -------------------------
  // WRITE ONLY ON CHANGE
  // -------------------------

  // 5ms 루프마다 같은 값을 다시 쓰면
  // I2C 버스를 100% 점유한다.
  // 그 버스가 다른 센서/장비와 공유되면
  // 지터의 직접 원인이 된다.
  //
  // 값이 실제로 달라졌을 때만 쓴다.

  if (outputIntensity != lastWrittenRealtimeValue) {

    drv.setRealtimeValue(
      outputIntensity
    );

    lastWrittenRealtimeValue =
      outputIntensity;
  }


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

    Serial.print(
      outputIntensity
    );

    Serial.print(
      " | Rx: "
    );

    Serial.print(receivedPackets);

    Serial.print(
      " | Lost: "
    );

    Serial.print(droppedPackets);

    Serial.print(
      " | Motor: "
    );

    Serial.println(
      motorEnabled
        ? "ON"
        : "OFF"
    );
  }


  delay(5);
}