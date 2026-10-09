
/*
  SOMA HAPTIC WEARABLE
  ESP32 + MPU6050 / ESP-NOW SENDER

  VERSION: ACC DIRECT CUT

  ACC:
  - 초기 코드의 가속도 변화량 계산 복원
  - Attack 없음
  - Release 없음
  - Fade Out 없음
  - 변화량이 임계값 이하이면 즉시 출력 0

  GYRO:
  - 회전 속도 기반
  - 기존 Envelope 유지

  MIX:
  - 복합 가속도 + 자이로
  - 기존 Envelope 유지

  Receiver Protocol Version: 2
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// ==================================================
// HARDWARE
// ==================================================

const uint8_t MPU_ADDR = 0x68;

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const uint8_t ESPNOW_CHANNEL = 1;

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

const uint8_t PROTOCOL_VERSION = 2;

// ==================================================
// SENSOR MODE
// ==================================================

enum SensorMode : uint8_t {
  ACCELERATION = 0,
  GYRO = 1,
  ACCEL_GYRO = 2
};

SensorMode sensorMode = ACCEL_GYRO;

// ==================================================
// TIME / DELAY
// ==================================================

// 초기 코드와 비슷하게 50ms부터 시작
uint32_t sampleIntervalMs = 50;

uint16_t selectedDelayMs = 0;

uint32_t lastSampleMs = 0;
uint32_t lastPrintMs = 0;

bool firstReading = true;

// ==================================================
// ACC DIRECT PARAMETERS
// ==================================================

// 초기 ACC 코드의 정규화 범위
const float ACC_DIRECT_MIN = 0.03f;
const float ACC_DIRECT_MAX = 1.00f;

// ACC 민감도
float accSensitivity = 1.0f;

// ==================================================
// MIX ACC PARAMETERS
// ==================================================

// MIX에는 기존 복합 ACC 계산을 유지

const float ACC_FULL_SCALE_G_PER_S = 14.0f;
const float ACC_MIN_RATE_G_PER_S = 0.20f;

const float GRAVITY_TRACK_TAU_SEC = 0.35f;

const float DYNAMIC_ACCEL_FULL_SCALE_G = 0.38f;
const float DYNAMIC_ACCEL_MIN_G = 0.012f;

const float ACC_JERK_WEIGHT = 0.30f;
const float ACC_DYNAMIC_WEIGHT = 0.70f;

// ==================================================
// GYRO PARAMETERS
// ==================================================

const float GYRO_FULL_SCALE_DPS = 220.0f;
const float GYRO_MIN_DPS = 1.50f;

// ==================================================
// MIX WEIGHTS
// ==================================================

const float ACC_MIX_WEIGHT = 0.65f;
const float GYRO_MIX_WEIGHT = 0.35f;

// ==================================================
// ENVELOPE
// GYRO / MIX ONLY
// ==================================================

const float ATTACK_TAU_SEC = 0.008f;
const float RELEASE_TAU_SEC = 0.045f;

float brakeReleaseTauSec = 0.12f;

const float BRAKE_START_GAP = 0.08f;
const float BRAKE_FULL_GAP = 0.33f;

float outputEnvelope = 0.0f;

// ==================================================
// MOTOR OUTPUT
// ==================================================

// 현재 버전의 최대 출력
const int MAX_MOTOR_INTENSITY = 105;

float outputGain = 1.0f;

// ==================================================
// ESP-NOW PACKET
// 수신부 프로토콜 2와 동일
// ==================================================

struct MotionPacket {

  uint32_t sequence;
  float movement;
  uint8_t intensity;
  uint8_t sensorMode;
  uint16_t delayMs;
  uint8_t protocolVersion;

};

uint32_t sequenceNumber = 0;

uint32_t espNowSendErrors = 0;
uint32_t i2cReadErrors = 0;

// ==================================================
// SENSOR STATE
// ==================================================

float previousAX = 0;
float previousAY = 0;
float previousAZ = 0;

float gravityX = 0;
float gravityY = 0;
float gravityZ = 1;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

float accelNoiseMagnitudeThresholdG =
  DYNAMIC_ACCEL_MIN_G;

float accelNoiseDeltaThresholdG = 0;

float gyroNoiseThresholdDps =
  GYRO_MIN_DPS;

bool calibrated = false;

// ==================================================
// SERIAL COMMAND BUFFER
// ==================================================

char commandBuffer[40];
uint8_t commandLength = 0;

// ==================================================
// UTILITY
// ==================================================

float clamp01(float value) {

  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;

  return value;
}

// ==================================================
// MPU6050 READ
// ==================================================

int16_t readSigned16() {

  uint8_t highByte = (uint8_t)Wire.read();
  uint8_t lowByte  = (uint8_t)Wire.read();

  return (int16_t)(
    ((uint16_t)highByte << 8) | lowByte
  );
}

bool readMPU(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  int received = Wire.requestFrom(
    (uint8_t)MPU_ADDR,
    (uint8_t)14,
    (uint8_t)true
  );

  if (received != 14) {
    return false;
  }

  int16_t rawAX = readSigned16();
  int16_t rawAY = readSigned16();
  int16_t rawAZ = readSigned16();

  readSigned16(); // Temperature

  int16_t rawGX = readSigned16();
  int16_t rawGY = readSigned16();
  int16_t rawGZ = readSigned16();

  // ±2g
  ax = rawAX / 16384.0f;
  ay = rawAY / 16384.0f;
  az = rawAZ / 16384.0f;

  // ±250 deg/s
  gx = rawGX / 131.0f;
  gy = rawGY / 131.0f;
  gz = rawGZ / 131.0f;

  return true;
}

// ==================================================
// MPU CONFIGURATION
// ==================================================

bool writeMPU(uint8_t reg, uint8_t value) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(reg);
  Wire.write(value);

  return Wire.endTransmission(true) == 0;
}

bool configureMPU() {

  if (!writeMPU(0x6B, 0x01)) {
    return false;
  }

  delay(100);

  return (
    writeMPU(0x1A, 0x04) &&
    writeMPU(0x19, 0x04) &&
    writeMPU(0x1B, 0x00) &&
    writeMPU(0x1C, 0x00)
  );
}

// ==================================================
// ESP-NOW SEND
// ==================================================

void sendPacket(
  float movement,
  uint8_t intensity
) {

  MotionPacket packet = {};

  packet.sequence = sequenceNumber++;
  packet.movement = movement;
  packet.intensity = intensity;

  packet.sensorMode = (uint8_t)sensorMode;
  packet.delayMs = selectedDelayMs;
  packet.protocolVersion = PROTOCOL_VERSION;

  esp_err_t result = esp_now_send(
    broadcastAddress,
    (uint8_t *)&packet,
    sizeof(packet)
  );

  if (result != ESP_OK) {
    espNowSendErrors++;
  }
}

// ==================================================
// CALIBRATION
// ==================================================

void calibrateSensor() {

  const int N = 120;
  const uint32_t INTERVAL_MS = 20;

  Serial.println();
  Serial.println("CALIBRATION START");
  Serial.println("KEEP SENSOR STILL");

  float sumAX = 0;
  float sumAY = 0;
  float sumAZ = 0;

  float sumAX2 = 0;
  float sumAY2 = 0;
  float sumAZ2 = 0;

  float sumGX = 0;
  float sumGY = 0;
  float sumGZ = 0;

  float sumGX2 = 0;
  float sumGY2 = 0;
  float sumGZ2 = 0;

  float sumDelta = 0;
  float sumDelta2 = 0;

  float pAX = 0;
  float pAY = 0;
  float pAZ = 0;

  bool havePrev = false;

  int valid = 0;
  int deltas = 0;

  for (int i = 0; i < N; i++) {

    uint32_t tick = millis();

    float ax, ay, az;
    float gx, gy, gz;

    if (readMPU(ax, ay, az, gx, gy, gz)) {

      sumAX += ax;
      sumAY += ay;
      sumAZ += az;

      sumAX2 += ax * ax;
      sumAY2 += ay * ay;
      sumAZ2 += az * az;

      sumGX += gx;
      sumGY += gy;
      sumGZ += gz;

      sumGX2 += gx * gx;
      sumGY2 += gy * gy;
      sumGZ2 += gz * gz;

      valid++;

      if (havePrev) {

        float delta =
          fabsf(ax - pAX) +
          fabsf(ay - pAY) +
          fabsf(az - pAZ);

        sumDelta += delta;
        sumDelta2 += delta * delta;

        deltas++;
      }

      pAX = ax;
      pAY = ay;
      pAZ = az;

      havePrev = true;
    }

    while (
      (uint32_t)(millis() - tick)
      < INTERVAL_MS
    ) {
      delay(1);
    }
  }

  if (valid < 100 || deltas < 80) {

    calibrated = false;
    firstReading = true;
    outputEnvelope = 0;

    Serial.println("CALIBRATION FAILED");
    return;
  }

  gravityX = sumAX / valid;
  gravityY = sumAY / valid;
  gravityZ = sumAZ / valid;

  float varAX = fmaxf(
    0.0f,
    sumAX2 / valid - gravityX * gravityX
  );

  float varAY = fmaxf(
    0.0f,
    sumAY2 / valid - gravityY * gravityY
  );

  float varAZ = fmaxf(
    0.0f,
    sumAZ2 / valid - gravityZ * gravityZ
  );

  accelNoiseMagnitudeThresholdG = fmaxf(
    DYNAMIC_ACCEL_MIN_G,
    3.0f * sqrtf(varAX + varAY + varAZ)
  );

  gyroOffsetX = sumGX / valid;
  gyroOffsetY = sumGY / valid;
  gyroOffsetZ = sumGZ / valid;

  float varGX = fmaxf(
    0.0f,
    sumGX2 / valid - gyroOffsetX * gyroOffsetX
  );

  float varGY = fmaxf(
    0.0f,
    sumGY2 / valid - gyroOffsetY * gyroOffsetY
  );

  float varGZ = fmaxf(
    0.0f,
    sumGZ2 / valid - gyroOffsetZ * gyroOffsetZ
  );

  gyroNoiseThresholdDps = fmaxf(
    GYRO_MIN_DPS,
    3.0f * sqrtf(varGX + varGY + varGZ)
  );

  float meanDelta = sumDelta / deltas;

  float sigmaDelta = sqrtf(
    fmaxf(
      0.0f,
      sumDelta2 / deltas -
      meanDelta * meanDelta
    )
  );

  accelNoiseDeltaThresholdG =
    meanDelta + 3.0f * sigmaDelta;

  calibrated = true;
  firstReading = true;
  outputEnvelope = 0;

  lastSampleMs = millis();

  Serial.println("CALIBRATION COMPLETE");
}

// ==================================================
// STATUS
// ==================================================

void printStatus() {

  Serial.println();
  Serial.println("========== STATUS ==========");

  Serial.print("Mode: ");

  if (sensorMode == ACCELERATION)
    Serial.println("ACC DIRECT");
  else if (sensorMode == GYRO)
    Serial.println("GYRO");
  else
    Serial.println("MIX");

  Serial.print("Delay: ");
  Serial.println(selectedDelayMs);

  Serial.print("Sample interval: ");
  Serial.println(sampleIntervalMs);

  Serial.print("GAIN: ");
  Serial.println(outputGain, 2);

  Serial.print("ASENS: ");
  Serial.println(accSensitivity, 2);

  Serial.print("BRAKE: ");
  Serial.println(brakeReleaseTauSec, 3);

  Serial.print("Calibration: ");
  Serial.println(calibrated ? "YES" : "NO");

  Serial.print("I2C Errors: ");
  Serial.println(i2cReadErrors);

  Serial.print("Send Errors: ");
  Serial.println(espNowSendErrors);

  Serial.println("============================");
}

// ==================================================
// HELP
// ==================================================

void printHelp() {

  Serial.println();
  Serial.println("========== COMMANDS ==========");

  Serial.println("ACC");
  Serial.println("GYRO");
  Serial.println("MIX");

  Serial.println("DELAY 0");
  Serial.println("DELAY 3");
  Serial.println("DELAY 5");

  Serial.println("RATE 20");
  Serial.println("RATE 50");

  Serial.println("GAIN 1.0");
  Serial.println("ASENS 1.0");
  Serial.println("BRAKE 0.12");

  Serial.println("CAL");
  Serial.println("STATUS");
  Serial.println("HELP");

  Serial.println("==============================");
}

// ==================================================
// SERIAL COMMAND
// ==================================================

void executeCommand(const char *cmd) {

  if (!strcmp(cmd, "ACC")) {

    sensorMode = ACCELERATION;

    firstReading = true;
    outputEnvelope = 0;

    sendPacket(0, 0);

    Serial.println("MODE -> ACC DIRECT CUT");
  }

  else if (!strcmp(cmd, "GYRO")) {

    sensorMode = GYRO;

    firstReading = true;
    outputEnvelope = 0;

    sendPacket(0, 0);

    Serial.println("MODE -> GYRO");
  }

  else if (!strcmp(cmd, "MIX")) {

    sensorMode = ACCEL_GYRO;

    firstReading = true;
    outputEnvelope = 0;

    sendPacket(0, 0);

    Serial.println("MODE -> MIX");
  }

  else if (!strcmp(cmd, "DELAY 0")) {
    selectedDelayMs = 0;
    Serial.println("DELAY -> 0");
  }

  else if (!strcmp(cmd, "DELAY 3")) {
    selectedDelayMs = 3000;
    Serial.println("DELAY -> 3000");
  }

  else if (!strcmp(cmd, "DELAY 5")) {
    selectedDelayMs = 5000;
    Serial.println("DELAY -> 5000");
  }

  else if (!strncmp(cmd, "RATE ", 5)) {

    int value = atoi(cmd + 5);

    if (value >= 5 && value <= 100) {

      sampleIntervalMs = (uint32_t)value;

      firstReading = true;
      outputEnvelope = 0;

      sendPacket(0, 0);

      Serial.print("RATE -> ");
      Serial.println(sampleIntervalMs);
    }
  }

  else if (!strncmp(cmd, "GAIN ", 5)) {

    float value = atof(cmd + 5);

    if (value >= 0.5f && value <= 1.5f) {

      outputGain = value;

      Serial.print("GAIN -> ");
      Serial.println(outputGain, 2);
    }
  }

  else if (!strncmp(cmd, "ASENS ", 6)) {

    float value = atof(cmd + 6);

    if (value >= 0.5f && value <= 3.0f) {

      accSensitivity = value;

      Serial.print("ASENS -> ");
      Serial.println(accSensitivity, 2);
    }
  }

  else if (!strncmp(cmd, "BRAKE ", 6)) {

    float value = atof(cmd + 6);

    if (
      value >= RELEASE_TAU_SEC &&
      value <= 0.25f
    ) {

      brakeReleaseTauSec = value;

      Serial.print("BRAKE -> ");
      Serial.println(brakeReleaseTauSec, 3);
    }
  }

  else if (!strcmp(cmd, "CAL")) {

    sendPacket(0, 0);
    calibrateSensor();
    sendPacket(0, 0);
  }

  else if (!strcmp(cmd, "STATUS")) {
    printStatus();
  }

  else if (!strcmp(cmd, "HELP")) {
    printHelp();
  }

  else {
    Serial.println("UNKNOWN COMMAND");
  }
}

void pollSerialCommand() {

  while (Serial.available() > 0) {

    char ch = (char)Serial.read();

    if (ch == '\n' || ch == '\r') {

      if (commandLength > 0) {

        commandBuffer[commandLength] = '\0';

        executeCommand(commandBuffer);

        commandLength = 0;
      }
    }

    else if (
      commandLength <
      sizeof(commandBuffer) - 1
    ) {

      commandBuffer[commandLength++] =
        (char)toupper((unsigned char)ch);
    }

    else {

      commandLength = 0;

      Serial.println("COMMAND TOO LONG");
    }
  }
}

// ==================================================
// SETUP
// ==================================================

void setup() {

  Serial.begin(115200);
  delay(500);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // MPU6050
  if (!configureMPU()) {

    Serial.println("MPU6050 ERROR");

    while (true) {
      delay(1000);
    }
  }

  // CALIBRATION
  calibrateSensor();

  // WIFI
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");

    while (true) {
      delay(1000);
    }
  }

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_wifi_set_channel(
    ESPNOW_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    broadcastAddress,
    6
  );

  peerInfo.channel = ESPNOW_CHANNEL;
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

  lastSampleMs = millis();

  Serial.println();
  Serial.println("SOMA HAPTIC SENDER READY");
  Serial.println("ACC DIRECT CUT VERSION");

  printHelp();
  printStatus();
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {

  pollSerialCommand();

  uint32_t now = millis();

  if (
    (uint32_t)(now - lastSampleMs)
    < sampleIntervalMs
  ) {
    return;
  }

  float dt =
    (now - lastSampleMs) / 1000.0f;

  lastSampleMs = now;

  // ================================================
  // 1. READ SENSOR
  // ================================================

  float ax, ay, az;
  float gx, gy, gz;

  if (!readMPU(ax, ay, az, gx, gy, gz)) {

    i2cReadErrors++;

    outputEnvelope = 0;
    firstReading = true;

    sendPacket(0, 0);
    return;
  }

  if (firstReading || dt > 0.25f) {

    previousAX = ax;
    previousAY = ay;
    previousAZ = az;

    firstReading = false;
    outputEnvelope = 0;

    sendPacket(0, 0);
    return;
  }

  // ================================================
  // 2. ACC DELTA
  // 초기 방식과 동일한 변화량
  // ================================================

  float deltaX =
    fabsf(ax - previousAX);

  float deltaY =
    fabsf(ay - previousAY);

  float deltaZ =
    fabsf(az - previousAZ);

  float deltaG =
    deltaX + deltaY + deltaZ;

  previousAX = ax;
  previousAY = ay;
  previousAZ = az;

  // ================================================
  // 3. ACC DIRECT NORMALIZATION
  // 초기 코드 방식
  // ================================================

  float accDirectNorm = 0.0f;

  if (deltaG > ACC_DIRECT_MIN) {

    accDirectNorm =
      (deltaG - ACC_DIRECT_MIN) /
      (ACC_DIRECT_MAX - ACC_DIRECT_MIN);

    accDirectNorm = clamp01(
      accDirectNorm * accSensitivity
    );

  } else {

    // 중요!
    // 변화량이 작으면 즉시 0

    accDirectNorm = 0.0f;
  }

  // ================================================
  // 4. ACC RATE
  // MIX에서만 사용
  // ================================================

  float accelRate = deltaG / dt;

  float accelFloor = fmaxf(
    ACC_MIN_RATE_G_PER_S,
    accelNoiseDeltaThresholdG / dt
  );

  float jerkNorm = clamp01(
    (accelRate - accelFloor) /
    fmaxf(
      1.0f,
      ACC_FULL_SCALE_G_PER_S - accelFloor
    )
  );

  // ================================================
  // 5. DYNAMIC ACC
  // MIX에서만 사용
  // ================================================

  float dynamicX = ax - gravityX;
  float dynamicY = ay - gravityY;
  float dynamicZ = az - gravityZ;

  float dynamicG = sqrtf(
    dynamicX * dynamicX +
    dynamicY * dynamicY +
    dynamicZ * dynamicZ
  );

  float gravityAlpha =
    1.0f - expf(
      -dt / GRAVITY_TRACK_TAU_SEC
    );

  gravityX +=
    (ax - gravityX) * gravityAlpha;

  gravityY +=
    (ay - gravityY) * gravityAlpha;

  gravityZ +=
    (az - gravityZ) * gravityAlpha;

  float dynamicNorm = clamp01(
    (dynamicG - accelNoiseMagnitudeThresholdG) /
    fmaxf(
      0.05f,
      DYNAMIC_ACCEL_FULL_SCALE_G -
      accelNoiseMagnitudeThresholdG
    )
  );

  float accMixNorm = clamp01(
    accSensitivity *
    (
      ACC_JERK_WEIGHT * jerkNorm +
      ACC_DYNAMIC_WEIGHT * dynamicNorm
    )
  );

  // ================================================
  // 6. GYRO
  // 기존 방식 유지
  // ================================================

  float cgx = gx - gyroOffsetX;
  float cgy = gy - gyroOffsetY;
  float cgz = gz - gyroOffsetZ;

  float gyroDps = sqrtf(
    cgx * cgx +
    cgy * cgy +
    cgz * cgz
  );

  float gyroFloor = gyroNoiseThresholdDps;

  float gyroNorm = clamp01(
    (gyroDps - gyroFloor) /
    fmaxf(
      1.0f,
      GYRO_FULL_SCALE_DPS - gyroFloor
    )
  );

  // ================================================
  // 7. SENSOR MODE SELECTION
  // ================================================

  float movement = 0.0f;

  if (sensorMode == ACCELERATION) {

    // ★ 초기 ACC 방식 복원
    // 가속도 변화량 직접 사용

    movement = accDirectNorm;
  }

  else if (sensorMode == GYRO) {

    // 기존 GYRO 유지

    movement = gyroNorm;
  }

  else {

    // 기존 MIX 유지

    movement =
      ACC_MIX_WEIGHT * accMixNorm +
      GYRO_MIX_WEIGHT * gyroNorm;
  }

  // ================================================
  // 8. TARGET
  // ================================================

  float target = clamp01(
    movement * outputGain
  );

  // ================================================
  // 9. ENVELOPE
  // ================================================

  if (sensorMode == ACCELERATION) {

    // ★ 핵심 수정!
    //
    // ACC에는 Attack 없음
    // ACC에는 Release 없음
    // ACC에는 Fade Out 없음
    //
    // 목표 출력값을 즉시 반영한다.
    //
    // movement = 0 이면
    // outputEnvelope = 0 이다.

    outputEnvelope = target;

  }

  else {

    // GYRO / MIX
    // 기존 Envelope 유지

    float releaseTau = RELEASE_TAU_SEC;

    if (target < outputEnvelope) {

      float fallingGap =
        outputEnvelope - target;

      float brakingBlend = clamp01(
        (fallingGap - BRAKE_START_GAP) /
        (BRAKE_FULL_GAP - BRAKE_START_GAP)
      );

      releaseTau +=
        brakingBlend *
        (brakeReleaseTauSec -
         RELEASE_TAU_SEC);
    }

    float tau =
      target >= outputEnvelope
      ? ATTACK_TAU_SEC
      : releaseTau;

    float alpha =
      1.0f - expf(-dt / tau);

    outputEnvelope +=
      (target - outputEnvelope) * alpha;

    if (outputEnvelope < 0.003f) {
      outputEnvelope = 0.0f;
    }
  }

  // ================================================
  // 10. INTENSITY
  // ================================================

  int intensity = (int)lroundf(
    outputEnvelope * MAX_MOTOR_INTENSITY
  );

  intensity = constrain(
    intensity,
    0,
    127
  );

  // ================================================
  // 11. ESP-NOW SEND
  // ================================================

  sendPacket(
    movement,
    (uint8_t)intensity
  );

  // ================================================
  // 12. SERIAL MONITOR
  // ================================================

  if (
    (uint32_t)(now - lastPrintMs)
    >= 300
  ) {

    lastPrintMs = now;

    Serial.print("MODE:");

    if (sensorMode == ACCELERATION)
      Serial.print("ACC");
    else if (sensorMode == GYRO)
      Serial.print("GYRO");
    else
      Serial.print("MIX");

    Serial.print(" | DeltaG:");
    Serial.print(deltaG, 4);

    Serial.print(" | ACC-Direct:");
    Serial.print(accDirectNorm, 3);

    Serial.print(" | ACC-Mix:");
    Serial.print(accMixNorm, 3);

    Serial.print(" | GYRO:");
    Serial.print(gyroNorm, 3);

    Serial.print(" | Movement:");
    Serial.print(movement, 3);

    Serial.print(" | ENV:");
    Serial.print(outputEnvelope, 3);

    Serial.print(" | OUT:");
    Serial.print(intensity);

    Serial.print(" | dt:");
    Serial.print(dt * 1000.0f, 0);

    Serial.println("ms");
  }
}