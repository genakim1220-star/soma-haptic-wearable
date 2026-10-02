#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =========================
// MPU6050
// =========================

const int MPU_ADDR = 0x68;

 
// =========================
// SENSOR MODE
// =========================

enum SensorMode {
  ACCELERATION,
  GYRO,
  ACCEL_GYRO
};

SensorMode sensorMode = ACCEL_GYRO;


// =========================
// DELAY MODE
// =========================

enum DelayMode {
  DELAY_0,
  DELAY_3S,
  DELAY_5S
};

DelayMode delayMode = DELAY_0;


// =========================
// CALIBRATION
// =========================

float accelOffsetX = 0;
float accelOffsetY = 0;
float accelOffsetZ = 0;

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

bool calibrated = false;


// =========================
// PREVIOUS SENSOR VALUES
// =========================

float previousX = 0;
float previousY = 0;
float previousZ = 0;

bool firstReading = true;


// =========================
// ESP-NOW
// =========================

uint8_t broadcastAddress[] = {
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF
};


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

uint32_t sequenceNumber = 0;


// =========================
// MPU READ
// =========================

void readMPU(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
) {

  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x3B);

  Wire.endTransmission(false);

  Wire.requestFrom(
    MPU_ADDR,
    14,
    true
  );


  int16_t rawAX =
    Wire.read() << 8 | Wire.read();

  int16_t rawAY =
    Wire.read() << 8 | Wire.read();

  int16_t rawAZ =
    Wire.read() << 8 | Wire.read();


  // Temperature
  Wire.read();
  Wire.read();


  int16_t rawGX =
    Wire.read() << 8 | Wire.read();

  int16_t rawGY =
    Wire.read() << 8 | Wire.read();

  int16_t rawGZ =
    Wire.read() << 8 | Wire.read();


  // ±2g

  ax = rawAX / 16384.0;
  ay = rawAY / 16384.0;
  az = rawAZ / 16384.0;


  // ±250 deg/s

  gx = rawGX / 131.0;
  gy = rawGY / 131.0;
  gz = rawGZ / 131.0;
}


// =========================
// DELAY VALUE
// =========================

uint16_t getDelayMs() {

  if (delayMode == DELAY_0)
    return 0;

  if (delayMode == DELAY_3S)
    return 3000;

  if (delayMode == DELAY_5S)
    return 5000;

  return 0;
}


// =========================
// CALIBRATION
// =========================

void calibrateSensor() {

  Serial.println();
  Serial.println("================================");
  Serial.println("CALIBRATION START");
  Serial.println("Keep the sensor completely still.");
  Serial.println("================================");


  const int samples = 100;


  float sumAX = 0;
  float sumAY = 0;
  float sumAZ = 0;

  float sumGX = 0;
  float sumGY = 0;
  float sumGZ = 0;


  for (int i = 0; i < samples; i++) {

    float ax, ay, az;
    float gx, gy, gz;


    readMPU(
      ax, ay, az,
      gx, gy, gz
    );


    sumAX += ax;
    sumAY += ay;
    sumAZ += az;

    sumGX += gx;
    sumGY += gy;
    sumGZ += gz;


    delay(20);
  }


  accelOffsetX =
    sumAX / samples;

  accelOffsetY =
    sumAY / samples;

  accelOffsetZ =
    sumAZ / samples;


  gyroOffsetX =
    sumGX / samples;

  gyroOffsetY =
    sumGY / samples;

  gyroOffsetZ =
    sumGZ / samples;


  calibrated = true;


  // 중요:
  // calibration 이후에는
  // 다음 센서값을 기준값으로 다시 잡는다.

  firstReading = true;


  Serial.println();
  Serial.println("CALIBRATION COMPLETE");

  Serial.print("Accel offset X: ");
  Serial.println(accelOffsetX);

  Serial.print("Accel offset Y: ");
  Serial.println(accelOffsetY);

  Serial.print("Accel offset Z: ");
  Serial.println(accelOffsetZ);

  Serial.print("Gyro offset X: ");
  Serial.println(gyroOffsetX);

  Serial.print("Gyro offset Y: ");
  Serial.println(gyroOffsetY);

  Serial.print("Gyro offset Z: ");
  Serial.println(gyroOffsetZ);

  Serial.println();
}


// =========================
// STATUS
// =========================

void printStatus() {

  Serial.println();
  Serial.println("========== STATUS ==========");


  Serial.print("Sensor Mode: ");

  if (sensorMode == ACCELERATION)
    Serial.println("ACCELERATION");

  else if (sensorMode == GYRO)
    Serial.println("GYRO");

  else
    Serial.println("ACC + GYRO");


  Serial.print("Delay: ");

  if (delayMode == DELAY_0)
    Serial.println("0 sec");

  else if (delayMode == DELAY_3S)
    Serial.println("3 sec");

  else
    Serial.println("5 sec");


  Serial.print("Calibration: ");

  if (calibrated)
    Serial.println("YES");

  else
    Serial.println("NO");


  Serial.println("============================");
  Serial.println();
}


// =========================
// HELP
// =========================

void printHelp() {

  Serial.println();
  Serial.println("========== COMMANDS ==========");

  Serial.println("ACC        -> Acceleration only");
  Serial.println("GYRO       -> Gyro only");
  Serial.println("MIX        -> Acceleration + Gyro");

  Serial.println("DELAY 0    -> Immediate");
  Serial.println("DELAY 3    -> 3 second delay");
  Serial.println("DELAY 5    -> 5 second delay");

  Serial.println("CAL        -> Sensor calibration");
  Serial.println("STATUS     -> Current settings");
  Serial.println("HELP       -> Show commands");

  Serial.println("==============================");
  Serial.println();
}


// =========================
// SERIAL COMMAND
// =========================

void handleSerialCommand() {

  if (!Serial.available())
    return;


  String command =
    Serial.readStringUntil('\n');

  command.trim();
  command.toUpperCase();


  // =========================
  // SENSOR MODE
  // =========================

  if (command == "ACC") {

    sensorMode =
      ACCELERATION;

    // 모드 변경 후
    // 다음 센서값부터 새 기준으로 계산

    firstReading = true;

    Serial.println(
      "Sensor Mode -> ACCELERATION"
    );
  }


  else if (command == "GYRO") {

    sensorMode =
      GYRO;

    firstReading = true;

    Serial.println(
      "Sensor Mode -> GYRO"
    );
  }


  else if (command == "MIX") {

    sensorMode =
      ACCEL_GYRO;

    firstReading = true;

    Serial.println(
      "Sensor Mode -> ACC + GYRO"
    );
  }


  // =========================
  // DELAY
  // =========================

  else if (command == "DELAY 0") {

    delayMode =
      DELAY_0;

    Serial.println(
      "Delay -> 0 sec"
    );
  }


  else if (command == "DELAY 3") {

    delayMode =
      DELAY_3S;

    Serial.println(
      "Delay -> 3 sec"
    );
  }


  else if (command == "DELAY 5") {

    delayMode =
      DELAY_5S;

    Serial.println(
      "Delay -> 5 sec"
    );
  }


  // =========================
  // CALIBRATION
  // =========================

  else if (command == "CAL") {

    calibrateSensor();
  }


  // =========================
  // STATUS
  // =========================

  else if (command == "STATUS") {

    printStatus();
  }


  // =========================
  // HELP
  // =========================

  else if (command == "HELP") {

    printHelp();
  }


  else {

    Serial.println(
      "Unknown command."
    );

    Serial.println(
      "Type HELP"
    );
  }
}


// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);

  delay(1000);


  // =========================
  // I2C
  // =========================

  Wire.begin(21, 22);

  Wire.setClock(100000);


  // =========================
  // MPU6050 WAKE UP
  // =========================

  Wire.beginTransmission(
    MPU_ADDR
  );

  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission(true);


  delay(100);


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

    return;
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
    esp_now_add_peer(
      &peerInfo
    )
    != ESP_OK
  ) {

    Serial.println(
      "FAILED TO ADD PEER"
    );

    return;
  }


  // =========================
  // START
  // =========================

  Serial.println();
  Serial.println("================================");
  Serial.println(" SOMA HAPTIC SYSTEM");
  Serial.println("================================");


  printHelp();

  printStatus();
}


// =========================
// LOOP
// =========================

void loop() {

  // =========================
  // SERIAL COMMAND
  // =========================

  handleSerialCommand();


  // =========================
  // READ MPU
  // =========================

  float ax, ay, az;
  float gx, gy, gz;


  readMPU(
    ax, ay, az,
    gx, gy, gz
  );


  // =========================
  // CALIBRATION OFFSET
  // =========================

  float correctedAX =
    ax - accelOffsetX;

  float correctedAY =
    ay - accelOffsetY;

  float correctedAZ =
    az - accelOffsetZ;


  float correctedGX =
    gx - gyroOffsetX;

  float correctedGY =
    gy - gyroOffsetY;

  float correctedGZ =
    gz - gyroOffsetZ;


  // ==================================================
  // 첫 번째 측정값
  // ==================================================

  // 이것이 예전 ACC 코드와 맞추기 위한
  // 중요한 부분.

  if (firstReading) {

    previousX =
      correctedAX;

    previousY =
      correctedAY;

    previousZ =
      correctedAZ;

    firstReading = false;

    delay(50);

    return;
  }


  // ==================================================
  // ACCELERATION
  // ==================================================

  float differenceX =
    abs(
      correctedAX
      - previousX
    );


  float differenceY =
    abs(
      correctedAY
      - previousY
    );


  float differenceZ =
    abs(
      correctedAZ
      - previousZ
    );


  // 예전 코드와 동일한 방식
  //
  // 현재값 - 이전값
  // ↓
  // 각 축 변화량
  // ↓
  // 세 축을 더함

  float accelerationMovement =
    differenceX
    + differenceY
    + differenceZ;


  // ==================================================
  // GYRO
  // ==================================================

  float gyroMagnitude =
    sqrt(
      correctedGX * correctedGX
      +
      correctedGY * correctedGY
      +
      correctedGZ * correctedGZ
    );


  // ==================================================
  // ACC NORMALIZATION
  // ==================================================

  const float MIN_ACCEL =
    0.03;

  const float MAX_ACCEL =
    1.00;


  float accelerationNormalized =
    0;


  if (
    accelerationMovement
    > MIN_ACCEL
  ) {

    accelerationNormalized =
      (
        accelerationMovement
        - MIN_ACCEL
      )
      /
      (
        MAX_ACCEL
        - MIN_ACCEL
      );


    accelerationNormalized =
      constrain(
        accelerationNormalized,
        0.0,
        1.0
      );
  }


  // ==================================================
  // GYRO NORMALIZATION
  // ==================================================

  const float MIN_GYRO =
    5.0;

  const float MAX_GYRO =
    250.0;


  float gyroNormalized =
    0;


  if (
    gyroMagnitude
    > MIN_GYRO
  ) {

    gyroNormalized =
      (
        gyroMagnitude
        - MIN_GYRO
      )
      /
      (
        MAX_GYRO
        - MIN_GYRO
      );


    gyroNormalized =
      constrain(
        gyroNormalized,
        0.0,
        1.0
      );
  }


  // ==================================================
  // SENSOR MODE
  // ==================================================

  float movement = 0;


  // -------------------------
  // ACC
  // -------------------------

  if (
    sensorMode
    == ACCELERATION
  ) {

    // ACC는 smoothing 없음.
    // 순간적인 변화량을
    // 그대로 사용한다.

    movement =
      accelerationNormalized;
  }


  // -------------------------
  // GYRO
  // -------------------------

  else if (
    sensorMode
    == GYRO
  ) {

    movement =
      gyroNormalized;
  }


  // -------------------------
  // MIX
  // -------------------------

  else if (
    sensorMode
    == ACCEL_GYRO
  ) {

    movement =
      (
        accelerationNormalized
        * 0.5
      )
      +
      (
        gyroNormalized
        * 0.5
      );
  }


  // ==================================================
  // INTENSITY
  // ==================================================

  int intensity =
    movement * 127;


  intensity =
    constrain(
      intensity,
      0,
      127
    );


  // ==================================================
  // PACKET
  // ==================================================

  MotionPacket packet;


  packet.sequence =
    sequenceNumber++;


  packet.movement =
    movement;


  packet.intensity =
    intensity;


  packet.sensorMode =
    sensorMode;


  packet.delayMs =
    getDelayMs();


  // ==================================================
  // ESP-NOW SEND
  // ==================================================

  esp_now_send(
    broadcastAddress,
    (uint8_t *)&packet,
    sizeof(packet)
  );


  // ==================================================
  // PREVIOUS VALUE
  // ==================================================

  previousX =
    correctedAX;

  previousY =
    correctedAY;

  previousZ =
    correctedAZ;


  // ==================================================
  // SERIAL MONITOR
  // ==================================================

  // 매번 출력하지 않고
  // 500ms마다 한 번만 출력

  static uint32_t lastPrint =
    0;


  if (
    millis() - lastPrint
    >= 500
  ) {

    lastPrint =
      millis();


    Serial.print(
      "Mode: "
    );


    if (
      sensorMode
      == ACCELERATION
    )
      Serial.print("ACC");

    else if (
      sensorMode
      == GYRO
    )
      Serial.print("GYRO");

    else
      Serial.print("MIX");


    Serial.print(
      " | ACC: "
    );

    Serial.print(
      accelerationMovement,
      3
    );


    Serial.print(
      " | ACC Norm: "
    );

    Serial.print(
      accelerationNormalized,
      2
    );


    Serial.print(
      " | GYRO: "
    );

    Serial.print(
      gyroMagnitude,
      2
    );


    Serial.print(
      " | GYRO Norm: "
    );

    Serial.print(
      gyroNormalized,
      2
    );


    Serial.print(
      " | Movement: "
    );

    Serial.print(
      movement,
      2
    );


    Serial.print(
      " | Intensity: "
    );

    Serial.print(
      intensity
    );


    Serial.print(
      " | Delay: "
    );

    Serial.print(
      getDelayMs()
    );

    Serial.println(
      " ms"
    ); 
  }


  // =========================
  // SAMPLE RATE
  // =========================

  delay(50);
}