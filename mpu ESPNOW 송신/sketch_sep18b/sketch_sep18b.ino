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
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
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

  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAX = Wire.read() << 8 | Wire.read();
  int16_t rawAY = Wire.read() << 8 | Wire.read();
  int16_t rawAZ = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  int16_t rawGX = Wire.read() << 8 | Wire.read();
  int16_t rawGY = Wire.read() << 8 | Wire.read();
  int16_t rawGZ = Wire.read() << 8 | Wire.read();

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

    readMPU(ax, ay, az, gx, gy, gz);

    sumAX += ax;
    sumAY += ay;
    sumAZ += az;

    sumGX += gx;
    sumGY += gy;
    sumGZ += gz;

    delay(20);
  }

  accelOffsetX = sumAX / samples;
  accelOffsetY = sumAY / samples;
  accelOffsetZ = sumAZ / samples;

  gyroOffsetX = sumGX / samples;
  gyroOffsetY = sumGY / samples;
  gyroOffsetZ = sumGZ / samples;

  calibrated = true;

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
// SERIAL COMMAND
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


void handleSerialCommand() {

  if (!Serial.available())
    return;

  String command = Serial.readStringUntil('\n');

  command.trim();
  command.toUpperCase();


  // -------------------------
  // SENSOR
  // -------------------------

  if (command == "ACC") {

    sensorMode = ACCELERATION;

    Serial.println("Sensor Mode → ACCELERATION");
  }


  else if (command == "GYRO") {

    sensorMode = GYRO;

    Serial.println("Sensor Mode → GYRO");
  }


  else if (command == "MIX") {

    sensorMode = ACCEL_GYRO;

    Serial.println("Sensor Mode → ACC + GYRO");
  }


  // -------------------------
  // DELAY
  // -------------------------

  else if (command == "DELAY 0") {

    delayMode = DELAY_0;

    Serial.println("Delay → 0 sec");
  }


  else if (command == "DELAY 3") {

    delayMode = DELAY_3S;

    Serial.println("Delay → 3 sec");
  }


  else if (command == "DELAY 5") {

    delayMode = DELAY_5S;

    Serial.println("Delay → 5 sec");
  }


  // -------------------------
  // CALIBRATION
  // -------------------------

  else if (command == "CAL") {

    calibrateSensor();
  }


  // -------------------------
  // STATUS
  // -------------------------

  else if (command == "STATUS") {

    printStatus();
  }


  // -------------------------
  // HELP
  // -------------------------

  else if (command == "HELP") {

    printHelp();
  }


  else {

    Serial.println("Unknown command.");
    Serial.println("Type HELP");
  }
}


// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  // MPU6050 wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);


  // -------------------------
  // WiFi / ESP-NOW
  // -------------------------

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  esp_wifi_set_channel(
    1,
    WIFI_SECOND_CHAN_NONE
  );


  if (esp_now_init() != ESP_OK) {

    Serial.println("ESP-NOW INIT FAILED");

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
    esp_now_add_peer(&peerInfo)
    != ESP_OK
  ) {

    Serial.println("FAILED TO ADD PEER");

    return;
  }


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

  handleSerialCommand();


  float ax, ay, az;
  float gx, gy, gz;

  readMPU(
    ax, ay, az,
    gx, gy, gz
  );


  // =========================
  // CALIBRATION OFFSET
  // =========================

  float correctedAX = ax - accelOffsetX;
  float correctedAY = ay - accelOffsetY;
  float correctedAZ = az - accelOffsetZ;

  float correctedGX = gx - gyroOffsetX;
  float correctedGY = gy - gyroOffsetY;
  float correctedGZ = gz - gyroOffsetZ;


  // =========================
  // ACCELERATION MOVEMENT
  // =========================

  float differenceX =
    abs(correctedAX - previousX);

  float differenceY =
    abs(correctedAY - previousY);

  float differenceZ =
    abs(correctedAZ - previousZ);


  float accelerationMovement =
    differenceX +
    differenceY +
    differenceZ;


  // =========================
  // GYRO MOVEMENT
  // =========================

  float gyroMagnitude =
    sqrt(
      correctedGX * correctedGX +
      correctedGY * correctedGY +
      correctedGZ * correctedGZ
    );


  // =========================
  // NORMALIZATION
  // =========================

  const float MIN_ACCEL = 0.03;
  const float MAX_ACCEL = 1.00;

  float accelerationNormalized =
    (accelerationMovement - MIN_ACCEL)
    /
    (MAX_ACCEL - MIN_ACCEL);

  accelerationNormalized =
    constrain(
      accelerationNormalized,
      0.0,
      1.0
    );


  const float MIN_GYRO = 5.0;
  const float MAX_GYRO = 250.0;

  float gyroNormalized =
    (gyroMagnitude - MIN_GYRO)
    /
    (MAX_GYRO - MIN_GYRO);

  gyroNormalized =
    constrain(
      gyroNormalized,
      0.0,
      1.0
    );


  // =========================
  // SENSOR MODE SELECTION
  // =========================

  float movement = 0;


  if (sensorMode == ACCELERATION) {

    movement =
      accelerationNormalized;
  }


  else if (sensorMode == GYRO) {

    movement =
      gyroNormalized;
  }


  else if (sensorMode == ACCEL_GYRO) {

    movement =
      (
        accelerationNormalized * 0.5
        +
        gyroNormalized * 0.5
      );
  }


  // =========================
  // INTENSITY
  // =========================

  int intensity =
    movement * 127;

  intensity =
    constrain(
      intensity,
      0,
      127
    );


  // =========================
  // PACKET
  // =========================

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


  esp_now_send(
    broadcastAddress,
    (uint8_t *)&packet,
    sizeof(packet)
  );


  // =========================
  // SERIAL MONITOR
  // =========================

  Serial.print("Movement: ");
  Serial.print(movement, 3);

  Serial.print(" | Intensity: ");
  Serial.print(intensity);

  Serial.print(" | Delay: ");
  Serial.print(getDelayMs());

  Serial.println(" ms");


  // =========================
  // SAVE PREVIOUS
  // =========================

  previousX = correctedAX;
  previousY = correctedAY;
  previousZ = correctedAZ;


  delay(50);
}