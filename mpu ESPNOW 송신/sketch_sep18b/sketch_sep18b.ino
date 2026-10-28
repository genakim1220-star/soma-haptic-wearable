// SOMA transmitter: mode display and adjustable MIX ratio.
// Commands: ACC, GYRO, MIX, MIX 70 30, MIX 70, DELAY 0/3/5, CAL, STATUS, HELP.
// Ratios are percentages (ACC first). Reset restores MIX 50/50 and DELAY 0.
// Packet layout and nominal 50 ms loop delay match the original receiver.

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

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
float accelWeight = 0.5f;

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


const char *sensorModeName() {
  if (sensorMode == ACCELERATION) return "ACC";
  if (sensorMode == GYRO) return "GYRO";
  return "MIX";
}

void printMixRatio() {
  Serial.print("Mix ACC:");
  Serial.print(accelWeight * 100.0f, 1);
  Serial.print("% GYRO:");
  Serial.print((1.0f - accelWeight) * 100.0f, 1);
  Serial.print("%");
}

// One number means ACC percentage; two numbers must total 100.
// Invalid commands leave the current mode and ratio unchanged.
bool parseMixWeights(const char *text, float &newWeight) {
  while (isspace((unsigned char)*text)) ++text;
  char *end = nullptr;
  errno = 0;
  float acc = strtof(text, &end);
  if (end == text || errno == ERANGE || !isfinite(acc) ||
      acc < 0.0f || acc > 100.0f) return false;

  if (*end && !isspace((unsigned char)*end)) return false;
  text = end;
  while (isspace((unsigned char)*text)) ++text;
  if (*text) {
    errno = 0;
    float gyro = strtof(text, &end);
    if (end == text || errno == ERANGE || !isfinite(gyro) ||
        gyro < 0.0f || gyro > 100.0f) return false;
    while (isspace((unsigned char)*end)) ++end;
    if (*end || fabsf(acc + gyro - 100.0f) > 0.001f) return false;
  }
  newWeight = acc / 100.0f;
  return true;
}

float mixNormalized(float acc, float gyro, float weight) {
  return acc * weight + gyro * (1.0f - weight);
}

void printStatus() {

  Serial.println();
  Serial.println("========== STATUS ==========");

  Serial.print("Sensor Mode: ");
  Serial.println(sensorModeName());
  printMixRatio();
  Serial.println();

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
  Serial.println("MIX        -> MIX mode, keep current ratio");
  Serial.println("MIX 70 30  -> ACC 70%, GYRO 30%; select MIX");
  Serial.println("MIX 70     -> Same as MIX 70 30");
  Serial.println("Use 0..100 percentages; two values must total 100.");

  Serial.println("DELAY 0    -> Immediate");
  Serial.println("DELAY 3    -> 3 second delay");
  Serial.println("DELAY 5    -> 5 second delay");

  Serial.println("CAL        -> Sensor calibration");
  Serial.println("STATUS     -> Current settings");
  Serial.println("HELP       -> Show commands");

  Serial.println("==============================");
  Serial.println();
}


void processSerialCommand(String command) {

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


  else if (command.startsWith("MIX ") || command.startsWith("MIX\t")) {
    float newWeight = accelWeight;
    if (!parseMixWeights(command.c_str() + 3, newWeight)) {
      Serial.println("ERROR: Use MIX 70 30 (total 100) or MIX 70; range 0..100.");
      return;
    }
    accelWeight = newWeight;
    sensorMode = ACCEL_GYRO;
    printStatus();
  }

  else if (command == "MIX") {

    sensorMode = ACCEL_GYRO;

    Serial.println("Sensor Mode -> MIX");
    printMixRatio();
    Serial.println();
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



// Read a complete line without Serial.readStringUntil() blocking motion sampling.
// The sender still uses its original delay(50) at the end of loop().
void handleSerialCommand() {
  static char line[64];
  static size_t used = 0;
  static bool overflow = false;
  unsigned int budget = 64;
  while (budget-- > 0 && Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (overflow) {
        Serial.println("ERROR: Command too long; discarded.");
      } else if (used > 0) {
        line[used] = '\0';
        processSerialCommand(String(line));
      }
      used = 0;
      overflow = false;
    } else if (!overflow) {
      if (used + 1 < sizeof(line)) line[used++] = c;
      else overflow = true;
    }
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

  if (firstReading) {
    previousX = correctedAX;
    previousY = correctedAY;
    previousZ = correctedAZ;
    firstReading = false;
  }

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
      mixNormalized(
        accelerationNormalized,
        gyroNormalized,
        accelWeight
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

  Serial.print("Mode: ");
  Serial.print(sensorModeName());
  Serial.print(" | ");
  printMixRatio();
  Serial.print(" | ACC: ");
  Serial.print(accelerationNormalized, 3);
  Serial.print(" | GYRO: ");
  Serial.print(gyroNormalized, 3);
  Serial.print(" | Movement: ");
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