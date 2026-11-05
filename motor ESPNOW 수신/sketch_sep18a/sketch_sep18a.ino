// SOMA receiver: quiet output with adjustable low-input response.
// Compatible with the existing MotionPacket and sender MIX / DELAY commands.
// Upload this sketch to the RECEIVER. Sender firmware does not change.
// Receiver Serial Monitor: 115200 baud, Newline or Both NL & CR.
// Manual preview: 1..123 interrupts motion once, then returns to delayed motion.
// STOP or 0 disables all output. MOTION enables/resumes movement playback.
// Manual effects work offline, use ERM library 1, and are limited to 3 seconds.
// RTP settings, including MAX, apply to movement only.
// Movement commands: MAX 70, MIN 0, GAMMA 0.8, DEAD 0.01, STATUS, DEFAULTS, HELP.
// MIN applies only to active movement. Rest, button OFF and timeout output 0.
// FLOOR is also accepted as an alias for MIN.
// Settings are RAM-only; reset restores the defaults below.
// GPIOs match the original classic ESP32 sketch: SDA 21, SCL 22, button 25.
// Keep the existing motor voltage / driver setup; RTP is a command, not volts.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <Adafruit_DRV2605.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

Adafruit_DRV2605 drv;
const int BUTTON_PIN = 25;
const uint32_t TIMEOUT_MS = 1000;
const uint32_t DEBOUNCE_MS = 25;
const int BUFFER_SIZE = 256; // About 12.8 s at the existing ~20 Hz sender rate.

// Must match the sender exactly. The legacy intensity field remains on wire.
struct MotionPacket {
  uint32_t sequence;
  float movement;
  uint8_t intensity;
  uint8_t sensorMode;
  uint16_t delayMs;
};
static_assert(sizeof(MotionPacket) == 12, "Unexpected packet layout");

struct HistorySample {
  uint32_t receiveTime;
  float movement;
  uint8_t sensorMode;
};

struct PlaybackState {
  bool received;
  bool ready;
  uint32_t ageMs;
  uint16_t delayMs;
  uint8_t sensorMode;
  float movement;
};

// Explicit declaration for Arduino's automatic prototype generator.
PlaybackState readPlayback();

HistorySample history[BUFFER_SIZE];
int writeIndex = 0;
int historyCount = 0;
uint16_t currentDelayMs = 0;
portMUX_TYPE historyMux = portMUX_INITIALIZER_UNLOCKED;

bool motorEnabled = true;
bool buttonRaw = HIGH;
bool buttonStable = HIGH;
uint32_t buttonChangedAt = 0;

uint8_t maxOutput = 70;    // Upper RTP command; NOT a percent or motor voltage.
uint8_t floorOutput = 0;   // Optional minimum active command. No assumed motor floor.
float curveGamma = 0.8f;  // <1 gives low inputs more of the reduced output range.
float deadZone = 0.01f;   // Ignore very small input; rest always produces zero.
uint8_t lastOutput = 255; // Forces the first I2C update.

const uint32_t EFFECT_LIMIT_MS = 3000;
uint8_t activeEffect = 0; // Zero means the driver is in RTP mode.
uint32_t effectStartedAt = 0;
uint32_t effectCheckedAt = 0;

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value; 
}

const char *modeName(uint8_t mode) {
  switch (mode) {
    case 0: return "ACC";
    case 1: return "GYRO";
    case 2: return "MIX";
    default: return "UNKNOWN";
  }
}

// Called only by the main loop. Use the float from packet.movement, not the
// already truncated packet.intensity. Final DRV2605L RTP is still an integer.
uint8_t renderIntensity(float movement) {
  if (!isfinite(movement) || maxOutput == 0) return 0;
  float x = clamp01(movement);
  if (x <= deadZone) return 0;
  x = (x - deadZone) / (1.0f - deadZone);
  float shaped = powf(x, curveGamma);
  long output = lroundf(floorOutput + (maxOutput - floorOutput) * shaped);
  if (output < 0) output = 0;
  if (output > maxOutput) output = maxOutput;
  return static_cast<uint8_t>(output);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *, const uint8_t *data, int len) {
#else
void onDataReceived(const uint8_t *, const uint8_t *data, int len) {
#endif
  if (!data || len != static_cast<int>(sizeof(MotionPacket))) return;
  MotionPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (!isfinite(packet.movement) || packet.sensorMode > 2) return;

  // The Wi-Fi task and loop can run on different cores. Protect shared data.
  portENTER_CRITICAL(&historyMux);
  history[writeIndex] = {millis(), clamp01(packet.movement), packet.sensorMode};
  writeIndex = (writeIndex + 1) % BUFFER_SIZE;
  if (historyCount < BUFFER_SIZE) ++historyCount;
  currentDelayMs = packet.delayMs;
  portEXIT_CRITICAL(&historyMux);
}

PlaybackState readPlayback() {
  PlaybackState state = {};
  portENTER_CRITICAL(&historyMux);
  if (historyCount > 0) {
    // Capture time while locked so no sample can have a newer timestamp.
    uint32_t now = millis();
    int newest = (writeIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
    state.received = true;
    state.ageMs = now - history[newest].receiveTime;
    state.delayMs = currentDelayMs;
    state.sensorMode = history[newest].sensorMode;
    // Newest sample at or before the requested playback time. Unsigned ages
    // handle millis() rollover; valid samples are tracked by count, not time=0.
    for (int n = 0; n < historyCount; ++n) {
      int index = (newest + BUFFER_SIZE - n) % BUFFER_SIZE;
      if (now - history[index].receiveTime >= state.delayMs) {
        state.ready = true;
        state.movement = history[index].movement;
        state.sensorMode = history[index].sensorMode;
        break;
      }
    }
  }
  portEXIT_CRITICAL(&historyMux);
  return state;
}

void writeMotor(uint8_t output) {
  if (output != lastOutput) {
    drv.setRealtimeValue(output);
    lastOutput = output;
  }
}

void endEffect() {
  if (activeEffect == 0) return;
  drv.stop();
  drv.setRealtimeValue(0); // Clear the RTP register before re-entering RTP mode.
  drv.setMode(DRV2605_MODE_REALTIME);
  lastOutput = 0;
  activeEffect = 0;
}

void setMotorEnabled(bool enabled) {
  endEffect();
  writeMotor(0);
  motorEnabled = enabled;
  Serial.println(enabled ? "Motor ON: motion playback" : "Motor OFF: all output stopped");
}

void startEffect(uint8_t id) {
  if (!motorEnabled) {
    Serial.println("Motor OFF. Send MOTION or press the button to enable.");
    return;
  }
  if (activeEffect) drv.stop();
  else writeMotor(0);
  drv.setMode(DRV2605_MODE_INTTRIG);
  drv.setWaveform(0, id);
  drv.setWaveform(1, 0);
  drv.go();
  activeEffect = id;
  effectStartedAt = effectCheckedAt = millis();
  Serial.printf("Playing effect %u (Library 1)\n", static_cast<unsigned>(id));
}

void serviceEffect() {
  if (activeEffect == 0) return;
  uint32_t now = millis();
  if (now - effectCheckedAt < 10) return;
  effectCheckedAt = now;
  bool done = (drv.readRegister8(0x0C) & 1) == 0;
  if (!done && now - effectStartedAt < EFFECT_LIMIT_MS) return;
  endEffect();
  Serial.println(done ? "DONE -> MOTION" : "3-second limit -> MOTION");
}

void printSettings() {
  Serial.printf("MAX:%u | GAMMA:%.3f | MIN:%u | DEAD:%.4f | Motor:%s\n",
                static_cast<unsigned>(maxOutput), curveGamma,
                static_cast<unsigned>(floorOutput), deadZone,
                motorEnabled ? "ON" : "OFF");
}

void printHelp() {
  Serial.println("1..123: preview once (even offline), then return to motion. Max 3 seconds.");
  Serial.println("0 / STOP: mute all. MOTION: enable motion. Button: toggle ON/OFF.");
  Serial.println("MAX/MIN/GAMMA/DEAD apply to motion only.");
  Serial.println("Receiver: MAX 70 | MIN 0 | GAMMA 0.8 | DEAD 0.01");
  Serial.println("MAX: integer 0..127, must be >= MIN. MIN: integer 0..MAX.");
  Serial.println("MIN applies only while movement > DEAD; rest is silent.");
  Serial.println("FLOOR is an alias for MIN.");
  Serial.println("GAMMA: 0.3..3.0; below 1 emphasizes low inputs within MAX.");
  Serial.println("DEAD: 0..0.2; inputs <= DEAD are silent.");
  Serial.println("STATUS | DEFAULTS | HELP. Settings reset on power-off.");
  Serial.println("ACC/GYRO/MIX and DELAY commands still go to the SENDER.");
}

void handleCommand(char *line) {
  for (char *p = line; *p; ++p) *p = toupper(static_cast<unsigned char>(*p));
  char *save = nullptr;
  char *command = strtok_r(line, " \t", &save);
  if (!command) return;
  char *argument = strtok_r(nullptr, " \t", &save);
  char *extra = strtok_r(nullptr, " \t", &save);
  if (!argument) {
    if (strcmp(command, "STOP") == 0) { setMotorEnabled(false); return; }
    if (strcmp(command, "MOTION") == 0) { setMotorEnabled(true); return; }
    if (strspn(command, "0123456789") == strlen(command)) {
      unsigned long id = strtoul(command, nullptr, 10);
      if (id > 123) Serial.println("ERROR: enter 1..123, or 0 to stop.");
      else if (id == 0) setMotorEnabled(false);
      else startEffect(static_cast<uint8_t>(id));
      return;
    }
    if (strcmp(command, "STATUS") == 0) { printSettings(); return; }
    if (strcmp(command, "HELP") == 0) { printHelp(); return; }
    if (strcmp(command, "DEFAULTS") == 0) {
      maxOutput = 70; floorOutput = 0; curveGamma = 0.8f; deadZone = 0.01f;
      printSettings();
      return;
    }
    Serial.println("ERROR: unknown command or missing value. Type HELP.");
    return;
  }
  char *end = nullptr;
  float value = strtof(argument, &end);
  if (extra || end == argument || *end || !isfinite(value)) {
    Serial.println("ERROR: use one finite numeric value. Settings unchanged.");
    return;
  }
  bool valid = false;
  if (strcmp(command, "MAX") == 0) {
    valid = value >= floorOutput && value <= 127 && floorf(value) == value;
    if (valid) maxOutput = static_cast<uint8_t>(value);
  } else if (strcmp(command, "MIN") == 0 || strcmp(command, "FLOOR") == 0) {
    valid = value >= 0 && value <= maxOutput && floorf(value) == value;
    if (valid) floorOutput = static_cast<uint8_t>(value);
  } else if (strcmp(command, "GAMMA") == 0) {
    valid = value >= 0.3f && value <= 3.0f;
    if (valid) curveGamma = value;
  } else if (strcmp(command, "DEAD") == 0) {
    valid = value >= 0 && value <= 0.2f;
    if (valid) deadZone = value;
  }
  if (valid) printSettings();
  else Serial.println("ERROR: unknown command or out-of-range value. Settings unchanged.");
}

void readSerialCommands() {
  static char line[64];
  static size_t length = 0;
  static bool overflow = false;
  // Bound work per loop. Partial commands never block haptic output.
  for (int budget = 64; budget > 0 && Serial.available(); --budget) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("ERROR: command too long; discarded.");
      else { line[length] = '\0'; handleCommand(line); }
      length = 0; overflow = false;
    } else if (!overflow) {
      if (c == '\0' || length >= sizeof(line) - 1) overflow = true;
      else line[length++] = c;
    }
  }
}

void updateButton() {
  uint32_t now = millis();
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != buttonRaw) {
    buttonRaw = reading;
    buttonChangedAt = now;
  }
  if (now - buttonChangedAt >= DEBOUNCE_MS && buttonStable != buttonRaw) {
    buttonStable = buttonRaw;
    if (buttonStable == LOW) {
      setMotorEnabled(!motorEnabled);
    }
  }
}

void haltWithError(const char *message) {
  Serial.println(message);
  while (true) delay(100);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  if (!drv.begin()) haltWithError("DRV2605L NOT FOUND");
  drv.useERM();
  drv.selectLibrary(1);
  // Ensure signed RTP format: positive command range is 0..127.
  drv.writeRegister8(0x1D, drv.readRegister8(0x1D) & ~0x08);
  drv.setMode(DRV2605_MODE_REALTIME);
  writeMotor(0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonRaw = buttonStable = digitalRead(BUTTON_PIN);
  buttonChangedAt = millis();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    haltWithError("WIFI CHANNEL SET FAILED");
  if (esp_now_init() != ESP_OK) haltWithError("ESP-NOW INIT FAILED");
  if (esp_now_register_recv_cb(onDataReceived) != ESP_OK)
    haltWithError("ESP-NOW CALLBACK REGISTRATION FAILED");
  Serial.println("SOMA QUIET HAPTIC RECEIVER READY");
  printSettings();
  printHelp();
}

void loop() {
  updateButton();
  readSerialCommands();
  serviceEffect();
  if (activeEffect) { delay(5); return; } // ESP-NOW still records history during preview.
  PlaybackState state = readPlayback();
  bool timedOut = !state.received || state.ageMs > TIMEOUT_MS;
  uint8_t output = (!motorEnabled || timedOut || !state.ready)
                     ? 0 : renderIntensity(state.movement);
  writeMotor(output);

  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    const char *status = !motorEnabled ? "OFF" : timedOut ? "NO SIGNAL"
                        : !state.ready ? "BUFFERING" : "PLAY";
    Serial.printf("Mode:%s | Delay:%u ms | Input:%.4f | Output:%u | MIN:%u MAX:%u | %s\n",
                  state.received ? modeName(state.sensorMode) : "--",
                  static_cast<unsigned>(state.delayMs), state.movement,
                  static_cast<unsigned>(output), static_cast<unsigned>(floorOutput),
                  static_cast<unsigned>(maxOutput), status);
  }
  delay(5);
}
