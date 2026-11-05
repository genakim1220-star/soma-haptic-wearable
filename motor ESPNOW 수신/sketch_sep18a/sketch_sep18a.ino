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
void updateMotionPattern(const PlaybackState &state, bool valid);

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
uint8_t motionPattern = 0; // 0 = continuous RTP
uint8_t shapeId = 0;
bool shapeRunning = false;
uint32_t shapeStart = 0;
bool patternArmed = false;
bool motionTriggered = false;
float onset = 0.10f;
uint16_t observedDelay = 0;
uint8_t observedMode = 255;
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
  uint32_t now = millis();
  int newest = (writeIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
  if (historyCount && now - history[newest].receiveTime > TIMEOUT_MS) historyCount = 0;
  history[writeIndex] = {now, clamp01(packet.movement), packet.sensorMode};
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

enum ShapeKind : uint8_t { Hit, Bump, Flutter, Buzz, DownSmooth, DownSharp, UpSmooth, UpSharp, Hold };
struct ShapeProfile {
  ShapeKind kind;
  uint16_t durationMs; // Duration of one pulse
  uint8_t count;
  uint16_t gapMs;      // Between pulses of one motif
  uint16_t restMs;     // Before repeating the motif
  uint8_t peakPercent;
};

const ShapeProfile shapes[] = {
  {Hit, 65, 1, 0, 140, 100}, // 1: strong click
  {Hit, 65, 1, 0, 140, 60}, // 2: strong click
  {Hit, 65, 1, 0, 140, 30}, // 3: strong click
  {Hit, 38, 1, 0, 140, 100}, // 4: sharp click
  {Hit, 38, 1, 0, 140, 60}, // 5: sharp click
  {Hit, 38, 1, 0, 140, 30}, // 6: sharp click
  {Bump, 130, 1, 0, 140, 100}, // 7: soft bump
  {Bump, 130, 1, 0, 140, 60}, // 8: soft bump
  {Bump, 130, 1, 0, 140, 30}, // 9: soft bump
  {Hit, 60, 2, 100, 140, 100}, // 10: double click
  {Hit, 60, 2, 100, 140, 60}, // 11: double click
  {Hit, 60, 3, 95, 140, 100}, // 12: triple click
  {Flutter, 230, 1, 0, 140, 60}, // 13: soft fuzz
  {Buzz, 285, 1, 0, 140, 100}, // 14: strong buzz
  {Buzz, 750, 1, 0, 140, 100}, // 15: alert
  {Buzz, 1000, 1, 0, 140, 100}, // 16: alert
  {Hit, 80, 1, 0, 140, 100}, // 17: strong click variants
  {Hit, 80, 1, 0, 140, 80}, // 18: strong click variants
  {Hit, 80, 1, 0, 140, 60}, // 19: strong click variants
  {Hit, 80, 1, 0, 140, 30}, // 20: strong click variants
  {Bump, 84, 1, 0, 140, 100}, // 21: medium click
  {Bump, 84, 1, 0, 140, 80}, // 22: medium click
  {Bump, 84, 1, 0, 140, 60}, // 23: medium click
  {Hit, 30, 1, 0, 140, 100}, // 24: sharp tick
  {Hit, 30, 1, 0, 140, 80}, // 25: sharp tick
  {Hit, 30, 1, 0, 140, 60}, // 26: sharp tick
  {Hit, 65, 2, 55, 140, 100}, // 27: short double strong
  {Hit, 65, 2, 55, 140, 80}, // 28: short double strong
  {Hit, 65, 2, 55, 140, 60}, // 29: short double strong
  {Hit, 65, 2, 55, 140, 30}, // 30: short double strong
  {Bump, 58, 2, 52, 140, 100}, // 31: short double medium
  {Bump, 58, 2, 52, 140, 80}, // 32: short double medium
  {Bump, 58, 2, 52, 140, 60}, // 33: short double medium
  {Hit, 30, 2, 50, 140, 100}, // 34: short double tick
  {Hit, 30, 2, 50, 140, 80}, // 35: short double tick
  {Hit, 30, 2, 50, 140, 60}, // 36: short double tick
  {Hit, 76, 2, 155, 140, 100}, // 37: long double strong
  {Hit, 76, 2, 155, 140, 80}, // 38: long double strong
  {Hit, 76, 2, 155, 140, 60}, // 39: long double strong
  {Hit, 76, 2, 155, 140, 30}, // 40: long double strong
  {Bump, 72, 2, 150, 140, 100}, // 41: long double medium
  {Bump, 72, 2, 150, 140, 80}, // 42: long double medium
  {Bump, 72, 2, 150, 140, 60}, // 43: long double medium
  {Hit, 32, 2, 150, 140, 100}, // 44: long double tick
  {Hit, 32, 2, 150, 140, 80}, // 45: long double tick
  {Hit, 32, 2, 150, 140, 60}, // 46: long double tick
  {Buzz, 220, 1, 0, 140, 100}, // 47: buzz
  {Buzz, 220, 1, 0, 140, 80}, // 48: buzz
  {Buzz, 220, 1, 0, 140, 60}, // 49: buzz
  {Buzz, 220, 1, 0, 140, 40}, // 50: buzz
  {Buzz, 220, 1, 0, 140, 20}, // 51: buzz
  {Hit, 125, 3, 90, 140, 100}, // 52: strong pulses
  {Hit, 125, 3, 90, 140, 60}, // 53: strong pulses
  {Bump, 100, 3, 75, 140, 100}, // 54: medium pulses
  {Bump, 100, 3, 75, 140, 60}, // 55: medium pulses
  {Hit, 45, 3, 80, 140, 100}, // 56: sharp pulses
  {Hit, 45, 3, 80, 140, 60}, // 57: sharp pulses
  {Hit, 95, 1, 0, 140, 100}, // 58: transition click
  {Hit, 95, 1, 0, 140, 80}, // 59: transition click
  {Hit, 95, 1, 0, 140, 60}, // 60: transition click
  {Hit, 95, 1, 0, 140, 40}, // 61: transition click
  {Hit, 95, 1, 0, 140, 20}, // 62: transition click
  {Hit, 95, 1, 0, 140, 10}, // 63: transition click
  {Bump, 270, 1, 0, 140, 100}, // 64: transition hum
  {Bump, 270, 1, 0, 140, 80}, // 65: transition hum
  {Bump, 270, 1, 0, 140, 60}, // 66: transition hum
  {Bump, 270, 1, 0, 140, 40}, // 67: transition hum
  {Bump, 270, 1, 0, 140, 20}, // 68: transition hum
  {Bump, 270, 1, 0, 140, 10}, // 69: transition hum
  {DownSmooth, 800, 1, 0, 140, 100}, // 70: Down Smooth ramp
  {DownSmooth, 680, 1, 0, 140, 100}, // 71: Down Smooth ramp
  {DownSmooth, 480, 1, 0, 140, 100}, // 72: Down Smooth ramp
  {DownSmooth, 408, 1, 0, 140, 100}, // 73: Down Smooth ramp
  {DownSmooth, 240, 1, 0, 140, 100}, // 74: Down Smooth ramp
  {DownSmooth, 204, 1, 0, 140, 100}, // 75: Down Smooth ramp
  {DownSharp, 800, 1, 0, 140, 100}, // 76: Down Sharp ramp
  {DownSharp, 680, 1, 0, 140, 100}, // 77: Down Sharp ramp
  {DownSharp, 480, 1, 0, 140, 100}, // 78: Down Sharp ramp
  {DownSharp, 408, 1, 0, 140, 100}, // 79: Down Sharp ramp
  {DownSharp, 240, 1, 0, 140, 100}, // 80: Down Sharp ramp
  {DownSharp, 204, 1, 0, 140, 100}, // 81: Down Sharp ramp
  {UpSmooth, 800, 1, 0, 140, 100}, // 82: Up Smooth ramp
  {UpSmooth, 680, 1, 0, 140, 100}, // 83: Up Smooth ramp
  {UpSmooth, 480, 1, 0, 140, 100}, // 84: Up Smooth ramp
  {UpSmooth, 408, 1, 0, 140, 100}, // 85: Up Smooth ramp
  {UpSmooth, 240, 1, 0, 140, 100}, // 86: Up Smooth ramp
  {UpSmooth, 204, 1, 0, 140, 100}, // 87: Up Smooth ramp
  {UpSharp, 800, 1, 0, 140, 100}, // 88: Up Sharp ramp
  {UpSharp, 680, 1, 0, 140, 100}, // 89: Up Sharp ramp
  {UpSharp, 480, 1, 0, 140, 100}, // 90: Up Sharp ramp
  {UpSharp, 408, 1, 0, 140, 100}, // 91: Up Sharp ramp
  {UpSharp, 240, 1, 0, 140, 100}, // 92: Up Sharp ramp
  {UpSharp, 204, 1, 0, 140, 100}, // 93: Up Sharp ramp
  {DownSmooth, 800, 1, 0, 140, 50}, // 94: Down Smooth ramp
  {DownSmooth, 680, 1, 0, 140, 50}, // 95: Down Smooth ramp
  {DownSmooth, 480, 1, 0, 140, 50}, // 96: Down Smooth ramp
  {DownSmooth, 408, 1, 0, 140, 50}, // 97: Down Smooth ramp
  {DownSmooth, 240, 1, 0, 140, 50}, // 98: Down Smooth ramp
  {DownSmooth, 204, 1, 0, 140, 50}, // 99: Down Smooth ramp
  {DownSharp, 800, 1, 0, 140, 50}, // 100: Down Sharp ramp
  {DownSharp, 680, 1, 0, 140, 50}, // 101: Down Sharp ramp
  {DownSharp, 480, 1, 0, 140, 50}, // 102: Down Sharp ramp
  {DownSharp, 408, 1, 0, 140, 50}, // 103: Down Sharp ramp
  {DownSharp, 240, 1, 0, 140, 50}, // 104: Down Sharp ramp
  {DownSharp, 204, 1, 0, 140, 50}, // 105: Down Sharp ramp
  {UpSmooth, 800, 1, 0, 140, 50}, // 106: Up Smooth ramp
  {UpSmooth, 680, 1, 0, 140, 50}, // 107: Up Smooth ramp
  {UpSmooth, 480, 1, 0, 140, 50}, // 108: Up Smooth ramp
  {UpSmooth, 408, 1, 0, 140, 50}, // 109: Up Smooth ramp
  {UpSmooth, 240, 1, 0, 140, 50}, // 110: Up Smooth ramp
  {UpSmooth, 204, 1, 0, 140, 50}, // 111: Up Smooth ramp
  {UpSharp, 800, 1, 0, 140, 50}, // 112: Up Sharp ramp
  {UpSharp, 680, 1, 0, 140, 50}, // 113: Up Sharp ramp
  {UpSharp, 480, 1, 0, 140, 50}, // 114: Up Sharp ramp
  {UpSharp, 408, 1, 0, 140, 50}, // 115: Up Sharp ramp
  {UpSharp, 240, 1, 0, 140, 50}, // 116: Up Sharp ramp
  {UpSharp, 204, 1, 0, 140, 50}, // 117: Up Sharp ramp
  {Hold, 1000, 1, 0, 0, 100}, // 118: long buzz
  {Hold, 1000, 1, 0, 0, 50}, // 119: smooth hum
  {Hold, 1000, 1, 0, 0, 40}, // 120: smooth hum
  {Hold, 1000, 1, 0, 0, 30}, // 121: smooth hum
  {Hold, 1000, 1, 0, 0, 20}, // 122: smooth hum
  {Hold, 1000, 1, 0, 0, 10}, // 123: smooth hum
};
static_assert(sizeof(shapes) / sizeof(shapes[0]) == 123, "Missing shape");

float shapeEnvelope(uint8_t id, uint32_t elapsed) {
  const ShapeProfile &p = shapes[id - 1];
  const uint32_t step = p.durationMs + p.gapMs;
  const uint32_t motif = step * p.count - p.gapMs;
  const uint32_t phase = elapsed % (motif + p.restMs);
  if (phase >= motif || phase % step >= p.durationMs) return 0;
  const float t = static_cast<float>(phase % step) / p.durationMs;
  float level = 0;
  switch (p.kind) {
    case Hit:        level = (1 - t) * (1 - t); break;
    case Bump:       level = 4 * t * (1 - t); break;
    case Flutter:    level = 0.65f + 0.35f * sinf(37.7f * t); break;
    case Buzz:       level = fminf(1, fminf(t, 1 - t) * 18); break;
    case DownSmooth: level = 1 - t * t * (3 - 2 * t); break;
    case UpSmooth:   level = t * t * (3 - 2 * t); break;
    case DownSharp:  level = (1 - t) * (1 - t); break;
    case UpSharp:    level = t * t; break;
    case Hold:       level = 1; break;
  }
  return clamp01(level) * (p.peakPercent / 100.0f);
}

uint8_t shapeOutput(uint8_t motionOutput) {
  if (!motionOutput) { shapeRunning = false; return 0; }
  uint32_t now = millis();
  if (!shapeRunning) { shapeStart = now; shapeRunning = true; }
  return static_cast<uint8_t>(lroundf(motionOutput * shapeEnvelope(shapeId, now - shapeStart)));
}

void endEffect() {
  if (activeEffect == 0) return;
  drv.stop();
  drv.setRealtimeValue(0); // Clear the RTP register before re-entering RTP mode.
  drv.setMode(DRV2605_MODE_REALTIME);
  lastOutput = 0;
  activeEffect = 0;
  motionTriggered = false;
}

void setMotorEnabled(bool enabled) {
  endEffect();
  writeMotor(0);
  patternArmed = false;
  shapeRunning = false;
  motorEnabled = enabled;
  Serial.println(enabled ? "Motor ON: motion playback" : "Motor OFF: all output stopped");
}

void startEffect(uint8_t id) {
  if (!motorEnabled) {
    Serial.println("Motor OFF. Send MOTION or press the button to enable.");
    return;
  }
  shapeRunning = false;
  if (activeEffect) drv.stop();
  else writeMotor(0);
  drv.setMode(DRV2605_MODE_INTTRIG);
  drv.setWaveform(0, id);
  drv.setWaveform(1, 0);
  motionTriggered = false;
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
  Serial.println(done ? "DONE" : "Stopped at 3-second limit");
}

// Track threshold crossings even during a preview. Never interrupt a playing effect.
void updateMotionPattern(const PlaybackState &state, bool valid) {
  if (!valid || observedDelay != state.delayMs || observedMode != state.sensorMode) {
    patternArmed = false;
    if (motionTriggered) endEffect();
  }
  observedDelay = state.delayMs;
  observedMode = state.sensorMode;
  if (!valid) return;
  if (state.movement <= onset * 0.5f) patternArmed = true;
  if (patternArmed && state.movement >= onset) {
    patternArmed = false;
    if (!activeEffect) {
      startEffect(motionPattern);
      motionTriggered = true;
    }
  }
}

void printSettings() {
  Serial.printf("SHAPE:%u (0=off, RTP approximation) | ", static_cast<unsigned>(shapeId));
  Serial.printf("Pattern:%u (0=RTP) | ONSET:%.3f\n",
                static_cast<unsigned>(motionPattern), onset);
  Serial.printf("MAX:%u | GAMMA:%.3f | MIN:%u | DEAD:%.4f | Motor:%s\n",
                static_cast<unsigned>(maxOutput), curveGamma,
                static_cast<unsigned>(floorOutput), deadZone,
                motorEnabled ? "ON" : "OFF");
}

void printHelp() {
  Serial.println("SHAPE 1..123: software approximation scaled by delayed motion; repeats while moving.");
  Serial.println("PATTERN 1..123: play on movement onset. Hold still first. MOTION: continuous RTP.");
  Serial.println("ONSET 0.10: trigger threshold (0.001..1). Release at half the threshold.");
  Serial.println("1..123: preview once (even offline), then return to motion. Max 3 seconds.");
  Serial.println("0 / STOP: mute all. MOTION: enable motion. Button: toggle ON/OFF.");
  Serial.println("MAX/MIN/GAMMA/DEAD apply to MOTION and SHAPE.");
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
    if (strcmp(command, "MOTION") == 0) { shapeId = 0; motionPattern = 0; setMotorEnabled(true); return; }
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
      onset = 0.10f; patternArmed = false;
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
  if (strcmp(command, "PATTERN") == 0 || strcmp(command, "SHAPE") == 0) {
    valid = strspn(argument, "0123456789") == strlen(argument) && value >= 1 && value <= 123;
    if (valid) {
      endEffect();
      writeMotor(0);
      bool useShape = strcmp(command, "SHAPE") == 0;
      shapeId = useShape ? static_cast<uint8_t>(value) : 0;
      motionPattern = useShape ? 0 : static_cast<uint8_t>(value);
      shapeRunning = false;
      patternArmed = false;
    }
  } else if (strcmp(command, "ONSET") == 0) {
    valid = value >= 0.001f && value <= 1.0f;
    if (valid) { onset = value; patternArmed = false; }
  } else if (strcmp(command, "MAX") == 0) {
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
  Serial.println("SOMA SHAPE RECEIVER READY");
  printSettings();
  printHelp();
}

void loop() {
  updateButton();
  readSerialCommands();
  serviceEffect();
  PlaybackState state = readPlayback();
  bool timedOut = !state.received || state.ageMs > TIMEOUT_MS;
  bool valid = motorEnabled && !timedOut && state.ready;
  if (motionPattern) updateMotionPattern(state, valid);
  uint8_t output = (!motionPattern && valid) ? renderIntensity(state.movement) : 0;
  if (!activeEffect) {
    if (shapeId) output = shapeOutput(output);
    writeMotor(output);
  }

  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    const char *status = !motorEnabled ? "OFF" : timedOut ? "NO SIGNAL"
                        : !state.ready ? "BUFFERING" : activeEffect ? "ROM" : motionPattern ? "PATTERN" : shapeId ? "SHAPE" : "PLAY";
    Serial.printf("Mode:%s | Delay:%u ms | Input:%.4f | RTP:%u | MIN:%u MAX:%u | %s\n",
                  state.received ? modeName(state.sensorMode) : "--",
                  static_cast<unsigned>(state.delayMs), state.movement,
                  static_cast<unsigned>(output), static_cast<unsigned>(floorOutput),
                  static_cast<unsigned>(maxOutput), status);
  }
  delay(5);
}
