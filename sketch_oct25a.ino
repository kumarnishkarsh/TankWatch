/**
 * ============================================================================
 * TankWatch Edge Controller Firmware
 * Platform: Arduino Uno (ATmega328P)
 * Baud: 115200 | Contract: Line-based CSV (D, E, K frames)
 * 
 * Hardware Mapping:
 *   - HC-SR04 TRIG  : Pin D9
 *   - HC-SR04 ECHO  : Pin D10
 *   - Relay IN      : Pin D7 (Active-LOW, boot-safe initialized)
 *   - Active Buzzer : Pin D6 (Non-blocking PWM/Tone state sequencer)
 *   - Reset Button  : Pin D2 (INPUT_PULLUP, debounced hardware interrupt/poll)
 *   - 16x2 LCD      : I2C A4 (SDA), A5 (SCL), Address 0x27
 * ============================================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================================
// CONFIGURATION & TUNABLES (Bench Demo 10-20 L Bucket Defaults)
// ============================================================================
#define SIM_MODE                 0       // 1 = Fake distance ramp (bench test without sensor), 0 = Real HC-SR04
#define RELAY_ACTIVE_LOW         1       // 1 = Active-LOW relay module (LOW = ON, HIGH = OFF)

// Physical Geometry & Levels (in mm and percentage)
#define TANK_HEIGHT_MM           350     // Total depth of the tank/bucket (top to bottom)
#define SENSOR_OFFSET_MM         60      // Distance from HC-SR04 face to the 100% water mark
#define LOW_PCT                  25      // Pump turns ON when level falls below this %
#define HIGH_PCT                 85      // Pump turns OFF when level reaches or exceeds this %

// Timing Failsafes (in seconds and milliseconds)
#define MAX_RUNTIME_S            45      // Failsafe cutoff: max continuous pumping time (Demo: 45s, Real: 900s)
#define MIN_OFF_TIME_MS          5000UL  // Anti-chatter: minimum pump off-time before re-triggering
#define SENSOR_FAULT_THRESHOLD   10      // Number of consecutive invalid pings before SENSOR_FAULT (10 * 200ms = 2.0s)
#define PING_INTERVAL_MS         200UL   // HC-SR04 sampling interval
#define TELEMETRY_INTERVAL_MS    1000UL  // Serial 'D' frame output interval
#define LCD_REFRESH_INTERVAL_MS  500UL   // LCD update rate (prevents I2C bus congestion and backlight flicker)

// Hardware Pin Assignments
#define PIN_RESET_BTN            2
#define PIN_BUZZER               6
#define PIN_RELAY                7
#define PIN_TRIG                 9
#define PIN_ECHO                 10
#define LCD_I2C_ADDR             0x27

// Median Filter Window Size
#define MEDIAN_WINDOW            5

// Valid Ultrasonic Distance Bounds (in mm)
#define MIN_VALID_DIST_MM        25      // Physical dead-zone of HC-SR04 (~2-3 cm)
#define MAX_VALID_DIST_MM        800     // Anything beyond tank bottom + margin is treated as invalid reflection

// Relay Helper Macros
#if RELAY_ACTIVE_LOW
  #define RELAY_STATE_ON         LOW
  #define RELAY_STATE_OFF        HIGH
#else
  #define RELAY_STATE_ON         HIGH
  #define RELAY_STATE_OFF        LOW
#endif

// ============================================================================
// STATE MACHINE DEFINITIONS
// ============================================================================
enum SystemState {
  STATE_IDLE,
  STATE_FILLING,
  STATE_TRIP
};

enum BuzzerPattern {
  BUZZ_OFF,
  BUZZ_SHORT,       // 150ms beep (Pump start)
  BUZZ_TRIPLE,      // 3 rapid beeps (Sensor fault)
  BUZZ_TRIP_ALARM   // Continuous pulsing (Failsafe tripped)
};

// ============================================================================
// GLOBAL SYSTEM STATE & VARIABLES
// ============================================================================
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);

SystemState currentState = STATE_IDLE;
bool pumpRelayActive = false;
bool forcePumpOff = false;
bool sensorFaultActive = false;

// Distance & Level Measurements
long lastRawDistMm = -1;
long medianDistMm = -1;
int calculatedLevelPct = 0;

// Median Filter Circular Buffer
long distHistory[MEDIAN_WINDOW];
uint8_t historyIndex = 0;
uint8_t validSamplesCount = 0;
uint16_t consecutiveInvalidPings = 0;

// Non-blocking Timestamps (all unsigned long to handle millis rollover)
unsigned long lastPingTimeMs = 0;
unsigned long lastTelemetryTimeMs = 0;
unsigned long lastLcdUpdateTimeMs = 0;
unsigned long pumpStartTimeMs = 0;
unsigned long lastPumpOffTimeMs = 0;

// Debouncing for D2 Reset Button
int lastButtonReading = HIGH;
int debouncedButtonState = HIGH;
unsigned long lastDebounceTimeMs = 0;
const unsigned long DEBOUNCE_DELAY_MS = 50;

// Buzzer State Machine Variables
BuzzerPattern currentBuzzerPattern = BUZZ_OFF;
unsigned long buzzerStepStartMs = 0;
uint8_t buzzerStep = 0;

// Serial Command Buffer
char serialBuffer[32];
uint8_t serialBufferIdx = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void processSensors();
void executeControlLogic();
void emitTelemetry();
void updateLcd();
void updateBuzzer();
void parseSerialCommands();
void checkHardwareResetButton();
void setPump(bool enable, const char* reasonEvent);
void setBuzzerPattern(BuzzerPattern pattern);
long readUltrasonicRaw();
void insertMedianSample(long sample);
long computeMedian();
int computeLevelPercent(long distMm);

// ============================================================================
// ARDUINO SETUP
// ============================================================================
void setup() {
  // CRITICAL BOOT SAFETY: Pre-load the output register with RELAY_STATE_OFF
  // BEFORE setting DDR to OUTPUT. This guarantees zero contact bounce/pulsing on reset.
  digitalWrite(PIN_RELAY, RELAY_STATE_OFF);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_STATE_OFF);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // Initialize Serial Interface
  Serial.begin(115200);
  while (!Serial && millis() < 1000); // Short grace period for USB enumeration

  // Initialize I2C LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("TankWatch v1.0"));
  lcd.setCursor(0, 1);
  lcd.print(F("Init Edge Ctrl.."));

  // Pre-seed median filter buffer
  for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) {
    distHistory[i] = -1;
  }

  // Prevent immediate pump restart upon reboot
  lastPumpOffTimeMs = millis();

  delay(600); // Brief pause to show boot screen
  lcd.clear();
}

// ============================================================================
// MAIN REAL-TIME LOOP (Zero delay calls)
// ============================================================================
void loop() {
  unsigned long now = millis();

  // 1. Ultrasonic Ping & Filtering (Every 200 ms)
  if (now - lastPingTimeMs >= PING_INTERVAL_MS) {
    lastPingTimeMs = now;
    processSensors();
  }

  // 2. State Machine & Pump Safety Logic
  executeControlLogic();

  // 3. Hardware Button Poll (D2 Reset)
  checkHardwareResetButton();

  // 4. Inbound Serial Command Parser (K frames)
  parseSerialCommands();

  // 5. Non-blocking Buzzer Sound Pattern Sequencer
  updateBuzzer();

  // 6. Outbound Telemetry (Every 1000 ms)
  if (now - lastTelemetryTimeMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTimeMs = now;
    emitTelemetry();
  }

  // 7. LCD Display Refresh (Every 500 ms)
  if (now - lastLcdUpdateTimeMs >= LCD_REFRESH_INTERVAL_MS) {
    lastLcdUpdateTimeMs = now;
    updateLcd();
  }
}

// ============================================================================
// SENSOR PROCESSING & MEDIAN FILTERING
// ============================================================================
void processSensors() {
#if SIM_MODE
  // Bench Simulator: Emulate water filling and draining
  static long simDist = TANK_HEIGHT_MM + SENSOR_OFFSET_MM - 20;
  if (pumpRelayActive) {
    simDist -= 6; // Tank fills: distance decreases
    if (simDist < SENSOR_OFFSET_MM + 10) simDist = SENSOR_OFFSET_MM + 10;
  } else {
    simDist += 2; // Slow usage/drainage
    if (simDist > TANK_HEIGHT_MM + SENSOR_OFFSET_MM) {
      simDist = TANK_HEIGHT_MM + SENSOR_OFFSET_MM;
    }
  }
  lastRawDistMm = simDist;
#else
  lastRawDistMm = readUltrasonicRaw();
#endif

  if (lastRawDistMm < 0) {
    consecutiveInvalidPings++;
    if (consecutiveInvalidPings >= SENSOR_FAULT_THRESHOLD) {
      if (!sensorFaultActive) {
        sensorFaultActive = true;
        Serial.print(F("E,"));
        Serial.print(millis());
        Serial.println(F(",SENSOR_FAULT"));
        setBuzzerPattern(BUZZ_TRIPLE);
        
        // Critical: Safe-Off action
        if (currentState == STATE_FILLING) {
          setPump(false, NULL);
          currentState = STATE_IDLE;
        }
      }
    }
  } else {
    // Valid sample received
    consecutiveInvalidPings = 0;
    if (sensorFaultActive) {
      sensorFaultActive = false; // Sensor recovered
    }
    insertMedianSample(lastRawDistMm);
    medianDistMm = computeMedian();
    calculatedLevelPct = computeLevelPercent(medianDistMm);
  }
}

long readUltrasonicRaw() {
  // Clear trigger pin
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  
  // 10 microsecond HIGH pulse
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Timeout = 26000 us (~4.4 meters range, avoids long blocking when no echo)
  unsigned long durationUs = pulseIn(PIN_ECHO, HIGH, 26000UL);

  if (durationUs == 0) {
    return -1; // Echo timed out
  }

  // Speed of sound: 343 m/s -> dist = (duration * 0.343 mm/us) / 2 = duration * 343 / 2000
  long distanceMm = (long)((durationUs * 343UL) / 2000UL);

  if (distanceMm < MIN_VALID_DIST_MM || distanceMm > MAX_VALID_DIST_MM) {
    return -1; // Out of realistic bounds
  }

  return distanceMm;
}

void insertMedianSample(long sample) {
  distHistory[historyIndex] = sample;
  historyIndex = (historyIndex + 1) % MEDIAN_WINDOW;
  if (validSamplesCount < MEDIAN_WINDOW) {
    validSamplesCount++;
  }
}

long computeMedian() {
  if (validSamplesCount == 0) return -1;

  long sorted[MEDIAN_WINDOW];
  uint8_t n = validSamplesCount;
  for (uint8_t i = 0; i < n; i++) {
    sorted[i] = distHistory[i];
  }

  // Simple insertion sort (N=5, extremely light on CPU)
  for (uint8_t i = 1; i < n; i++) {
    long key = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  return sorted[n / 2];
}

int computeLevelPercent(long distMm) {
  if (distMm < 0) return 0;

  // level_pct = 100 * (tank_height_mm - (dist_mm - sensor_offset_mm)) / tank_height_mm
  long effectiveWaterHeight = (long)TANK_HEIGHT_MM - (distMm - (long)SENSOR_OFFSET_MM);
  long pct = (effectiveWaterHeight * 100L) / (long)TANK_HEIGHT_MM;

  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)pct;
}

// ============================================================================
// FINITE STATE MACHINE (FSM)
// ============================================================================
void executeControlLogic() {
  unsigned long now = millis();

  switch (currentState) {
    case STATE_IDLE: {
      // Check if low water triggered and safety constraints allow pumping
      if (!sensorFaultActive && !forcePumpOff && validSamplesCount >= 3) {
        if (calculatedLevelPct < LOW_PCT) {
          // Anti-chatter: check minimum pump off duration
          if (now - lastPumpOffTimeMs >= MIN_OFF_TIME_MS) {
            currentState = STATE_FILLING;
            pumpStartTimeMs = now;
            setPump(true, "PUMP_ON");
            setBuzzerPattern(BUZZ_SHORT);
          }
        }
      }
      break;
    }

    case STATE_FILLING: {
      // 1. High Level Cutoff Check
      if (!sensorFaultActive && calculatedLevelPct >= HIGH_PCT) {
        setPump(false, "PUMP_OFF_HIGH");
        currentState = STATE_IDLE;
        break;
      }

      // 2. Failsafe Maximum Runtime Check
      unsigned long continuousRuntimeMs = now - pumpStartTimeMs;
      if (continuousRuntimeMs >= ((unsigned long)MAX_RUNTIME_S * 1000UL)) {
        setPump(false, "TRIP_MAXRUN");
        currentState = STATE_TRIP;
        setBuzzerPattern(BUZZ_TRIP_ALARM);
        break;
      }

      // 3. Sensor Fault while pumping
      if (sensorFaultActive) {
        setPump(false, NULL);
        currentState = STATE_IDLE;
        break;
      }
      break;
    }

    case STATE_TRIP: {
      // Latched state. Pump stays OFF until manual reset or serial command.
      if (pumpRelayActive) {
        setPump(false, NULL);
      }
      break;
    }
  }
}

// ============================================================================
// HARDWARE DRIVERS & ACTUATORS
// ============================================================================
void setPump(bool enable, const char* reasonEvent) {
  if (enable) {
    digitalWrite(PIN_RELAY, RELAY_STATE_ON);
    pumpRelayActive = true;
  } else {
    digitalWrite(PIN_RELAY, RELAY_STATE_OFF);
    pumpRelayActive = false;
    lastPumpOffTimeMs = millis();
  }

  if (reasonEvent != NULL) {
    Serial.print(F("E,"));
    Serial.print(millis());
    Serial.print(F(","));
    Serial.println(reasonEvent);
  }
}

void checkHardwareResetButton() {
  int reading = digitalRead(PIN_RESET_BTN);

  if (reading != lastButtonReading) {
    lastDebounceTimeMs = millis();
  }

  if ((millis() - lastDebounceTimeMs) > DEBOUNCE_DELAY_MS) {
    if (reading != debouncedButtonState) {
      debouncedButtonState = reading;
      // Button is active-LOW (INPUT_PULLUP, pressed connects to GND)
      if (debouncedButtonState == LOW) {
        if (currentState == STATE_TRIP) {
          currentState = STATE_IDLE;
          setBuzzerPattern(BUZZ_SHORT);
          lastPumpOffTimeMs = millis(); // Enforce anti-chatter pause before restarting
          Serial.print(F("E,"));
          Serial.print(millis());
          Serial.println(F(",RECOVERED"));
        }
      }
    }
  }
  lastButtonReading = reading;
}

// ============================================================================
// BUZZER PATTERN SEQUENCER (Strictly Non-Blocking)
// ============================================================================
void setBuzzerPattern(BuzzerPattern pattern) {
  currentBuzzerPattern = pattern;
  buzzerStep = 0;
  buzzerStepStartMs = millis();
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, LOW);
}

void updateBuzzer() {
  if (currentBuzzerPattern == BUZZ_OFF) {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, LOW);
    return;
  }

  unsigned long elapsed = millis() - buzzerStepStartMs;

  switch (currentBuzzerPattern) {
    case BUZZ_SHORT:
      if (buzzerStep == 0) {
        tone(PIN_BUZZER, 2400);
        if (elapsed >= 150) {
          noTone(PIN_BUZZER);
          currentBuzzerPattern = BUZZ_OFF;
        }
      }
      break;

    case BUZZ_TRIPLE:
      // Beep 80ms, Off 60ms, Beep 80ms, Off 60ms, Beep 80ms
      if (buzzerStep == 0) {
        tone(PIN_BUZZER, 3000);
        if (elapsed >= 80) { noTone(PIN_BUZZER); buzzerStep++; buzzerStepStartMs = millis(); }
      } else if (buzzerStep == 1) {
        if (elapsed >= 60) { buzzerStep++; buzzerStepStartMs = millis(); }
      } else if (buzzerStep == 2) {
        tone(PIN_BUZZER, 3000);
        if (elapsed >= 80) { noTone(PIN_BUZZER); buzzerStep++; buzzerStepStartMs = millis(); }
      } else if (buzzerStep == 3) {
        if (elapsed >= 60) { buzzerStep++; buzzerStepStartMs = millis(); }
      } else if (buzzerStep == 4) {
        tone(PIN_BUZZER, 3000);
        if (elapsed >= 80) { noTone(PIN_BUZZER); currentBuzzerPattern = BUZZ_OFF; }
      }
      break;

    case BUZZ_TRIP_ALARM:
      // Repeating alarm: 500ms ON, 300ms OFF
      if (buzzerStep == 0) {
        tone(PIN_BUZZER, 1800);
        if (elapsed >= 500) {
          noTone(PIN_BUZZER);
          buzzerStep = 1;
          buzzerStepStartMs = millis();
        }
      } else if (buzzerStep == 1) {
        if (elapsed >= 300) {
          buzzerStep = 0;
          buzzerStepStartMs = millis();
        }
      }
      break;

    default:
      currentBuzzerPattern = BUZZ_OFF;
      break;
  }
}

// ============================================================================
// TELEMETRY & SERIAL PROTOCOL
// ============================================================================
void emitTelemetry() {
  // Protocol: D,<uptime_ms>,<dist_mm>,<pump 0|1>,<state>
  // Sending RAW distance as requested so gateway can perform rolling median
  Serial.print(F("D,"));
  Serial.print(millis());
  Serial.print(F(","));
  Serial.print(lastRawDistMm);
  Serial.print(F(","));
  Serial.print(pumpRelayActive ? 1 : 0);
  Serial.print(F(","));
  switch (currentState) {
    case STATE_IDLE:    Serial.println(F("IDLE")); break;
    case STATE_FILLING: Serial.println(F("FILLING")); break;
    case STATE_TRIP:    Serial.println(F("TRIP")); break;
  }
}

void parseSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufferIdx > 0) {
        serialBuffer[serialBufferIdx] = '\0';
        
        // Execute commands
        if (strcmp(serialBuffer, "K,RESET_TRIP") == 0) {
          if (currentState == STATE_TRIP) {
            currentState = STATE_IDLE;
            setBuzzerPattern(BUZZ_SHORT);
            lastPumpOffTimeMs = millis();
            Serial.print(F("E,"));
            Serial.print(millis());
            Serial.println(F(",RECOVERED"));
          }
        } else if (strcmp(serialBuffer, "K,PUMP_FORCE_OFF") == 0) {
          forcePumpOff = true;
          if (currentState == STATE_FILLING) {
            setPump(false, "PUMP_OFF_HIGH");
            currentState = STATE_IDLE;
          }
        } else if (strcmp(serialBuffer, "K,PUMP_AUTO") == 0) {
          forcePumpOff = false;
        }

        serialBufferIdx = 0; // Reset buffer
      }
    } else {
      if (serialBufferIdx < sizeof(serialBuffer) - 1) {
        serialBuffer[serialBufferIdx++] = c;
      }
    }
  }
}

// ============================================================================
// LCD USER INTERFACE
// ============================================================================
void updateLcd() {
  char line1[17];
  char line2[17];

  // Line 1: "Lvl  63% PUMP ON " or "Lvl  --% PUMP OFF"
  const char* pumpStr = pumpRelayActive ? "ON " : "OFF";
  if (sensorFaultActive || validSamplesCount == 0) {
    snprintf(line1, sizeof(line1), "Lvl ---%% PUMP %s", pumpStr);
  } else {
    snprintf(line1, sizeof(line1), "Lvl %3d%% PUMP %s", calculatedLevelPct, pumpStr);
  }

  // Line 2: State and Uptime/Alerts
  if (currentState == STATE_TRIP) {
    snprintf(line2, sizeof(line2), "TRIP: MAX RUNTIME");
  } else if (sensorFaultActive) {
    snprintf(line2, sizeof(line2), "ERR: SENSOR FLT ");
  } else {
    const char* stStr = (currentState == STATE_FILLING) ? "FILL" : "IDLE";
    unsigned long upSec = millis() / 1000UL;
    snprintf(line2, sizeof(line2), "%-4s  Up: %5lus", stStr, upSec);
  }

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
