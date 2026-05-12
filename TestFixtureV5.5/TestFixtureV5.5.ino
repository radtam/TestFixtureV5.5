/*
 * ============================================================================
 * ESP32 Automated Test Fixture Controller — V5.3  (sketch: TestFixtureV5_3.ino)
 * ============================================================================
 *
 * V5.3 = V5_2 (non-blocking LC task architecture)
 *      + V5.2 (typed P/D/R/S LINK_UART protocol consumed by the display ESP32)
 *
 * LINK_UART protocol (to display ESP32 — see main.cpp handleCommand):
 *   "P pos=<n>"            — Position update (jog done, setpos home)
 *   "D pos=<n> lc=<f>"     — Data point during in-motion LC sampling
 *   "R pos=<n> lc=<f>"     — Result from a stationary LC read (M100 / read lc)
 *   "S <state>"            — State change: ready | paused | complete
 *
 * Architecture:
 *   Core 0 (Priority 5) : taskStepper  — AccelStepper::run() / runSpeed() (force seek)
 *   Core 1 (Priority 3) : taskControl  — Serial I/O, state machine, test execution
 *   Core 1 (Priority 2) : taskLcSample — HX711 read loop; writes g_lcLatest cache.
 *                         Only samples when the current sequence step needs LC data
 *                         (MOVE+lcInMotion, SEEK_FORCE, READ_LC, or ADJ seek active).
 *                         taskControl reads the cache non-blocking; no HX711 blocking
 *                         on the control path during motion or PID loops.
 *
 * Modes:
 *   SETUP       — Define test sequence via G-code, configure hardware settings
 *   ADJUSTMENTS — Jog motor, set named positions (XA/XB/XC/XD), calibrate LC
 *   LIVE_TEST   — Execute test cycles (start / pause / resume / stop / status)
 *
 * G-code format (SETUP mode):
 *   G0 X<steps|XA..XD> [F<speed>] [A<accel>]
 *      [LC [R<samples>] [RT<ms>] [P<s1,s2,...>] [L<0|1>]]
 *                                               — Move, with optional in-motion LC
 *   G4 P<ms>                                   — Dwell / wait
 *   M100 [R<samples>] [L<0|1>]                 — Stationary LC read (motor stopped)
 *
 * In-motion LC sampling on G0:
 *   LC           — enable in-motion sampling on this move step
 *   RT<ms>       — rate-based: take a reading every <ms> milliseconds
 *   P<s1,s2,...> — position-based: take a reading when motor passes each absolute step
 *   Both RT and P can be combined on one G0 line
 *   R<n>         — number of averaged samples per reading (default from cfg.lc_inMotionSamples)
 *   L1           — pause test immediately if a reading exceeds limits
 *
 * Cycle-profile mode (LIVE_TEST):
 *   start 10                    — run 10 cycles; LC behaviour defined per G0 step
 *   start 10 LC every 3         — full LC logging only on cycles 3,6,9; others check limits only
 *   start 10 LC cycles 1,5,10   — full LC logging only on those specific cycles
 *
 *   M101 — Seek to target force (PI+D velocity loop, sequence step)
 *
 * Libraries required:
 *   AccelStepper, HX711_MP, Preferences (built-in ESP32 Arduino)
 * ============================================================================
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <HX711_MP.h>
#include <Preferences.h>
#include <string.h>
#include "fixture_hal.h"

// ============================================================================
// PIN DEFINITIONS  (edit for your hardware)
// ============================================================================

static const int STEP_PIN   = 14;
static const int DIR_PIN    = 13;
static const int ENABLE_PIN = 12;   // -1 if unused; LOW = enabled

#define LC_DT_PIN   32
#define LC_SCK_PIN  33

constexpr int LINK_RX_PIN = 16;
constexpr int LINK_TX_PIN = 17;
#define LINK_UART Serial2

// ---- Servo + break-beam pin selection -------------------------------------
// SERVO_PIN  : 27 — PWM-capable via LEDC, not a strapping pin, not a DAC pin
//              (leaves GPIO 25/26 free if you ever want DAC).
// BREAK1_PIN : 35 — input-only, boot-safe, no internal pull-up
// BREAK2_PIN : 34 — input-only, boot-safe, no internal pull-up
//
// Most break-beam modules (Adafruit IR break-beam, common diffuse photo-
// electrics) supply their own pull-up resistor, so GPIO 34/35 work fine.
// If you wire a bare phototransistor with NO pull-up, move BREAK1/2 to
// GPIO 18/19 (which support INPUT_PULLUP) and enable `set break*_pullup 1`.
static const int SERVO_PIN  = 27;
static const int BREAK1_PIN = 35;
static const int BREAK2_PIN = 34;

// LEDC PWM config for the servo (50 Hz, 16-bit duty resolution).
// 16 bits at 50 Hz gives ~0.3 us resolution which is plenty for a servo.
static const uint32_t SERVO_LEDC_FREQ_HZ  = 50;
static const uint8_t  SERVO_LEDC_RES_BITS = 16;
static const uint8_t  SERVO_LEDC_CHANNEL  = 0;   // used only on Arduino-ESP32 v2.x

// ============================================================================
// COMPILE-TIME CONSTANTS
// ============================================================================

#define MAX_TEST_STEPS      64    // Max G-code steps per sequence
#define LC_INDEX_SIZE       10    // HX711_MP multipoint calibration slots
#define NUM_NAMED_POS        4    // XA, XB, XC, XD
#define NUM_NAMED_SERVO_POS  4    // SA, SB, SC, SD
#define MAX_LC_POSITIONS    16    // Max position-trigger points per G0 step
#define MAX_PROFILE_CYCLES  32    // Max entries in explicit cycle list

#define NVS_MAGIC        0xC0BEEF02u
#define NVS_VERSION      5u          // bumped: added servo + break-beam config fields
#define SEQ_MAGIC        0x5E900005u // bumped: added SERVO_MOVE, WAIT_BB, labels

#define LC_RING_CAP      512
#define SEQ_NVS_KEY_HDR  "seq_hdr"
#define SEQ_NVS_KEY_DATA "seq_data"

// ============================================================================
// ENUMS
// ============================================================================

enum class Mode     : uint8_t { SETUP=0, ADJUSTMENTS=1, LIVE_TEST=2 };
enum class RunState : uint8_t { IDLE=0, RUNNING=1, PAUSED=2, COMPLETE=3 };
enum class StepType : uint8_t { MOVE=0, DWELL=1, READ_LC=2, SEEK_FORCE=3, PAUSE=4,
                               SERVO_MOVE=5, WAIT_BB=6 };

// How cycle-profile LC selection works when "start N LC ..." is used
enum class ProfileMode : uint8_t {
  ALL     = 0,   // every cycle logs LC (default, same as plain "start N")
  EVERY_N = 1,   // log on cycles that are multiples of profileEveryN
  EXPLICIT = 2   // log only on cycles listed in profileCycles[]
};

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/*
 * TestStep — one entry in the programmable G-code sequence.
 *
 * In-motion LC sampling (MOVE steps only):
 *   lcInMotion    — true if any LC sampling is enabled for this move
 *   lcSamples     — samples to average per reading (0 = use cfg default)
 *   lcCheckLimits — pause test if reading exceeds limits
 *   lcRateMs      — if > 0: take a reading every lcRateMs during the move
 *   lcPosCount    — number of entries in lcPositions[]
 *   lcPositions[] — absolute step positions at which to take a reading
 *
 * Both rate and position triggers can be active simultaneously on one move.
 */
struct TestStep {
  StepType type;

  // Step label (1-255 = named label, 0 = unlabeled).
  // Set with N<n> on any G-code line; referenced by J<n> on limit/beam-fail steps.
  uint8_t stepLabel;
  // On limit/beam failure: jump to step with stepLabel==gotoLabel instead of pausing.
  // 0 = no jump (default: pause on failure).
  uint8_t gotoLabel;

  // --- MOVE ---
  bool    useNamedPos;
  uint8_t posVarIdx;          // 0=XA, 1=XB, 2=XC, 3=XD
  long    targetSteps;
  float   speed;              // 0 = use cfg default
  float   accel;              // 0 = use cfg default

  // In-motion LC (MOVE only)
  bool    lcInMotion;
  uint8_t lcSamples;          // 0 = use cfg.lc_inMotionSamples
  bool    lcCheckLimits;
  uint32_t lcRateMs;          // 0 = rate-based disabled
  uint8_t  lcPosCount;        // 0 = position-based disabled
  long     lcPositions[MAX_LC_POSITIONS];
  float lcStepUpperLimit;
  float lcStepLowerLimit;

  // Concurrent servo (MOVE only): fire servo move at start of motor move
  bool    servoWithMove;
  bool    servoMoveUseNamed;  // true = named pos (SA..SD), false = degree value
  uint8_t servoMoveVarIdx;    // 0=SA..3=SD (when servoMoveUseNamed)
  float   servoMoveDeg;       // degrees (when !servoMoveUseNamed)

  // --- DWELL ---
  uint32_t dwellMs;

  // --- READ_LC (stationary) ---
  uint8_t lcReadings;
  bool    checkLimits;
  float   readStepUpperLimit;
  float   readStepLowerLimit;

  // --- SEEK_FORCE (M101) — PI+D on force, velocity via runSpeed() ---
  float    sfTarget;
  float    sfMaxSps;
  float    sfCreepSps;
  float    sfKp;
  float    sfKi;
  float    sfKd;
  float    sfEpsilon;
  uint32_t sfSettleMs;
  int8_t   sfDir;
  long     sfMaxTravel;  // 0 = no limit (|pos - start|)

  // --- SERVO_MOVE (M200) ---
  bool    servoUseNamed;      // true = named pos (SA..SD), false = degree value
  uint8_t servoVarIdx;        // 0=SA..3=SD (when servoUseNamed)
  float   servoDeg;           // degrees (when !servoUseNamed)

  // --- WAIT_BB (M202) ---
  uint8_t  bbSensor;          // 1 or 2
  bool     bbExpected;        // true=expect broken, false=expect clear
  bool     bbWaitMode;        // false=pause-if-wrong, true=wait-until-right
  uint32_t bbTimeoutMs;       // 0=infinite (wait mode only)
};

/*
 * Persistent configuration.
 * NVS_VERSION was bumped — old saved data will be rejected and defaults used.
 */
struct Config {
  // Stepper
  float maxSpeed_sps   = 1200.0f;
  float accel_sps2     = 8000.0f;
  long  stepsPerRev    = 400;

  // Load cell limits (used by all LC read paths)
  float lc_upperLimit  = 9999.0f;
  float lc_lowerLimit  = -9999.0f;
  bool  lc_limitsOn    = true;

  float lc_knownWeight = 10.0f;

  // Default in-motion LC settings (used when G0 LC params are omitted)
  // R=1 minimizes blocking (each HX711 conversion ~100 ms @ 10 SPS, ~12.5 ms @ 80 SPS).
  uint8_t  lc_inMotionSamples = 1;
  uint32_t lc_inMotionRateMs  = 100;  // default RT when LC has no explicit RT/P

  // High-rate LC ring buffer (optional; reduces printf jitter)
  bool     lc_ring_enable       = false;
  uint16_t lc_ring_flush_thresh = 64;
  uint16_t lc_sampler_period_ms = 0;  // 0 = off; background samples during LC moves (uses lc_mutex)

  // Default M101 / interactive seek-force tuning
  float    seek_kp         = 80.f;
  float    seek_ki         = 5.f;
  float    seek_kd         = 0.02f;
  float    seek_vmax       = 600.f;
  float    seek_vcreep     = 40.f;
  float    seek_epsilon    = 0.08f;
  uint32_t seek_settle_ms  = 200;
  int8_t   seek_dir        = 1;

  uint8_t  link_proto_version = LINK_PROTO_VERSION;

  // ---- Servo ----
  // Calibrate min/max pulse widths to match YOUR servo's true 0..180 deg range.
  // Common hobby servos: 500..2500 us; SG90 clones often: 1000..2000 us.
  uint16_t servoMinPulseUs = 500;
  uint16_t servoMaxPulseUs = 2500;
  float    servoCurrentDeg = 90.0f;        // last commanded angle (for restore)
  float    servoNamedPositions[NUM_NAMED_SERVO_POS] = {0.f, 0.f, 0.f, 0.f};
  bool     servoNamedPosSet  [NUM_NAMED_SERVO_POS] = {false, false, false, false};

  // ---- Break-beam sensors ----
  // active_low=true means the pin reads LOW when the beam is broken
  // (default for the Adafruit IR break-beam and most photoelectric modules).
  bool     break1ActiveLow = true;
  bool     break2ActiveLow = true;
  // Use internal pull-up at pin init. Only effective on pins that support it
  // (input-only GPIO 34/35/36/39 ignore this and need an external pull-up).
  bool     break1UsePullup = false;
  bool     break2UsePullup = false;

  // Named positions (XA..XD)
  long namedPositions[NUM_NAMED_POS] = {0, 0, 0, 0};
  bool namedPosSet   [NUM_NAMED_POS] = {false, false, false, false};

  // Load cell calibration matrix
  float lc_calRaw   [LC_INDEX_SIZE] = {};
  float lc_calWeight[LC_INDEX_SIZE] = {};
  bool  lc_calUsed  [LC_INDEX_SIZE] = {};
} cfg;

// ============================================================================
// GLOBALS
// ============================================================================

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
HX711_MP     lc(LC_INDEX_SIZE);
LoadCellSensor<HX711_MP>    g_loadCell(lc);
StepperActuator<AccelStepper> g_ramMotor(stepper, ENABLE_PIN);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
ServoActuator      g_servo(SERVO_PIN, SERVO_LEDC_FREQ_HZ, SERVO_LEDC_RES_BITS);
#else
ServoActuator      g_servo(SERVO_PIN, SERVO_LEDC_FREQ_HZ, SERVO_LEDC_RES_BITS, SERVO_LEDC_CHANNEL);
#endif
BreakBeamSensor    g_break1(BREAK1_PIN, /*activeLow=*/true, /*pullup=*/false);
BreakBeamSensor    g_break2(BREAK2_PIN, /*activeLow=*/true, /*pullup=*/false);

// Test sequence
TestStep testSequence[MAX_TEST_STEPS];
int      numSteps = 0;

// Execution state
volatile int      currentStepIdx  = 0;
volatile bool     stepInitialized = false;
volatile uint32_t stepStartMs     = 0;

// Mode / run state
volatile Mode     currentMode = Mode::SETUP;
volatile RunState runState    = RunState::IDLE;
volatile int      cyclesDone  = 0;
volatile int      cyclesGoal  = 1;
volatile bool     lcLimitTrip = false;

// Cycle-profile LC mode — set when "start N LC ..." is parsed
ProfileMode profileMode     = ProfileMode::ALL;
int         profileEveryN   = 1;
int         profileCycles[MAX_PROFILE_CYCLES];
int         profileCycleCount = 0;



// In-motion LC state — reset at the start of each MOVE step
uint32_t lcLastRateMs       = 0;
uint8_t  lcNextPosIdx       = 0;

// Jog position reporting — Core 0 writes, Core 1 reads and prints.
// Keeping Serial.printf out of the stepper hot-loop eliminates UART-stall jitter.
volatile long    jogReportPos    = LONG_MIN;  // LONG_MIN = nothing pending
volatile bool    jogMoveDone     = false;     // set true when move completes in ADJ mode
volatile long    jogFinalPos     = 0;

// Shared last LC value — written by Core 1 LC reads, read by taskControl heartbeat
volatile float   lastLcValue     = 0.0f;

// Calibration bulk-upload state
volatile bool    inCalUploadMode = false;
volatile int     calUploadSlot   = 0;   // next slot index to write into

// Inter-task sync
SemaphoreHandle_t cfgMutex   = nullptr;
SemaphoreHandle_t seqMutex   = nullptr;
SemaphoreHandle_t lcMutex    = nullptr;
SemaphoreHandle_t lcRingMux  = nullptr;
SemaphoreHandle_t lcLatestMux = nullptr;  // guards g_lcLatest

/*
 * g_lcLatest — written by taskLcSample, read non-blocking by taskControl.
 * Eliminates HX711 blocking on the control/PID path during motion.
 * 'fresh' is set true when a new sample arrives; consumed by seekForceProgress
 * so the PID only runs at the true HX711 rate, not at taskControl's tick rate.
 */
struct LcLatest {
  float    value  = 0.f;
  long     pos    = 0;
  uint32_t t_ms   = 0;
  bool     fresh  = false;
} g_lcLatest;

struct LcRingSample { uint32_t t_ms; long pos; float lc; };
LcRingSample lcRing[LC_RING_CAP];
volatile uint16_t lcRingHead = 0;
volatile uint16_t lcRingTail = 0;

volatile bool  g_velocitySeekMode = false;
volatile float g_velocityCommand  = 0.f;

struct SeekRuntime {
  bool     pidInit;
  uint32_t lastPidMs;
  float    integral;
  float    lastErr;
  bool     inBand;
  uint32_t inBandSince;
  long     startPos;
};
static SeekRuntime g_seekRT;

static bool    g_adjSeekActive = false;
static TestStep g_adjSeekStep = {};

TaskHandle_t taskLcSampleHandle = nullptr;

Preferences prefs;
volatile bool inUploadMode = false;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void handleCommand(const String &line);
void parseGCode(const String &line);
void taskLcSample(void *pv);
void lc_ring_flush();
uint16_t lc_ring_count();
void serviceAdjSeekForce();
void servo_init();
void servo_writeDegrees(float deg);
void servo_writeMicros(uint16_t us);
void break_applyConfig();
int  servoNamedIdx(char letter);
void handleServoCommand(const String &args);
void handleReadBreakCommand(const String &args);

// ============================================================================
// UTILITIES
// ============================================================================

inline void lockCfg()   { xSemaphoreTake(cfgMutex, portMAX_DELAY); }
inline void unlockCfg() { xSemaphoreGive(cfgMutex); }
inline void lockSeq()   { xSemaphoreTake(seqMutex,  portMAX_DELAY); }
inline void unlockSeq() { xSemaphoreGive(seqMutex); }

void enableMotor(bool en) {
  if (ENABLE_PIN >= 0) digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
}

const char* modeStr(Mode m) {
  switch (m) {
    case Mode::SETUP:       return "SETUP";
    case Mode::ADJUSTMENTS: return "ADJUSTMENTS";
    case Mode::LIVE_TEST:   return "LIVE_TEST";
  }
  return "?";
}
const char* stateStr(RunState s) {
  switch (s) {
    case RunState::IDLE:     return "IDLE";
    case RunState::RUNNING:  return "RUNNING";
    case RunState::PAUSED:   return "PAUSED";
    case RunState::COMPLETE: return "COMPLETE";
  }
  return "?";
}

int namedPosIdx(char letter) {
  letter = toupper((unsigned char)letter);
  if (letter >= 'A' && letter < 'A' + NUM_NAMED_POS) return letter - 'A';
  return -1;
}

/*
 * Returns true if, for the current cycle number, LC logging should be active.
 * cyclesDone is 0-based here (the cycle currently executing).
 * We use 1-based numbers in the user-facing commands for readability.
 */
bool lcActiveThisCycle() {
  int cycleNum = cyclesDone + 1;   // 1-based
  switch (profileMode) {
    case ProfileMode::ALL:      return true;
    case ProfileMode::EVERY_N:  return (profileEveryN > 0) && (cycleNum % profileEveryN == 0);
    case ProfileMode::EXPLICIT:
      for (int i = 0; i < profileCycleCount; i++)
        if (profileCycles[i] == cycleNum) return true;
      return false;
  }
  return true;
}

// ============================================================================
// PERSISTENT STORAGE — Config
// ============================================================================

struct NvsHeader { uint32_t magic; uint16_t version; uint16_t length; };

bool saveConfig() {
  lockCfg();
  NvsHeader h = { NVS_MAGIC, NVS_VERSION, (uint16_t)sizeof(Config) };
  prefs.begin("fixture", false);
  bool ok = (prefs.putBytes("hdr", &h,   sizeof(h))   == sizeof(h)) &&
            (prefs.putBytes("cfg", &cfg, sizeof(cfg)) == sizeof(cfg));
  prefs.end();
  unlockCfg();
  return ok;
}

bool loadConfig() {
  prefs.begin("fixture", true);
  NvsHeader h;
  size_t n = prefs.getBytes("hdr", &h, sizeof(h));
  bool ok = (n == sizeof(h) && h.magic == NVS_MAGIC &&
             h.version == NVS_VERSION && h.length == sizeof(Config));
  if (!ok) { prefs.end(); return false; }
  Config tmp;
  n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  prefs.end();
  if (n != sizeof(tmp)) return false;
  lockCfg();
  cfg = tmp;
  stepper.setMaxSpeed(cfg.maxSpeed_sps);
  stepper.setAcceleration(cfg.accel_sps2);
  unlockCfg();
  return true;
}

bool wipeConfig() {
  prefs.begin("fixture", false);
  bool ok = prefs.remove("hdr") && prefs.remove("cfg");
  prefs.end();
  return ok;
}

// ============================================================================
// PERSISTENT STORAGE — Sequence
// ============================================================================

struct SeqHeader {
  uint32_t magic;
  uint16_t stepSize;
  uint16_t numSteps;
};

bool saveSequence() {
  lockSeq();
  SeqHeader h = { SEQ_MAGIC, (uint16_t)sizeof(TestStep), (uint16_t)numSteps };
  size_t dataBytes = numSteps * sizeof(TestStep);
  prefs.begin("fixture", false);
  bool ok = (prefs.putBytes(SEQ_NVS_KEY_HDR, &h, sizeof(h)) == sizeof(h));
  if (ok && dataBytes > 0)
    ok = (prefs.putBytes(SEQ_NVS_KEY_DATA, testSequence, dataBytes) == dataBytes);
  prefs.end();
  unlockSeq();
  return ok;
}

bool loadSequence() {
  prefs.begin("fixture", true);
  SeqHeader h;
  size_t n = prefs.getBytes(SEQ_NVS_KEY_HDR, &h, sizeof(h));
  bool ok = (n == sizeof(h) && h.magic == SEQ_MAGIC &&
             h.stepSize == sizeof(TestStep) && h.numSteps <= MAX_TEST_STEPS);
  if (!ok) { prefs.end(); return false; }
  size_t dataBytes = h.numSteps * sizeof(TestStep);
  lockSeq();
  if (dataBytes > 0) {
    n = prefs.getBytes(SEQ_NVS_KEY_DATA, testSequence, dataBytes);
    if (n != dataBytes) { unlockSeq(); prefs.end(); return false; }
  }
  numSteps       = h.numSteps;
  currentStepIdx = 0;
  unlockSeq();
  prefs.end();
  return true;
}

bool wipeSequence() {
  prefs.begin("fixture", false);
  bool ok = prefs.remove(SEQ_NVS_KEY_HDR) && prefs.remove(SEQ_NVS_KEY_DATA);
  prefs.end();
  return ok;
}

// ============================================================================
// LOAD CELL  (HX711_MP multipoint calibration)
// ============================================================================

void lc_printMatrix() {
  Serial.println(F("[LC] Calibration matrix:"));
  bool any = false;
  for (int i = 0; i < LC_INDEX_SIZE; i++) {
    if (cfg.lc_calUsed[i]) {
      Serial.printf("  [%d] raw=%.2f  weight=%.3f\n", i, cfg.lc_calRaw[i], cfg.lc_calWeight[i]);
      any = true;
    }
  }
  if (!any) Serial.println(F("  (empty)"));
}

void lc_applyCalibration() {
  int n = 0;
  if (lcMutex) xSemaphoreTake(lcMutex, portMAX_DELAY);
  for (int i = 0; i < LC_INDEX_SIZE; i++) {
    lc.setCalibrate(i, cfg.lc_calUsed[i] ? cfg.lc_calRaw[i] : 0,
                       cfg.lc_calUsed[i] ? cfg.lc_calWeight[i] : 0);
    if (cfg.lc_calUsed[i]) n++;
  }
  if (lcMutex) xSemaphoreGive(lcMutex);
  if (n < 2) Serial.println(F("[LC] WARNING: need >= 2 calibration points!"));
  else       Serial.printf("[LC] Applied %d calibration points.\n", n);
}

void lc_clearCalibration() {
  if (lcMutex) xSemaphoreTake(lcMutex, portMAX_DELAY);
  for (int i = 0; i < LC_INDEX_SIZE; i++) {
    cfg.lc_calRaw[i] = 0; cfg.lc_calWeight[i] = 0; cfg.lc_calUsed[i] = false;
    lc.setCalibrate(i, 0, 0);
  }
  if (lcMutex) xSemaphoreGive(lcMutex);
}

void lc_tare() {
  lc_clearCalibration();
  float raw;
  if (lcMutex) xSemaphoreTake(lcMutex, portMAX_DELAY);
  raw = lc.read_average(5);
  cfg.lc_calRaw[0] = raw; cfg.lc_calWeight[0] = 0; cfg.lc_calUsed[0] = true;
  lc.setCalibrate(0, raw, 0);
  if (lcMutex) xSemaphoreGive(lcMutex);
  Serial.printf("[LC] Tare: raw=%.2f -> 0.\n", raw);
  lc_printMatrix();
}

void lc_addCalPoint(float knownWeight) {
  int slot = -1;
  for (int i = 1; i < LC_INDEX_SIZE; i++) if (!cfg.lc_calUsed[i]) { slot = i; break; }
  if (slot < 0) { Serial.println(F("[LC] Matrix full!")); return; }
  float raw;
  if (lcMutex) xSemaphoreTake(lcMutex, portMAX_DELAY);
  raw = lc.read_average(5);
  if (lcMutex) xSemaphoreGive(lcMutex);
  cfg.lc_calRaw[slot] = raw; cfg.lc_calWeight[slot] = knownWeight; cfg.lc_calUsed[slot] = true;
  Serial.printf("[LC] Added point %d: weight=%.3f  raw=%.2f\n", slot, knownWeight, raw);
  LINK_UART.printf("C i=%d  wgt=%.3f  raw=%.2f\n", slot, knownWeight, raw);
  lc_printMatrix();
  lc_applyCalibration();
}

float lc_read(int samples = 3) {
  if (!lcMutex) return lc.get_units(samples);
  xSemaphoreTake(lcMutex, portMAX_DELAY);
  float v = lc.get_units(samples);
  xSemaphoreGive(lcMutex);
  return v;
}

uint16_t lc_ring_count() {
  if (lcRingMux) xSemaphoreTake(lcRingMux, portMAX_DELAY);
  uint16_t n = (uint16_t)((LC_RING_CAP + lcRingHead - lcRingTail) % LC_RING_CAP);
  if (lcRingMux) xSemaphoreGive(lcRingMux);
  return n;
}

void lc_ring_push(uint32_t t_ms, long pos, float w) {
  if (lcRingMux) xSemaphoreTake(lcRingMux, portMAX_DELAY);
  uint16_t next = (uint16_t)((lcRingHead + 1) % LC_RING_CAP);
  if (next == lcRingTail) lcRingTail = (uint16_t)((lcRingTail + 1) % LC_RING_CAP);
  lcRing[lcRingHead].t_ms = t_ms;
  lcRing[lcRingHead].pos = pos;
  lcRing[lcRingHead].lc  = w;
  lcRingHead = next;
  if (lcRingMux) xSemaphoreGive(lcRingMux);
}

void lc_ring_flush() {
  if (lcRingMux) xSemaphoreTake(lcRingMux, portMAX_DELAY);
  while (lcRingTail != lcRingHead) {
    const LcRingSample &s = lcRing[lcRingTail];
    Serial.printf("[LCbuf] t=%lu pos=%ld lc=%.4f\n", (unsigned long)s.t_ms, s.pos, s.lc);
    lcRingTail = (uint16_t)((lcRingTail + 1) % LC_RING_CAP);
  }
  if (lcRingMux) xSemaphoreGive(lcRingMux);
}

/*
 * seekForceProgress — PI+D on force error, commands velocity via g_velocityCommand.
 * Returns 0 = continue, 1 = target captured (step done), 2 = paused (limit / travel).
 *
 * Force value is read from g_lcLatest (written by taskLcSample) — non-blocking.
 * The PID only recalculates when a fresh sample is available, so the effective
 * PID rate matches the HX711 sample rate rather than the taskControl tick rate.
 */
static int seekForceProgress(const TestStep &step) {
  if (!g_seekRT.pidInit) {
    g_seekRT.pidInit     = true;
    g_seekRT.lastPidMs   = millis();
    g_seekRT.integral    = 0.f;
    g_seekRT.lastErr     = 0.f;
    g_seekRT.inBand      = false;
    g_seekRT.startPos    = stepper.currentPosition();
    g_velocitySeekMode   = true;
    stepper.setMaxSpeed(step.sfMaxSps);
    return 0;
  }

  // Non-blocking cache read — only proceed if taskLcSample has a new value
  xSemaphoreTake(lcLatestMux, portMAX_DELAY);
  bool fresh = g_lcLatest.fresh;
  float f    = g_lcLatest.value;
  g_lcLatest.fresh = false;
  xSemaphoreGive(lcLatestMux);

  if (!fresh) return 0;  // no new sample yet — hold current velocity command

  lastLcValue = f;

  uint32_t now_ms = millis();
  float dt_s = (now_ms - g_seekRT.lastPidMs) * 0.001f;
  g_seekRT.lastPidMs = now_ms;
  if (dt_s < 0.0005f) dt_s = 0.0005f;
  if (dt_s > 0.35f) dt_s = 0.35f;

  float err = step.sfTarget - f;
  float derr = (err - g_seekRT.lastErr) / dt_s;
  g_seekRT.lastErr = err;
  g_seekRT.integral += err * dt_s;
  float iMax = step.sfMaxSps / fmaxf(1e-6f, step.sfKi);
  g_seekRT.integral = constrain(g_seekRT.integral, -iMax, iMax);

  float v = step.sfKp * err + step.sfKi * g_seekRT.integral + step.sfKd * derr;
  v = constrain(v, -step.sfMaxSps, step.sfMaxSps);
  if (fabsf(v) < step.sfCreepSps && fabsf(err) > step.sfEpsilon)
    v = (err > 0.f ? step.sfCreepSps : -step.sfCreepSps);
  v *= (float)step.sfDir;
  g_velocityCommand = v;

  if (step.sfMaxTravel > 0) {
    long traveled = labs(stepper.currentPosition() - g_seekRT.startPos);
    if (traveled > step.sfMaxTravel) {
      Serial.printf("[SEEK] Max travel %ld steps exceeded — pausing.\n", (long)step.sfMaxTravel);
      g_velocitySeekMode = false;
      g_velocityCommand  = 0.f;
      stepper.stop();
      memset(&g_seekRT, 0, sizeof g_seekRT);
      if (currentMode == Mode::LIVE_TEST) {
        runState    = RunState::PAUSED;
        lcLimitTrip = true;
      }
      return 2;
    }
  }

  if (cfg.lc_limitsOn) {
    if (f > cfg.lc_upperLimit || f < cfg.lc_lowerLimit) {
      Serial.println(F("[SEEK] LC global limit — pausing."));
      g_velocitySeekMode = false;
      g_velocityCommand  = 0.f;
      stepper.stop();
      memset(&g_seekRT, 0, sizeof g_seekRT);
      if (currentMode == Mode::LIVE_TEST) {
        runState    = RunState::PAUSED;
        lcLimitTrip = true;
      }
      return 2;
    }
  }

  if (fabsf(err) < step.sfEpsilon) {
    if (!g_seekRT.inBand) {
      g_seekRT.inBand      = true;
      g_seekRT.inBandSince = now_ms;
    } else if (now_ms - g_seekRT.inBandSince >= step.sfSettleMs) {
      g_velocitySeekMode = false;
      g_velocityCommand  = 0.f;
      stepper.stop();
      memset(&g_seekRT, 0, sizeof g_seekRT);
      Serial.printf("[SEEK] Target reached: lc=%.4f (target %.4f)\n", f, step.sfTarget);
      return 1;
    }
  } else {
    g_seekRT.inBand = false;
  }
  return 0;
}

void serviceAdjSeekForce() {
  if (!g_adjSeekActive) return;
  int r = seekForceProgress(g_adjSeekStep);
  if (r == 1) {
    g_adjSeekActive    = false;
    g_velocitySeekMode = false;
    g_velocityCommand  = 0.f;
    stepper.stop();
    lockCfg();
    if (cfg.lc_ring_enable) {
      unlockCfg();
      lc_ring_flush();
    } else {
      unlockCfg();
    }
    Serial.println(F("[ADJ] Seek complete."));
  } else if (r == 2) {
    g_adjSeekActive = false;
    Serial.println(F("[ADJ] Seek aborted (limit or travel)."));
  }
}

// Forward declarations — defined later in the execution section
int  findStepByLabel(uint8_t label);
bool handleFailGoto(const TestStep &step);

/*
 * lc_inMotionRead — reads the latest LC value from the g_lcLatest cache.
 * Non-blocking: taskLcSample owns the HX711 conversion; this function
 * just consumes the result. The 'pos' argument is the motor position at
 * the moment of the trigger (rate or position), which is more accurate
 * than the cached pos from when taskLcSample last read.
 * Returns false if the test should be paused (limit tripped with L1).
 */
bool lc_inMotionRead(const TestStep &step, long pos, bool doLog) {
  // Non-blocking cache read
  xSemaphoreTake(lcLatestMux, portMAX_DELAY);
  float val = g_lcLatest.value;
  xSemaphoreGive(lcLatestMux);

  lastLcValue = val;

  if (doLog) {
    bool useRing;
    lockCfg();
    useRing = cfg.lc_ring_enable;
    unlockCfg();
    if (useRing) {
      lc_ring_push(millis(), pos, val);
    } else {
      //Serial.printf("[LIVE] t=%lu  cyc=%d/%d  step=%d/%d  pos=%ld  lc=%.4f\n",
      //              (unsigned long)millis(), cyclesDone, cyclesGoal,
      //              currentStepIdx, numSteps,
       //             pos, val);
      Serial.printf("%lu,  %d,  %ld, %.4f\n", (unsigned long)millis(), cyclesDone, pos, val);
      //LINK_UART.printf("%lu,  %d,  %ld, %.4f\n", (unsigned long)millis(), cyclesDone, pos, val);
      LINK_UART.printf("D pos=%ld lc=%.4f\n", pos, val);  // D = Data point (in-motion)
    }
  }
  //Example from below
  // Serial.printf("[LIVE] t=%lu  cyc=%d/%d  step=%d/%d  pos=%ld  lc=%.4f  %s%s\n",
  //                       (unsigned long)now, cyclesDone, cyclesGoal,
  //                       currentStepIdx, numSteps,
  //                       stepper.currentPosition(),
  //                       lastLcValue,
  //                       stateStr(runState),
  //                       lcLimitTrip ? "  [LC_TRIP]" : "");

  //if (step.lcCheckLimits && cfg.lc_limitsOn) {
    //lockCfg();
    //bool over = (val > cfg.lc_upperLimit || val < cfg.lc_lowerLimit);
    //unlockCfg();
  
  if (cfg.lc_limitsOn) {
    float upper = step.lcCheckLimits ? step.lcStepUpperLimit : cfg.lc_upperLimit;
    float lower = step.lcCheckLimits ? step.lcStepLowerLimit : cfg.lc_lowerLimit;
    bool outside = (val > upper || val < lower);
    if (outside) {
      Serial.printf("[TEST] LC LIMIT EXCEEDED cyc=%d pos=%ld val=%.4f (range [%.3f,%.3f])\n",
                    cyclesDone + 1, pos, val, lower, upper);
      lcLimitTrip = true;
      stepper.stop();        // always stop motor on limit trip
      handleFailGoto(step);  // then either jump to label or pause
      return false;
    }
  }
  return true;
}

// ============================================================================
// SERVO  (LEDC PWM on SERVO_PIN, 50 Hz)
// ============================================================================
/*
 * The servo is driven from the ADJUSTMENTS-mode command path on Core 1.
 * No mutex is needed — LEDC duty updates are atomic at the peripheral level
 * and we are the only writer.
 */

void servo_init() {
  if (!g_servo.begin()) {
    Serial.printf("[SERVO] WARNING: LEDC attach failed on GPIO %d\n", SERVO_PIN);
    return;
  }
  lockCfg();
  uint16_t minUs = cfg.servoMinPulseUs;
  uint16_t maxUs = cfg.servoMaxPulseUs;
  float    deg   = cfg.servoCurrentDeg;
  unlockCfg();
  g_servo.writeDegrees(deg, minUs, maxUs);
  Serial.printf("[SERVO] Init pin=%d  %.1f deg  pulse=%u us  range=%u..%u us\n",
                SERVO_PIN, deg, g_servo.lastUs, minUs, maxUs);
}

void servo_writeMicros(uint16_t us) {
  g_servo.writeMicroseconds(us);
}

void servo_writeDegrees(float deg) {
  if (deg < 0.f) deg = 0.f;
  if (deg > 180.f) deg = 180.f;
  lockCfg();
  uint16_t minUs = cfg.servoMinPulseUs;
  uint16_t maxUs = cfg.servoMaxPulseUs;
  cfg.servoCurrentDeg = deg;
  unlockCfg();
  g_servo.writeDegrees(deg, minUs, maxUs);
}

/*
 * Name-letter -> index for SA..SD. Mirrors namedPosIdx() but kept separate so
 * future expansion of servo slots doesn't have to track the motor slot count.
 */
int servoNamedIdx(char letter) {
  letter = toupper((unsigned char)letter);
  if (letter >= 'A' && letter < 'A' + NUM_NAMED_SERVO_POS) return letter - 'A';
  return -1;
}

// ============================================================================
// BREAK-BEAM SENSORS  (digital inputs)
// ============================================================================

/*
 * (Re)applies polarity + pull-up config to both break-beam pins.
 * Call from setup() after loadConfig() and any time the related cfg.* fields
 * change at runtime.
 */
void break_applyConfig() {
  lockCfg();
  g_break1.activeLow = cfg.break1ActiveLow;
  g_break1.usePullup = cfg.break1UsePullup;
  g_break2.activeLow = cfg.break2ActiveLow;
  g_break2.usePullup = cfg.break2UsePullup;
  unlockCfg();
  g_break1.begin();
  g_break2.begin();
}

// ============================================================================
// HELP & STATUS
// ============================================================================

void printHelp() {
  Serial.println(F(
    "\n============ COMMANDS ============\n"
    "GLOBAL (any mode):\n"
    "  help | status | flush lc\n"
    "  mode setup | adj | live\n"
    "  save        -- Save config + sequence to NVS\n"
    "  load        -- Load config + sequence from NVS\n"
    "  wipe        -- Erase config AND sequence from NVS\n"
    "  save seq | load seq | wipe seq\n"
    "\nSETUP MODE — sequence building:\n"
    "  G0 X<steps|XA..XD> [F<sps>] [A<sps2>]   -- Move (no LC)\n"
    "  G0 X<..> [F<..>] [A<..>] LC [R<n>] [RT<ms>] [P<s1,s2,...>] [L<0|1>]\n"
    "                                             -- Move WITH in-motion LC\n"
    "    LC        -- enable in-motion sampling\n"
    "    R<n>      -- samples to average per reading (default: lc_motion_samples)\n"
    "    RT<ms>    -- rate-based: read every <ms> during move\n"
    "    P<s,...>  -- position-based: read when pos passes each step value\n"
    "    L1        -- pause test if reading exceeds limits\n"
    "    SN<A-D>   -- also move servo to named position at start of move\n"
    "    SV<deg>   -- also move servo to degree value (0-180) at start of move\n"
    "  G4 P<ms>                  -- Dwell\n"
    "  G4 P                      -- Pause sequence; 'resume' continues next step\n"
    "  M100 [R<n>] [L<0|1>] [UL<upper>] [LL<lower>]  -- Stationary LC read\n"
    "  M101 F<f> [V<sps> C<creep> E<eps> T<ms> P<Kp> I<Ki> D<Kd> S<±1> X<max_steps>]\n"
    "                            -- Seek target force (PI+D velocity loop)\n"
    "  M200 D<deg>               -- Servo move to degrees (0-180) as a sequence step\n"
    "  M200 S<A-D>               -- Servo move to named position SA..SD as a step\n"
    "  M202 B<1|2> E<0|1> [W<0|1>] [T<ms>]  -- Break-beam check/wait step\n"
    "    B<1|2>    -- which sensor (default 1)\n"
    "    E0        -- expect beam CLEAR (not broken)\n"
    "    E1        -- expect beam BROKEN\n"
    "    W0        -- check mode: pause if state wrong (default)\n"
    "    W1        -- wait mode: block until state matches (or timeout)\n"
    "    T<ms>     -- timeout for wait mode; 0=infinite (default)\n"
    "  Step labels & conditional goto (any step type):\n"
    "    N<n>      -- attach label n (1-255) to this step\n"
    "    J<n>      -- on failure jump to step labeled n instead of pausing\n"
    "    Example:  M100 R4 L1 UL500 LL-50 J3   (goto label 3 if limit tripped)\n"
    "              G0 XA N3                     (label 3 is this step)\n"
    "  upload seq   -- Paste sequence; send END to finish (auto-saves)\n"
    "  clear seq | list seq\n"
    "\nSETUP MODE — settings:\n"
    "  set speed <sps>           -- Default move speed\n"
    "  set accel <sps2>          -- Default acceleration\n"
    "  set steps_rev <n>         -- Steps per revolution\n"
    "  set lc_upper <val>        -- LC upper limit\n"
    "  set lc_lower <val>        -- LC lower limit\n"
    "  set lc_limits <0|1>       -- Enable/disable limit checking\n"
    "  set knownweight <val>     -- Default weight for 'cal weight'\n"
    "  set lc_motion_samples <n> -- Default samples for in-motion reads\n"
    "  set lc_motion_rate <ms>   -- Requested RT; real dt >= samples*HX711_period (~100ms@10SPS)\n"
    "  set lc_ring <0|1>         -- Buffer [LIVE] samples; flush with flush lc / auto\n"
    "  set lc_ring_thresh <n>    -- Auto-flush when buffer has >= n samples\n"
    "  set lc_sampler_ms <n>     -- Background LC samples every n ms (0=off); use lc_ring\n"
    "  set seek_kp|ki|kd <v>     -- Default M101 / seek force PID\n"
    "  set seek_vmax|vcreep <sps>  set seek_epsilon <f>  set seek_settle_ms <n>\n"
    "  set seek_dir <±1>         -- +1 if +steps increase measured force\n"
    "  set link_proto <n>        -- LINK_UART protocol version (display ESP32)\n"
    "  set servo_min <us>        -- Servo pulse width at 0 deg (e.g. 500)\n"
    "  set servo_max <us>        -- Servo pulse width at 180 deg (e.g. 2500)\n"
    "  set break1_active_low <0|1>  -- Break-beam 1 active level (1 = LOW when broken)\n"
    "  set break2_active_low <0|1>  -- Break-beam 2 active level\n"
    "  set break1_pullup <0|1>   -- Break-beam 1 INPUT_PULLUP (only if pin supports it)\n"
    "  set break2_pullup <0|1>   -- Break-beam 2 INPUT_PULLUP\n"
    "\nADJUSTMENTS MODE:\n"
    "  jog <steps>               -- Jog (+ or -); prints position live\n"
    "  goto <steps|XA..XD>\n"
    "  setpos home                   -- Set current position as step 0\n"
    "  setpos XA [<steps>]       -- Save position as named variable\n"
    "  listpos\n"
    "  tare | cal weight <val> | cal clear | cal preset | read lc\n"
    "  seek force <f> [vmax] [creep]  -- PI+D seek (uses set seek_* defaults)\n"
    "  seek stop | flush lc\n"
    "  cal upload <idx>          -- Bulk cal entry from slot <idx>; lines: <raw>;<weight>; END\n"
    "    e.g. cal upload 0  then  42484.60;0.00  then  283636.41;9.75  then  END\n"
    "    Use cal upload 5 to append from slot 5, leaving slots 0-4 unchanged\n"
    "  servo <deg>               -- Move servo to angle (0..180)\n"
    "  servo us <us>             -- Move servo to raw pulse width (calibration)\n"
    "  servo set S<A-D> [<deg>]  -- Save current/given angle into slot (SA..SD)\n"
    "  servo goto S<A-D>         -- Move to named slot\n"
    "  servo list                -- List servo slots\n"
    "  servo off                 -- Stop emitting pulses (silence servo)\n"
    "  read bb | read bb1 | read bb2  -- Read break-beam sensors\n"
    "\nLIVE TEST MODE:\n"
    "  start <n>                 -- Run n cycles (LC per G0 step definitions)\n"
    "  start <n> LC every <k>   -- Full LC log every k-th cycle; others check limits only\n"
    "  start <n> LC cycles <c1,c2,...>  -- Full LC log on listed cycles only\n"
    "  pause | resume | stop | status\n"
    "==================================\n"
  ));
}

void printStatus() {
  lockCfg();
  Serial.println(F("---- STATUS ----"));
  Serial.printf("Mode: %-14s  State: %s\n", modeStr(currentMode), stateStr(runState));
  Serial.printf("Motor pos: %ld  steps/rev: %ld\n", stepper.currentPosition(), cfg.stepsPerRev);
  Serial.printf("Speed: %.0f sps   Accel: %.0f sps^2\n", cfg.maxSpeed_sps, cfg.accel_sps2);
  Serial.printf("Cycles: %d / %d   Seq steps: %d\n", cyclesDone, cyclesGoal, numSteps);
  Serial.printf("LC limits: %s  [%.3f .. %.3f]\n",
                cfg.lc_limitsOn ? "ON" : "off", cfg.lc_lowerLimit, cfg.lc_upperLimit);
  Serial.printf("LC in-motion defaults: R=%d  RT=%lu ms\n",
                cfg.lc_inMotionSamples, (unsigned long)cfg.lc_inMotionRateMs);
  Serial.printf("LC ring: %s  thresh=%u  sampler=%u ms\n",
                cfg.lc_ring_enable ? "ON" : "off",
                (unsigned)cfg.lc_ring_flush_thresh, (unsigned)cfg.lc_sampler_period_ms);
  Serial.printf("Seek defaults: Kp=%.2f Ki=%.2f Kd=%.4f Vmax=%.0f creep=%.0f eps=%.4f T=%lums dir=%d\n",
                cfg.seek_kp, cfg.seek_ki, cfg.seek_kd, cfg.seek_vmax, cfg.seek_vcreep,
                cfg.seek_epsilon, (unsigned long)cfg.seek_settle_ms, (int)cfg.seek_dir);
  Serial.printf("LINK proto version: %u\n", (unsigned)cfg.link_proto_version);

  // Profile mode
  switch (profileMode) {
    case ProfileMode::ALL:
      Serial.println("LC profile: all cycles");
      break;
    case ProfileMode::EVERY_N:
      Serial.printf("LC profile: every %d cycle(s)\n", profileEveryN);
      break;
    case ProfileMode::EXPLICIT: {
      Serial.print("LC profile: cycles ");
      for (int i = 0; i < profileCycleCount; i++) {
        if (i) Serial.print(',');
        Serial.print(profileCycles[i]);
      }
      Serial.println();
      break;
    }
  }

  Serial.println("Named positions:");
  for (int i = 0; i < NUM_NAMED_POS; i++) {
    if (cfg.namedPosSet[i])
      Serial.printf("  X%c = %ld\n", 'A'+i, cfg.namedPositions[i]);
    else
      Serial.printf("  X%c = (unset)\n", 'A'+i);
  }

  Serial.printf("Servo: pin=%d  cur=%.1f deg  pulse=%u us  range=%u..%u us\n",
                SERVO_PIN, cfg.servoCurrentDeg, g_servo.lastUs,
                cfg.servoMinPulseUs, cfg.servoMaxPulseUs);
  for (int i = 0; i < NUM_NAMED_SERVO_POS; i++) {
    if (cfg.servoNamedPosSet[i])
      Serial.printf("  S%c = %.1f deg\n", 'A'+i, cfg.servoNamedPositions[i]);
    else
      Serial.printf("  S%c = (unset)\n", 'A'+i);
  }
  Serial.printf("Break-beam: pin1=%d (%s)  pin2=%d (%s)\n",
                BREAK1_PIN, g_break1.isBroken() ? "BROKEN" : "clear",
                BREAK2_PIN, g_break2.isBroken() ? "BROKEN" : "clear");
  unlockCfg();
}

void printSequence() {
  lockSeq();
  Serial.printf("[SEQ] %d step(s):\n", numSteps);
  for (int i = 0; i < numSteps; i++) {
    const TestStep &s = testSequence[i];
    // Print label prefix if set
    if (s.stepLabel) Serial.printf("  #%d", s.stepLabel);
    else             Serial.print("  ");
    switch (s.type) {
      case StepType::MOVE: {
        char xbuf[12];
        if (s.useNamedPos) snprintf(xbuf, sizeof(xbuf), "X%c", 'A'+s.posVarIdx);
        else               snprintf(xbuf, sizeof(xbuf), "%ld", s.targetSteps);
        Serial.printf("[%02d] MOVE %s  F%.0f  A%.0f", i, xbuf, s.speed, s.accel);
        if (s.lcInMotion) {
          Serial.printf("  LC R=%d", s.lcSamples ? s.lcSamples : cfg.lc_inMotionSamples);
          if (s.lcRateMs)   Serial.printf(" RT=%lu", (unsigned long)s.lcRateMs);
          if (s.lcPosCount) {
            Serial.print(" P=");
            for (int j = 0; j < s.lcPosCount; j++) {
              if (j) Serial.print(',');
              Serial.print(s.lcPositions[j]);
            }
          }
          if (s.lcCheckLimits) Serial.print(" L1");
        }
        if (s.servoWithMove) {
          if (s.servoMoveUseNamed) Serial.printf("  SN=%c", 'A'+s.servoMoveVarIdx);
          else                     Serial.printf("  SV=%.1f", s.servoMoveDeg);
        }
        if (s.gotoLabel) Serial.printf("  J=%d", s.gotoLabel);
        Serial.println();
        break;
      }
      case StepType::DWELL:
        Serial.printf("[%02d] DWELL %lu ms\n", i, (unsigned long)s.dwellMs);
        break;
      case StepType::PAUSE:
        Serial.printf("[%02d] PAUSE (wait for 'resume')\n", i);
        LINK_UART.printf("S paused\n");
        break;
      case StepType::READ_LC:
        Serial.printf("[%02d] READ_LC  R=%d  limits=%s", i, s.lcReadings,
                      s.checkLimits ? "ON" : "off");
        if (s.checkLimits)
          Serial.printf("  UL=%.3f  LL=%.3f", s.readStepUpperLimit, s.readStepLowerLimit);
        if (s.gotoLabel) Serial.printf("  J=%d", s.gotoLabel);
        Serial.println();
        break;
      case StepType::SEEK_FORCE:
        Serial.printf("[%02d] SEEK_FORCE F=%.4f V=%.0f C=%.0f E=%.4f T=%lu P=%.2f I=%.2f D=%.3f S=%d X=%ld\n",
                      i, s.sfTarget, s.sfMaxSps, s.sfCreepSps, s.sfEpsilon,
                      (unsigned long)s.sfSettleMs, s.sfKp, s.sfKi, s.sfKd, (int)s.sfDir, (long)s.sfMaxTravel);
        break;
      case StepType::SERVO_MOVE:
        if (s.servoUseNamed) Serial.printf("[%02d] SERVO_MOVE S%c\n", i, 'A'+s.servoVarIdx);
        else                 Serial.printf("[%02d] SERVO_MOVE %.1f deg\n", i, s.servoDeg);
        break;
      case StepType::WAIT_BB:
        Serial.printf("[%02d] WAIT_BB  sensor=%d  expect=%s  mode=%s", i,
                      s.bbSensor,
                      s.bbExpected ? "broken" : "clear",
                      s.bbWaitMode ? "wait" : "check");
        if (s.bbWaitMode && s.bbTimeoutMs)
          Serial.printf("  T=%lums", (unsigned long)s.bbTimeoutMs);
        if (s.gotoLabel) Serial.printf("  J=%d", s.gotoLabel);
        Serial.println();
        break;
    }
  }
  unlockSeq();
}

// ============================================================================
// G-CODE PARSER
// ============================================================================
/*
 * G0 / G1  X<n|XA..XD>  [F<n>]  [A<n>]  [LC [R<n>] [RT<n>] [P<n,n,...>] [L<n>]]
 * G4       P<n>
 * M100     [R<n>] [L<n>]
 *
 * Parsing approach for in-motion LC on G0:
 *   1. Look for "LC" keyword in the (uppercased) line
 *   2. If present, extract R, RT, P, L parameters that follow it
 *   3. P<...> is a comma-separated list of absolute step positions
 *
 * Note: 'P' is also used by G4 for dwell time, but G4 and G0 are handled
 * separately so there's no ambiguity.
 */
void parseGCode(const String &rawLine) {
  String line = rawLine;
  line.trim();
  line.toUpperCase();
  if (line.length() == 0) return;

  if (numSteps >= MAX_TEST_STEPS) {
    Serial.println(F("[SEQ] ERROR: sequence full."));
    return;
  }

  // Generic single-letter float extractor (returns defaultVal if letter absent)
  auto getParam = [&](char letter, float defaultVal) -> float {
    int idx = line.indexOf(letter);
    if (idx < 0) return defaultVal;
    return line.substring(idx + 1).toFloat();
  };

  // Two-letter parameter extractor (RT, etc.)
  auto get2Param = [&](const char* key, float defaultVal) -> float {
    int idx = line.indexOf(key);
    if (idx < 0) return defaultVal;
    return line.substring(idx + strlen(key)).toFloat();
  };

  TestStep step = {};

  // ---- G0 / G1 : Move ----
  if (line.startsWith("G0") || line.startsWith("G1")) {
    step.type  = StepType::MOVE;
    step.speed = getParam('F', 0.0f);
    step.accel = getParam('A', 0.0f);

    // X parameter
    int xIdx = line.indexOf('X');
    if (xIdx < 0) { Serial.println(F("[G0] Missing X. Example: G0 X4000 F1200 A8000")); return; }
    char afterX = line.charAt(xIdx + 1);
    int  varIdx = namedPosIdx(afterX);
    if (varIdx >= 0) {
      step.useNamedPos = true;
      step.posVarIdx   = (uint8_t)varIdx;
    } else {
      step.useNamedPos = false;
      step.targetSteps = line.substring(xIdx + 1).toInt();
    }

    // In-motion LC: look for "LC" keyword
    int lcIdx = line.indexOf(" LC");
    if (lcIdx >= 0) {
      step.lcInMotion = true;

      // R<n> — samples (space+R so "RT" is not mistaken for R)
      String lcPart = line.substring(lcIdx);
      int rSp = lcPart.indexOf(" R");
      step.lcSamples = (rSp >= 0) ? (uint8_t)lcPart.substring(rSp + 2).toInt() : 0;

      // RT<n> — rate interval ms
      int rtIdx = lcPart.indexOf("RT");
      step.lcRateMs = (rtIdx >= 0) ? (uint32_t)lcPart.substring(rtIdx + 2).toFloat() : 0;

      // L<n> — limit check
      int lIdx = lcPart.indexOf('L');
      step.lcCheckLimits = (lIdx >= 0) && (lcPart.charAt(lIdx + 1) == '1');
      
      // After parsing L1 in the LC section:
      int ulIdx = lcPart.indexOf("UL");
      step.lcStepUpperLimit = (ulIdx >= 0) ? lcPart.substring(ulIdx + 2).toFloat() : cfg.lc_upperLimit;
      int llIdx = lcPart.indexOf("LL");
      step.lcStepLowerLimit = (llIdx >= 0) ? lcPart.substring(llIdx + 2).toFloat() : cfg.lc_lowerLimit;

      // P<s1,s2,...> — position list
      int pIdx = lcPart.indexOf('P');
      step.lcPosCount = 0;
      if (pIdx >= 0) {
        String posStr = lcPart.substring(pIdx + 1);
        // Read up to the next space or end of string
        int endIdx = posStr.indexOf(' ');
        if (endIdx > 0) posStr = posStr.substring(0, endIdx);
        // Parse comma-separated values
        int start = 0;
        while (start < (int)posStr.length() && step.lcPosCount < MAX_LC_POSITIONS) {
          int comma = posStr.indexOf(',', start);
          String token = (comma < 0) ? posStr.substring(start)
                                     : posStr.substring(start, comma);
          token.trim();
          if (token.length() > 0) {
            step.lcPositions[step.lcPosCount++] = token.toInt();
          }
          if (comma < 0) break;
          start = comma + 1;
        }
      }

      if (!step.lcRateMs && !step.lcPosCount) {
        // LC keyword present but no RT or P — apply the config default rate
        step.lcRateMs = cfg.lc_inMotionRateMs;
      }
    }

    // Concurrent servo: SN<A..D> = named pos, SV<deg> = degree value
    {
      int snIdx = line.indexOf("SN");
      if (snIdx >= 0) {
        int svar = servoNamedIdx(line.charAt(snIdx + 2));
        if (svar >= 0) {
          step.servoWithMove      = true;
          step.servoMoveUseNamed  = true;
          step.servoMoveVarIdx    = (uint8_t)svar;
        }
      }
      if (!step.servoWithMove) {
        float svDeg = get2Param("SV", -1.0f);
        if (svDeg >= 0.f && svDeg <= 180.f) {
          step.servoWithMove     = true;
          step.servoMoveUseNamed = false;
          step.servoMoveDeg      = svDeg;
        }
      }
    }

    // Step label (N) and goto-on-fail label (J) — shared by all step types
    step.stepLabel = (uint8_t)getParam('N', 0.f);
    step.gotoLabel = (uint8_t)getParam('J', 0.f);

    lockSeq();
    testSequence[numSteps++] = step;
    unlockSeq();

    // Confirmation print
    char xbuf[12];
    if (step.useNamedPos) snprintf(xbuf, sizeof(xbuf), "X%c", 'A'+step.posVarIdx);
    else                  snprintf(xbuf, sizeof(xbuf), "%ld", step.targetSteps);
    Serial.printf("[SEQ] Step %d: MOVE %s  F%.0f  A%.0f", numSteps-1, xbuf, step.speed, step.accel);
    if (step.lcInMotion) {
      Serial.printf("  LC R=%d", step.lcSamples ? step.lcSamples : cfg.lc_inMotionSamples);
      if (step.lcRateMs)   Serial.printf(" RT=%lu", (unsigned long)step.lcRateMs);
      if (step.lcPosCount) { Serial.printf(" P[%d]", step.lcPosCount); }
      if (step.lcCheckLimits) Serial.print(" L1");
    }
    if (step.servoWithMove) {
      if (step.servoMoveUseNamed) Serial.printf("  SN=%c", 'A'+step.servoMoveVarIdx);
      else                        Serial.printf("  SV=%.1f", step.servoMoveDeg);
    }
    if (step.stepLabel) Serial.printf("  N=%d", step.stepLabel);
    if (step.gotoLabel) Serial.printf("  J=%d", step.gotoLabel);
    Serial.println();
    return;
  }

  // ---- G4 : Dwell / Pause ----
  //   G4 P<ms>  -> timed dwell
  //   G4 P      -> pause here; resume with 'resume' command (advances to next step)
  if (line.startsWith("G4")) {
    int pIdx = line.indexOf('P');
    bool hasNumericP = false;
    if (pIdx >= 0) {
      for (int i = pIdx + 1; i < (int)line.length(); i++) {
        char c = line.charAt(i);
        if (c == ' ' || c == '\t') continue;
        if (isDigit((unsigned char)c) || c == '-' || c == '+' || c == '.') hasNumericP = true;
        break;
      }
    }
    if (pIdx >= 0 && !hasNumericP) {
      step.type      = StepType::PAUSE;
      step.stepLabel = (uint8_t)getParam('N', 0.f);
      step.gotoLabel = (uint8_t)getParam('J', 0.f);
      lockSeq();
      testSequence[numSteps++] = step;
      unlockSeq();
      Serial.printf("[SEQ] Step %d: PAUSE (wait for 'resume')", numSteps-1);
      if (step.stepLabel) Serial.printf("  N=%d", step.stepLabel);
      Serial.println();
      return;
    }
    step.type      = StepType::DWELL;
    step.dwellMs   = (uint32_t)getParam('P', 1000.0f);
    step.stepLabel = (uint8_t)getParam('N', 0.f);
    step.gotoLabel = (uint8_t)getParam('J', 0.f);
    lockSeq();
    testSequence[numSteps++] = step;
    unlockSeq();
    Serial.printf("[SEQ] Step %d: DWELL %lu ms", numSteps-1, (unsigned long)step.dwellMs);
    if (step.stepLabel) Serial.printf("  N=%d", step.stepLabel);
    Serial.println();
    return;
  }

  // ---- M100 : Stationary LC read ----
  // M100 [R<n>] [L<0|1>] [UL<upper>] [LL<lower>] [N<label>] [J<goto_label>]
  if (line.startsWith("M100")) {
    step.type        = StepType::READ_LC;
    step.lcReadings  = max((uint8_t)1, (uint8_t)getParam('R', 5.0f));
    step.checkLimits = ((int)getParam('L', 0.0f)) != 0;
    lockCfg();
    step.readStepUpperLimit = get2Param("UL", cfg.lc_upperLimit);
    step.readStepLowerLimit = get2Param("LL", cfg.lc_lowerLimit);
    unlockCfg();
    step.stepLabel = (uint8_t)getParam('N', 0.f);
    step.gotoLabel = (uint8_t)getParam('J', 0.f);
    lockSeq();
    testSequence[numSteps++] = step;
    unlockSeq();
    Serial.printf("[SEQ] Step %d: READ_LC  R=%d  limits=%s", numSteps-1,
                  step.lcReadings, step.checkLimits ? "ON" : "off");
    if (step.checkLimits)
      Serial.printf("  UL=%.3f  LL=%.3f", step.readStepUpperLimit, step.readStepLowerLimit);
    if (step.stepLabel) Serial.printf("  N=%d", step.stepLabel);
    if (step.gotoLabel) Serial.printf("  J=%d", step.gotoLabel);
    Serial.println();
    return;
  }

  // ---- M101 : Seek force (PI+D velocity loop) ----
  // M101 F<target> V<max_sps> C<creep_sps> E<epsilon> T<settle_ms> P<Kp> I<Ki> D<Kd> S<dir> X<max_travel_steps>
  if (line.startsWith("M101")) {
    step.type = StepType::SEEK_FORCE;
    int fIdx = line.indexOf('F');
    if (fIdx < 0 || fIdx >= (int)line.length() - 1) {
      Serial.println(F("[M101] Missing F (target force). Example: M101 F12.5 V600 C40"));
      return;
    }
    auto gp = [&](char letter, float defVal) -> float {
      int i = line.indexOf(letter);
      if (i < 0) return defVal;
      return line.substring(i + 1).toFloat();
    };
    lockCfg();
    step.sfTarget    = line.substring(fIdx + 1).toFloat();
    step.sfMaxSps    = gp('V', cfg.seek_vmax);
    step.sfCreepSps  = gp('C', cfg.seek_vcreep);
    step.sfEpsilon   = gp('E', cfg.seek_epsilon);
    step.sfSettleMs  = (uint32_t)gp('T', (float)cfg.seek_settle_ms);
    step.sfKp        = gp('P', cfg.seek_kp);
    step.sfKi        = gp('I', cfg.seek_ki);
    step.sfKd        = gp('D', cfg.seek_kd);
    int sgn          = (int)gp('S', (float)cfg.seek_dir);
    step.sfDir       = (int8_t)(sgn >= 0 ? 1 : -1);
    step.sfMaxTravel = (long)gp('X', 0.f);
    step.stepLabel   = (uint8_t)gp('N', 0.f);
    step.gotoLabel   = (uint8_t)gp('J', 0.f);
    unlockCfg();
    if (step.sfCreepSps < 1.f) step.sfCreepSps = 1.f;
    if (step.sfMaxSps < step.sfCreepSps) step.sfMaxSps = step.sfCreepSps;
    lockSeq();
    testSequence[numSteps++] = step;
    unlockSeq();
    Serial.printf("[SEQ] Step %d: SEEK_FORCE F=%.4f V=%.0f C=%.0f E=%.4f T=%lu P=%.3f I=%.3f D=%.4f S=%d X=%ld\n",
                  numSteps - 1, step.sfTarget, step.sfMaxSps, step.sfCreepSps, step.sfEpsilon,
                  (unsigned long)step.sfSettleMs, step.sfKp, step.sfKi, step.sfKd, (int)step.sfDir,
                  (long)step.sfMaxTravel);
    return;
  }

  // ---- M200 : Servo move (sequence step) ----
  // M200 S<A..D>         -- move to named servo position
  // M200 D<deg>          -- move to degree value (0..180)
  // Optional: N<label>  J<goto_label>  (labels have no effect on SERVO_MOVE failures)
  if (line.startsWith("M200")) {
    step.type = StepType::SERVO_MOVE;

    // Check for named servo position: S<A..D>
    int sIdx = line.indexOf(" S");
    if (sIdx >= 0) {
      int svar = servoNamedIdx(line.charAt(sIdx + 2));
      if (svar >= 0) {
        step.servoUseNamed = true;
        step.servoVarIdx   = (uint8_t)svar;
      }
    }
    if (!step.servoUseNamed) {
      step.servoDeg = getParam('D', -1.0f);
      if (step.servoDeg < 0.f || step.servoDeg > 180.f) {
        Serial.println(F("[M200] Provide S<A-D> or D<deg 0-180>. Example: M200 D45  or  M200 SA"));
        return;
      }
    }
    step.stepLabel = (uint8_t)getParam('N', 0.f);
    step.gotoLabel = (uint8_t)getParam('J', 0.f);
    lockSeq();
    testSequence[numSteps++] = step;
    unlockSeq();
    if (step.servoUseNamed)
      Serial.printf("[SEQ] Step %d: SERVO_MOVE S%c", numSteps-1, 'A'+step.servoVarIdx);
    else
      Serial.printf("[SEQ] Step %d: SERVO_MOVE %.1f deg", numSteps-1, step.servoDeg);
    if (step.stepLabel) Serial.printf("  N=%d", step.stepLabel);
    Serial.println();
    return;
  }

  // ---- M202 : Wait / check break-beam ----
  // M202 B<1|2> E<0|1> [W<0|1>] [T<ms>] [N<label>] [J<goto_label>]
  //   B = sensor number (1 or 2, default 1)
  //   E = expected state: 0=clear (beam intact), 1=broken
  //   W = mode: 0=pause-if-wrong (default), 1=wait-until-right
  //   T = timeout ms in wait mode (0=infinite)
  //   J = on failure (wrong state or timeout) jump to step labeled J instead of pausing
  if (line.startsWith("M202")) {
    step.type         = StepType::WAIT_BB;
    step.bbSensor     = (uint8_t)constrain((int)getParam('B', 1.f), 1, 2);
    step.bbExpected   = ((int)getParam('E', 0.f)) != 0;
    step.bbWaitMode   = ((int)getParam('W', 0.f)) != 0;
    step.bbTimeoutMs  = (uint32_t)getParam('T', 0.f);
    step.stepLabel    = (uint8_t)getParam('N', 0.f);
    step.gotoLabel    = (uint8_t)getParam('J', 0.f);
    lockSeq();
    testSequence[numSteps++] = step;
    unlockSeq();
    Serial.printf("[SEQ] Step %d: WAIT_BB  sensor=%d  expect=%s  mode=%s",
                  numSteps-1, step.bbSensor,
                  step.bbExpected ? "broken" : "clear",
                  step.bbWaitMode ? "wait" : "check");
    if (step.bbWaitMode && step.bbTimeoutMs)
      Serial.printf("  T=%lums", (unsigned long)step.bbTimeoutMs);
    if (step.stepLabel) Serial.printf("  N=%d", step.stepLabel);
    if (step.gotoLabel) Serial.printf("  J=%d", step.gotoLabel);
    Serial.println();
    return;
  }

  Serial.printf("[G-code] Unknown: %s\n", line.c_str());
}

// ============================================================================
// COMMAND HANDLER — sub-handlers for servo & break-beam
// ============================================================================

/*
 * handleServoCommand — ADJUSTMENTS-mode 'servo ...' commands.
 *
 *   servo <deg>            move directly to angle (0..180)
 *   servo us <us>          move to raw pulse width (handy for calibration)
 *   servo off              stop emitting pulses (pin idle LOW)
 *   servo set S<A..D> [<deg>]  save current/given angle into slot
 *   servo goto S<A..D>     move to named slot
 *   servo list             list slots
 *
 * `args` is the substring AFTER "servo " with surrounding whitespace trimmed
 * and converted to lower-case (matching the rest of the command dispatch).
 */
void handleServoCommand(const String &args) {
  if (args.length() == 0) {
    Serial.println(F("Usage: servo <deg> | servo us <us> | servo set S<A-D> [<deg>] |"));
    Serial.println(F("       servo goto S<A-D> | servo list | servo off"));
    return;
  }

  // servo off
  if (args == "off") {
    g_servo.setEnabled(false);
    Serial.println(F("[SERVO] Pulses disabled."));
    return;
  }

  // servo list
  if (args == "list") {
    lockCfg();
    Serial.printf("[SERVO] Current: %.1f deg  (pulse=%u us, range=%u..%u us)\n",
                  cfg.servoCurrentDeg, g_servo.lastUs,
                  cfg.servoMinPulseUs, cfg.servoMaxPulseUs);
    for (int i = 0; i < NUM_NAMED_SERVO_POS; i++) {
      if (cfg.servoNamedPosSet[i])
        Serial.printf("  S%c = %.1f deg\n", 'A'+i, cfg.servoNamedPositions[i]);
      else
        Serial.printf("  S%c = (unset)\n", 'A'+i);
    }
    unlockCfg();
    return;
  }

  // servo us <us>
  if (args.startsWith("us")) {
    String v = args.substring(2); v.trim();
    if (v.length() == 0) { Serial.println(F("Usage: servo us <microseconds>")); return; }
    long us = v.toInt();
    if (us < 100 || us > 4000) { Serial.println(F("[SERVO] us must be 100..4000")); return; }
    servo_writeMicros((uint16_t)us);
    Serial.printf("[SERVO] pulse=%ld us\n", us);
    return;
  }

  // servo goto S<A..D>
  if (args.startsWith("goto ")) {
    String v = args.substring(5); v.trim();
    if (v.length() < 2 || v.charAt(0) != 's') { Serial.println(F("Usage: servo goto S<A-D>")); return; }
    int idx = servoNamedIdx(v.charAt(1));
    if (idx < 0) { Serial.println(F("[SERVO] Use SA..SD.")); return; }
    lockCfg();
    bool  isSet = cfg.servoNamedPosSet[idx];
    float deg   = cfg.servoNamedPositions[idx];
    unlockCfg();
    if (!isSet) { Serial.printf("[SERVO] S%c not set.\n", 'A'+idx); return; }
    servo_writeDegrees(deg);
    Serial.printf("[SERVO] -> S%c = %.1f deg\n", 'A'+idx, deg);
    return;
  }

  // servo set S<A..D> [<deg>]
  if (args.startsWith("set ")) {
    String v = args.substring(4); v.trim();
    if (v.length() < 2 || v.charAt(0) != 's') {
      Serial.println(F("Usage: servo set S<A-D> [<deg>]")); return;
    }
    int idx = servoNamedIdx(v.charAt(1));
    if (idx < 0) { Serial.println(F("[SERVO] Use SA..SD.")); return; }
    String rest = v.substring(2); rest.trim();
    float deg;
    if (rest.length() > 0) {
      deg = rest.toFloat();
      if (deg < 0.f || deg > 180.f) { Serial.println(F("[SERVO] deg must be 0..180")); return; }
    } else {
      lockCfg();
      deg = cfg.servoCurrentDeg;
      unlockCfg();
    }
    lockCfg();
    cfg.servoNamedPositions[idx] = deg;
    cfg.servoNamedPosSet  [idx] = true;
    unlockCfg();
    Serial.printf("[SERVO] S%c = %.1f deg\n", 'A'+idx, deg);
    return;
  }

  // Bare number -> degrees
  // First char must be a digit / sign / dot, otherwise treat as unknown.
  char c0 = args.charAt(0);
  if (isDigit((unsigned char)c0) || c0 == '+' || c0 == '-' || c0 == '.') {
    float deg = args.toFloat();
    if (deg < 0.f || deg > 180.f) { Serial.println(F("[SERVO] deg must be 0..180")); return; }
    servo_writeDegrees(deg);
    Serial.printf("[SERVO] -> %.1f deg (pulse=%u us)\n", deg, g_servo.lastUs);
    return;
  }

  Serial.println(F("Unknown 'servo' sub-command. Type 'help'."));
}

/*
 * handleReadBreakCommand — ADJUSTMENTS-mode 'read bb...' commands.
 *
 *   read bb       both sensors
 *   read bb1      sensor 1 only
 *   read bb2      sensor 2 only
 */
void handleReadBreakCommand(const String &args) {
  auto reportOne = [](int which, const BreakBeamSensor &s) {
    int lvl = s.rawLevel();
    bool broken = s.isBroken();
    Serial.printf("[BB%d] pin=%d  raw=%d  %s%s\n",
                  which, s.pin, lvl,
                  broken ? "BROKEN" : "CLEAR",
                  s.usePullup ? "  (pullup)" : "");
    
  };

  if (args.length() == 0 || args == "bb") {
    reportOne(1, g_break1);
    reportOne(2, g_break2);
    LINK_UART.printf("B s1=%d s2=%d\n",
                     g_break1.isBroken() ? 1 : 0,
                     g_break2.isBroken() ? 1 : 0);
    return;
  }
  if (args == "bb1") { reportOne(1, g_break1); return; }
  if (args == "bb2") { reportOne(2, g_break2); return; }
  Serial.println(F("Usage: read bb | read bb1 | read bb2"));
}

// ============================================================================
// COMMAND HANDLER
// ============================================================================

void handleCommand(const String &rawLine) {
  String s = rawLine;
  s.trim();
  if (s.length() == 0) return;
  String sl = s; sl.toLowerCase();

  // ------------------------------------------------------------------
  // UPLOAD MODE — sequence
  // ------------------------------------------------------------------
  if (inUploadMode) {
    if (sl == "end") {
      inUploadMode = false;
      Serial.printf("[UPLOAD] Done. %d step(s) received.\n", numSteps);
      if (saveSequence()) Serial.println(F("[UPLOAD] Sequence saved to NVS."));
      else                Serial.println(F("[UPLOAD] WARNING: NVS save failed."));
      printSequence();
    } else {
      parseGCode(s);
    }
    return;
  }

  // ------------------------------------------------------------------
  // CAL UPLOAD MODE — bulk calibration point entry
  // Each line: <raw>;<weight>   e.g.  42484.60;0.00
  // Send END to finish. Slot index advances automatically from the
  // starting index given in "cal upload <idx>".
  // ------------------------------------------------------------------
  if (inCalUploadMode) {
    if (sl == "end") {
      inCalUploadMode = false;
      lc_applyCalibration();
      Serial.println(F("[CAL UPLOAD] Done. Calibration applied."));
      lc_printMatrix();
    } else {
      // Parse "raw;weight"
      int sep = s.indexOf(';');
      if (sep < 0) {
        Serial.println(F("[CAL UPLOAD] Bad format. Use: <raw>;<weight>  e.g. 42484.60;0.00"));
        return;
      }
      float raw    = s.substring(0, sep).toFloat();
      float weight = s.substring(sep + 1).toFloat();
      if (calUploadSlot >= LC_INDEX_SIZE) {
        Serial.println(F("[CAL UPLOAD] Matrix full! Send END."));
        return;
      }
      lockCfg();
      cfg.lc_calRaw[calUploadSlot]    = raw;
      cfg.lc_calWeight[calUploadSlot] = weight;
      cfg.lc_calUsed[calUploadSlot]   = true;
      unlockCfg();
      Serial.printf("[CAL UPLOAD] Slot %d: raw=%.2f  weight=%.3f\n", calUploadSlot, raw, weight);
      calUploadSlot++;
    }
    return;
  }

  // ------------------------------------------------------------------
  // GLOBAL commands
  // ------------------------------------------------------------------
  if (sl == "help")   { printHelp();   return; }
  if (sl == "status") { printStatus(); return; }
  if (sl == "flush lc") { lc_ring_flush(); return; }

  if (sl.startsWith("mode ")) {
    String m = sl.substring(5); m.trim();
    if      (m == "setup")                     { currentMode = Mode::SETUP;       runState = RunState::IDLE; Serial.println(F("Mode -> SETUP")); }
    else if (m == "adj" || m == "adjustments") { currentMode = Mode::ADJUSTMENTS; runState = RunState::IDLE; Serial.println(F("Mode -> ADJUSTMENTS")); }
    else if (m == "live" || m == "live_test")  { currentMode = Mode::LIVE_TEST;   Serial.println(F("Mode -> LIVE_TEST")); }
    else Serial.println(F("Unknown mode. Options: setup | adj | live"));
    return;
  }

  if (sl == "save") {
    bool c = saveConfig(), sq = saveSequence();
    Serial.printf("Config %s | Sequence %s\n", c?"saved":"FAILED", sq?"saved":"FAILED");
    return;
  }
  if (sl == "load") {
    bool c = loadConfig(), sq = loadSequence();
    Serial.printf("Config %s | Sequence %s\n", c?"loaded":"not found", sq?"loaded":"not found");
    if (sq) printSequence();
    return;
  }
  if (sl == "wipe") {
    bool c = wipeConfig(), sq = wipeSequence();
    Serial.printf("Config %s | Sequence %s\n", c?"wiped":"FAILED", sq?"wiped":"FAILED");
    return;
  }
  if (sl == "save seq") { saveSequence() ? Serial.println(F("Sequence saved."))    : Serial.println(F("ERROR: save failed.")); return; }
  if (sl == "load seq") {
    if (loadSequence()) { Serial.println(F("Sequence loaded:")); printSequence(); }
    else                  Serial.println(F("No saved sequence found."));
    return;
  }
  if (sl == "wipe seq") { wipeSequence() ? Serial.println(F("Sequence wiped.")) : Serial.println(F("ERROR: wipe failed.")); return; }

  // ------------------------------------------------------------------
  // SETUP mode
  // ------------------------------------------------------------------
  if (currentMode == Mode::SETUP) {

    String su = s; su.toUpperCase();
    if (su.startsWith("G") || su.startsWith("M")) { parseGCode(s); return; }

    if (sl == "upload seq") {
      lockSeq(); numSteps = 0; currentStepIdx = 0; unlockSeq();
      inUploadMode = true;
      Serial.println(F("[UPLOAD] Ready. Paste G-code lines, then send 'END'."));
      return;
    }
    if (sl == "clear seq") {
      lockSeq(); numSteps = 0; currentStepIdx = 0; unlockSeq();
      Serial.println(F("Sequence cleared."));
      return;
    }
    if (sl == "list seq") { printSequence(); return; }

    if (sl.startsWith("set ")) {
      String rest = sl.substring(4);
      int sp = rest.indexOf(' ');
      if (sp < 0) { Serial.println(F("Bad 'set' syntax.")); return; }
      String key = rest.substring(0, sp);
      String val = rest.substring(sp + 1);
      lockCfg();
      if      (key == "speed")              { cfg.maxSpeed_sps       = val.toFloat(); stepper.setMaxSpeed(cfg.maxSpeed_sps);    Serial.printf("speed                -> %.0f sps\n", cfg.maxSpeed_sps); }
      else if (key == "accel")              { cfg.accel_sps2         = val.toFloat(); stepper.setAcceleration(cfg.accel_sps2); Serial.printf("accel                -> %.0f sps^2\n", cfg.accel_sps2); }
      else if (key == "steps_rev")          { cfg.stepsPerRev        = val.toInt();                                             Serial.printf("steps/rev            -> %ld\n", cfg.stepsPerRev); }
      else if (key == "lc_upper")           { cfg.lc_upperLimit      = val.toFloat();                                           Serial.printf("LC upper             -> %.3f\n", cfg.lc_upperLimit); }
      else if (key == "lc_lower")           { cfg.lc_lowerLimit      = val.toFloat();                                           Serial.printf("LC lower             -> %.3f\n", cfg.lc_lowerLimit); }
      else if (key == "lc_limits")          { cfg.lc_limitsOn        = val.toInt() != 0;                                       Serial.printf("LC limits            -> %s\n", cfg.lc_limitsOn?"ON":"OFF"); }
      else if (key == "knownweight")        { cfg.lc_knownWeight     = val.toFloat();                                           Serial.printf("knownWeight          -> %.3f\n", cfg.lc_knownWeight); }
      else if (key == "lc_motion_samples")  { cfg.lc_inMotionSamples = (uint8_t)val.toInt();                                   Serial.printf("lc_motion_samples    -> %d\n", cfg.lc_inMotionSamples); }
      else if (key == "lc_motion_rate")     { cfg.lc_inMotionRateMs  = (uint32_t)val.toInt();                                  Serial.printf("lc_motion_rate       -> %lu ms\n", (unsigned long)cfg.lc_inMotionRateMs); }
      else if (key == "lc_ring")            { cfg.lc_ring_enable = (val.toInt() != 0);                                         Serial.printf("lc_ring              -> %s\n", cfg.lc_ring_enable ? "ON" : "off"); }
      else if (key == "lc_ring_thresh")     { cfg.lc_ring_flush_thresh = (uint16_t)constrain(val.toInt(), 1, LC_RING_CAP - 1); Serial.printf("lc_ring_thresh       -> %u\n", (unsigned)cfg.lc_ring_flush_thresh); }
      else if (key == "lc_sampler_ms")      { cfg.lc_sampler_period_ms = (uint16_t)val.toInt();                               Serial.printf("lc_sampler_ms        -> %u (0=off)\n", (unsigned)cfg.lc_sampler_period_ms); }
      else if (key == "seek_kp")            { cfg.seek_kp = val.toFloat();                                                     Serial.printf("seek_kp              -> %.4f\n", cfg.seek_kp); }
      else if (key == "seek_ki")            { cfg.seek_ki = val.toFloat();                                                     Serial.printf("seek_ki              -> %.4f\n", cfg.seek_ki); }
      else if (key == "seek_kd")            { cfg.seek_kd = val.toFloat();                                                     Serial.printf("seek_kd              -> %.4f\n", cfg.seek_kd); }
      else if (key == "seek_vmax")          { cfg.seek_vmax = val.toFloat();                                                   Serial.printf("seek_vmax            -> %.1f sps\n", cfg.seek_vmax); }
      else if (key == "seek_vcreep")        { cfg.seek_vcreep = val.toFloat();                                                 Serial.printf("seek_vcreep          -> %.1f sps\n", cfg.seek_vcreep); }
      else if (key == "seek_epsilon")       { cfg.seek_epsilon = val.toFloat();                                                Serial.printf("seek_epsilon         -> %.4f\n", cfg.seek_epsilon); }
      else if (key == "seek_settle_ms")     { cfg.seek_settle_ms = (uint32_t)val.toInt();                                      Serial.printf("seek_settle_ms       -> %lu\n", (unsigned long)cfg.seek_settle_ms); }
      else if (key == "seek_dir")           { cfg.seek_dir = (int8_t)(val.toInt() >= 0 ? 1 : -1);                              Serial.printf("seek_dir             -> %d\n", (int)cfg.seek_dir); }
      else if (key == "link_proto")         { cfg.link_proto_version = (uint8_t)constrain(val.toInt(), 0, 255);               Serial.printf("link_proto           -> %u\n", (unsigned)cfg.link_proto_version); linkAnnounceProtocol(LINK_UART, cfg.link_proto_version); }
      else if (key == "servo_min")          { cfg.servoMinPulseUs = (uint16_t)constrain(val.toInt(), 100, 4000);              Serial.printf("servo_min            -> %u us\n", (unsigned)cfg.servoMinPulseUs); }
      else if (key == "servo_max")          { cfg.servoMaxPulseUs = (uint16_t)constrain(val.toInt(), 100, 4000);              Serial.printf("servo_max            -> %u us\n", (unsigned)cfg.servoMaxPulseUs); }
      else if (key == "break1_active_low")  { cfg.break1ActiveLow = (val.toInt() != 0); g_break1.activeLow = cfg.break1ActiveLow;  Serial.printf("break1_active_low    -> %d\n", (int)cfg.break1ActiveLow); }
      else if (key == "break2_active_low")  { cfg.break2ActiveLow = (val.toInt() != 0); g_break2.activeLow = cfg.break2ActiveLow;  Serial.printf("break2_active_low    -> %d\n", (int)cfg.break2ActiveLow); }
      else if (key == "break1_pullup")      { cfg.break1UsePullup = (val.toInt() != 0); g_break1.usePullup = cfg.break1UsePullup;  g_break1.begin();  Serial.printf("break1_pullup        -> %d\n", (int)cfg.break1UsePullup); }
      else if (key == "break2_pullup")      { cfg.break2UsePullup = (val.toInt() != 0); g_break2.usePullup = cfg.break2UsePullup;  g_break2.begin();  Serial.printf("break2_pullup        -> %d\n", (int)cfg.break2UsePullup); }
      else Serial.println(F("Unknown 'set' field."));
      unlockCfg();
      return;
    }

    Serial.println(F("Unknown SETUP command. Type 'help'."));
    return;
  }

  // ------------------------------------------------------------------
  // ADJUSTMENTS mode
  // ------------------------------------------------------------------
  if (currentMode == Mode::ADJUSTMENTS) {
    if (sl.startsWith("jog ")) {
      long delta = sl.substring(4).toInt();
      // Use moveTo(current + delta) so AccelStepper plans one smooth trapezoid.
      // If the motor is still decelerating from a previous jog, moveTo() replans
      // cleanly — no jerk from a mid-profile interrupt.
      long newTarget = stepper.currentPosition() + delta;
      stepper.moveTo(newTarget);
      jogMoveDone = false;
      Serial.printf("[ADJ] Jog %+ld -> target %ld\n", delta, newTarget);
      return;
    }
    if (sl.startsWith("goto ")) {
      String arg = sl.substring(5); arg.trim();
      if (arg.length() >= 2 && arg.charAt(0) == 'x') {
        int idx = namedPosIdx(arg.charAt(1));
        if (idx >= 0) {
          lockCfg();
          bool isSet = cfg.namedPosSet[idx]; long pos = cfg.namedPositions[idx];
          unlockCfg();
          if (isSet) { stepper.moveTo(pos); Serial.printf("[ADJ] -> X%c = %ld\n", 'A'+idx, pos); }
          else         Serial.printf("[ADJ] X%c not set.\n", 'A'+idx);
        } else Serial.println(F("[ADJ] Use XA..XD."));
      } else {
        long tgt = arg.toInt(); stepper.moveTo(tgt);
        Serial.printf("[ADJ] -> %ld\n", tgt);
      }
      return;
    }
    if (sl == "setpos home") {
      stepper.setCurrentPosition(0);
      Serial.println(F("[ADJ] Home set to 0"));
      LINK_UART.printf("P pos=%ld\n", 0L);  // P = Position update

      return;
    }
    if (sl.startsWith("setpos ")) {
      String args = sl.substring(7); args.trim();
      if (args.length() < 2 || args.charAt(0) != 'x') { Serial.println(F("Usage: setpos XA [<steps>]")); return; }
      int idx = namedPosIdx(args.charAt(1));
      if (idx < 0) { Serial.println(F("Use XA..XD.")); return; }
      long pos = (args.length() > 3) ? args.substring(3).toInt() : stepper.currentPosition();
      lockCfg(); cfg.namedPositions[idx] = pos; cfg.namedPosSet[idx] = true; unlockCfg();
      Serial.printf("[ADJ] X%c = %ld\n", 'A'+idx, pos);
      return;
    }
    if (sl == "listpos") {
      lockCfg();
      for (int i = 0; i < NUM_NAMED_POS; i++) {
        if (cfg.namedPosSet[i]) Serial.printf("  X%c = %ld\n", 'A'+i, cfg.namedPositions[i]);
        else                    Serial.printf("  X%c = (unset)\n", 'A'+i);
      }
      unlockCfg(); return;
    }
    if (sl == "tare")          { lc_tare(); return; }
    if (sl.startsWith("cal weight ")) { lc_addCalPoint(sl.substring(11).toFloat()); return; }
    if (sl == "cal weight")    { lc_addCalPoint(cfg.lc_knownWeight); return; }

    // cal upload <startIndex>  — bulk raw;weight entry, terminated by END
    // Example:  cal upload 0   then send lines like  42484.60;0.00
    // Use cal upload 5 to start from slot 5, leaving 0-4 untouched.
    if (sl.startsWith("cal upload")) {
      String idxStr = sl.substring(10); idxStr.trim();
      int startIdx = (idxStr.length() > 0) ? idxStr.toInt() : 0;
      if (startIdx < 0 || startIdx >= LC_INDEX_SIZE) {
        Serial.printf("[CAL UPLOAD] Invalid start index. Must be 0-%d.\n", LC_INDEX_SIZE - 1);
        return;
      }
      calUploadSlot   = startIdx;
      inCalUploadMode = true;
      Serial.printf("[CAL UPLOAD] Ready from slot %d. Send lines as: <raw>;<weight>  then END.\n", startIdx);
      return;
    }
    if (sl == "cal preset") {
      lockCfg(); lc_clearCalibration();
      //const float rP[] = {-679135.20f,-322304.55f,42789.00f,299100.41f,309657.00f,
      //                     348070.59f,417649.81f,490064.59f,586401.19f,715813.00f};
      //const float wP[] = {-30.00f,-15.00f,0.00f,10.30f,10.71f,12.27f,15.09f,18.00f,21.70f,26.80f};
      const float rP[] = {-699034.80f,-514919.2f,-333225.2f,-153952.8f,-65224.7f,
                           110415.3f,197327.2f,369334.8f,538920.8f,706085.2f};
      const float wP[] = {-40.00f,-30.00f,-20.00f,-10.00f,-5.00f,5.00f,10.00f,20.00f,30.00f,40.00f};
      for (int i = 0; i < LC_INDEX_SIZE; i++) { cfg.lc_calRaw[i]=rP[i]; cfg.lc_calWeight[i]=wP[i]; cfg.lc_calUsed[i]=true; }
      lc_applyCalibration(); unlockCfg(); lc_printMatrix(); return;
    }
    if (sl == "cal clear") { lockCfg(); lc_clearCalibration(); unlockCfg(); Serial.println(F("[LC] Cleared.")); return; }
    if (sl == "read lc") {
      float w = lc_read(5);
      Serial.printf("[LC] %.4f\n", w);
      LINK_UART.printf("R pos=%ld lc=%.4f\n", stepper.currentPosition(), w);  // R=Result
      return;
    }
    // read bb | read bb1 | read bb2
    if (sl.startsWith("read bb")) {
      String args = sl.substring(5); args.trim();   // "bb", "bb1", "bb2"
      handleReadBreakCommand(args);
      return;
    }
    // servo <deg> | servo us <us> | servo set ... | servo goto ... | servo list | servo off
    if (sl == "servo" || sl.startsWith("servo ") || sl.startsWith("servo\t")) {
      String args;
      if (sl.length() > 5) { args = sl.substring(5); args.trim(); }
      handleServoCommand(args);
      return;
    }
    if (sl == "seek stop") {
      g_adjSeekActive    = false;
      g_velocitySeekMode = false;
      g_velocityCommand  = 0.f;
      stepper.stop();
      memset(&g_seekRT, 0, sizeof g_seekRT);
      Serial.println(F("[ADJ] Seek stopped."));
      return;
    }
    if (sl.startsWith("seek force ")) {
      String rest = sl.substring(11);
      rest.trim();
      float tgt = rest.toFloat();
      lockCfg();
      float vmax = cfg.seek_vmax, creep = cfg.seek_vcreep;
      int sp = rest.indexOf(' ');
      if (sp > 0) {
        String r2 = rest.substring(sp + 1);
        r2.trim();
        sp = r2.indexOf(' ');
        vmax = r2.toFloat();
        if (sp > 0) creep = r2.substring(sp + 1).toFloat();
      }
      g_adjSeekStep = TestStep{};
      g_adjSeekStep.type        = StepType::SEEK_FORCE;
      g_adjSeekStep.sfTarget    = tgt;
      g_adjSeekStep.sfMaxSps    = vmax;
      g_adjSeekStep.sfCreepSps  = creep;
      g_adjSeekStep.sfKp        = cfg.seek_kp;
      g_adjSeekStep.sfKi        = cfg.seek_ki;
      g_adjSeekStep.sfKd        = cfg.seek_kd;
      g_adjSeekStep.sfEpsilon   = cfg.seek_epsilon;
      g_adjSeekStep.sfSettleMs  = cfg.seek_settle_ms;
      g_adjSeekStep.sfDir       = cfg.seek_dir >= 0 ? (int8_t)1 : (int8_t)-1;
      g_adjSeekStep.sfMaxTravel = 0;
      unlockCfg();
      memset(&g_seekRT, 0, sizeof g_seekRT);
      g_adjSeekActive = true;
      Serial.printf("[ADJ] Seek force -> %.4f (Vmax=%.0f creep=%.0f). seek stop to abort.\n", tgt, vmax, creep);
      return;
    }
    if (sl == "flush lc") { lc_ring_flush(); return; }
    Serial.println(F("Unknown ADJUSTMENTS command. Type 'help'."));
    return;
  }

  // ------------------------------------------------------------------
  // LIVE_TEST mode
  // ------------------------------------------------------------------
  if (currentMode == Mode::LIVE_TEST) {

    if (sl.startsWith("start")) {
      if (numSteps == 0) { Serial.println(F("No sequence. Define steps in SETUP first.")); return; }

      String arg = sl.substring(5); arg.trim();

      // Parse cycle count (first token)
      int n = 1;
      int spaceIdx = arg.indexOf(' ');
      if (spaceIdx > 0) {
        n = arg.substring(0, spaceIdx).toInt();
        arg = arg.substring(spaceIdx + 1); arg.trim();
      } else if (arg.length() > 0) {
        n = arg.toInt();
        arg = "";
      }
      if (n < 1) n = 1;

      // Parse optional "LC every <k>" or "LC cycles <c1,c2,...>"
      profileMode       = ProfileMode::ALL;
      profileEveryN     = 1;
      profileCycleCount = 0;

      if (arg.startsWith("lc")) {
        String lcArg = arg.substring(2); lcArg.trim();
        if (lcArg.startsWith("every ")) {
          profileMode   = ProfileMode::EVERY_N;
          profileEveryN = lcArg.substring(6).toInt();
          if (profileEveryN < 1) profileEveryN = 1;
        } else if (lcArg.startsWith("cycles ")) {
          profileMode = ProfileMode::EXPLICIT;
          String cList = lcArg.substring(7); cList.trim();
          int cStart = 0;
          while (cStart < (int)cList.length() && profileCycleCount < MAX_PROFILE_CYCLES) {
            int comma = cList.indexOf(',', cStart);
            String tok = (comma < 0) ? cList.substring(cStart) : cList.substring(cStart, comma);
            tok.trim();
            if (tok.length() > 0) profileCycles[profileCycleCount++] = tok.toInt();
            if (comma < 0) break;
            cStart = comma + 1;
          }
        }
      }

      cyclesGoal      = n;
      cyclesDone      = 0;
      currentStepIdx  = 0;
      stepInitialized = false;
      lcLimitTrip     = false;
      runState        = RunState::RUNNING;

      Serial.printf("[TEST] START %d cycle(s)", n);
      LINK_UART.printf("R cyc=%d/%d \n", cyclesDone, cyclesGoal);
      switch (profileMode) {
        case ProfileMode::ALL:      Serial.println(" — LC on all cycles"); break;
        case ProfileMode::EVERY_N:  Serial.printf(" — LC every %d cycle(s)\n", profileEveryN); break;
        case ProfileMode::EXPLICIT: Serial.printf(" — LC on %d explicit cycle(s)\n", profileCycleCount); break;
      }
      return;
    }

    if (sl == "pause") {
      if (runState == RunState::RUNNING) {
        runState = RunState::PAUSED;
        stepper.stop();
        g_velocitySeekMode = false;
        g_velocityCommand  = 0.f;
        memset(&g_seekRT, 0, sizeof g_seekRT);
        Serial.println(F("[TEST] PAUSED."));
        LINK_UART.println("S paused");  // S = State change
      } else Serial.println(F("[TEST] Not running."));
      return;
    }
    if (sl == "resume") {
      if (runState == RunState::PAUSED) { stepInitialized = false; runState = RunState::RUNNING; Serial.println(F("[TEST] RESUMED.")); }
      else Serial.println(F("[TEST] Not paused.")); return;
    }
    if (sl == "stop") {
      stepper.stop();
      runState = RunState::IDLE;
      cyclesDone = 0;
      currentStepIdx = 0;
      g_adjSeekActive    = false;
      g_velocitySeekMode = false;
      g_velocityCommand  = 0.f;
      memset(&g_seekRT, 0, sizeof g_seekRT);
      Serial.println(F("[TEST] STOPPED."));
      return;
    }
    if (sl == "status") { printStatus(); return; }

    Serial.println(F("Unknown LIVE_TEST command. Type 'help'."));
    return;
  }
}

// ============================================================================
// SERIAL LINE READERS
// ============================================================================

bool readLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (buf.length() > 0) { out = buf; buf = ""; return true; } }
    else buf += c;
  }
  return false;
}

bool readLink(String &out) {
  static String buf;
  while (LINK_UART.available()) {
    char c = (char)LINK_UART.read();
    if (c == '\r') continue;
    if (c == '\n') { if (buf.length() > 0) { out = buf; buf = ""; return true; } }
    else { buf += c; if (buf.length() > 256) buf = ""; }
  }
  return false;
}

// ============================================================================
// TEST EXECUTION
// ============================================================================
/*
 * tickTestExecution() runs every Core 1 tick during LIVE_TEST.
 *
 * In-motion LC sampling during a MOVE step:
 *   - lcActiveThisCycle() determines whether data is logged this cycle.
 *     On non-logging cycles, lc_inMotionRead() still checks limits if L1 is set.
 *   - Rate-based: reads when (now - lcLastRateMs) >= step.lcRateMs
 *   - Position-based: reads when the motor crosses each position in lcPositions[]
 *     Positions are sorted ascending/descending at step init to support both
 *     forward and reverse moves.
 *   - Both modes can fire on the same step; they are checked independently.
 *
 * The status heartbeat in taskControl includes the last LC value if in-motion
 * sampling is active, keeping the "[LIVE]" line informative.
 */

// Returns the first step index whose stepLabel == label, or -1 if not found.
// Caller must NOT hold seqMutex.
int findStepByLabel(uint8_t label) {
  if (label == 0) return -1;
  lockSeq();
  int found = -1;
  for (int i = 0; i < numSteps; i++) {
    if (testSequence[i].stepLabel == label) { found = i; break; }
  }
  unlockSeq();
  return found;
}

// Jump to a labeled step on failure, or pause if the label isn't found / not set.
// Returns true if we jumped (caller should return from tickTestExecution immediately).
bool handleFailGoto(const TestStep &step) {
  if (step.gotoLabel != 0) {
    int idx = findStepByLabel(step.gotoLabel);
    if (idx >= 0) {
      Serial.printf("[TEST] -> goto label %d (step %d)\n", step.gotoLabel, idx);
      currentStepIdx  = idx;
      stepInitialized = false;
      return true;
    }
    Serial.printf("[TEST] WARNING: goto label %d not found, pausing.\n", step.gotoLabel);
  }
  runState = RunState::PAUSED;
  stepper.stop();
  return false;
}

void tickTestExecution() {
  if (runState != RunState::RUNNING) return;

  // Cycle complete
  if (currentStepIdx >= numSteps) {
    cyclesDone++;
    Serial.printf("[TEST] Cycle %d / %d complete.\n", cyclesDone, cyclesGoal);
    LINK_UART.printf("R cyc=%d/%d \n", cyclesDone, cyclesGoal);
    if (cyclesDone >= cyclesGoal) {
      runState = RunState::COMPLETE; stepper.stop();
      Serial.println(F("[TEST] All cycles complete."));
      LINK_UART.println("S complete");  // S = State change
      return;
    }
    currentStepIdx = 0; stepInitialized = false;
    return;
  }

  lockSeq();
  const TestStep &step = testSequence[currentStepIdx];
  unlockSeq();

  // ---- Initialize step ----
  if (!stepInitialized) {
    stepInitialized = true;
    stepStartMs     = millis();

    switch (step.type) {

      case StepType::MOVE: {
        lockCfg();
        float spd = (step.speed > 0) ? step.speed : cfg.maxSpeed_sps;
        float acc = (step.accel > 0) ? step.accel : cfg.accel_sps2;
        long  tgt = 0;
        bool  valid = true;
        if (step.useNamedPos) {
          if (cfg.namedPosSet[step.posVarIdx]) tgt = cfg.namedPositions[step.posVarIdx];
          else { Serial.printf("[TEST] WARNING: X%c not set, skip step %d.\n", 'A'+step.posVarIdx, currentStepIdx); valid = false; }
        } else { tgt = step.targetSteps; }

        // Resolve concurrent servo target while cfg is held, then write after unlock
        float concurrentServoDeg  = -1.f;
        bool  doConcurrentServo   = false;
        if (valid && step.servoWithMove) {
          if (step.servoMoveUseNamed) {
            if (cfg.servoNamedPosSet[step.servoMoveVarIdx]) {
              concurrentServoDeg = cfg.servoNamedPositions[step.servoMoveVarIdx];
              doConcurrentServo  = true;
            } else {
              Serial.printf("[TEST] WARNING: S%c not set, skipping concurrent servo on step %d.\n",
                            'A'+step.servoMoveVarIdx, currentStepIdx);
            }
          } else {
            concurrentServoDeg = step.servoMoveDeg;
            doConcurrentServo  = true;
          }
        }

        if (valid) {
          stepper.setMaxSpeed(spd); stepper.setAcceleration(acc); stepper.moveTo(tgt);
          Serial.printf("[TEST] Step %d: MOVE -> %ld  F%.0f  A%.0f%s%s\n",
                        currentStepIdx, tgt, spd, acc,
                        step.lcInMotion ? "  [LC]" : "",
                        doConcurrentServo ? "  [SERVO]" : "");
        }
        unlockCfg();
        if (!valid) { currentStepIdx++; stepInitialized = false; }

        // Fire servo now (outside cfg lock since servo_writeDegrees acquires it)
        if (doConcurrentServo) {
          servo_writeDegrees(concurrentServoDeg);
          Serial.printf("[TEST] Step %d: SERVO concurrent -> %.1f deg\n",
                        currentStepIdx, concurrentServoDeg);
        }

        // Reset in-motion LC state
        lcLastRateMs = millis();
        lcNextPosIdx = 0;
        // Sort position list in the direction of travel for correct trigger order
        if (step.lcPosCount > 1) {
          long curPos = stepper.currentPosition();
          // Determine direction
          bool forward = (step.useNamedPos
            ? (cfg.namedPosSet[step.posVarIdx] && cfg.namedPositions[step.posVarIdx] > curPos)
            : (step.targetSteps > curPos));
          // Sort directly on the global array, not through the const reference
    lockSeq();
    for (int a = 0; a < testSequence[currentStepIdx].lcPosCount - 1; a++) {
        for (int b = 0; b < testSequence[currentStepIdx].lcPosCount - 1 - a; b++) {
            bool doSwap = forward
                ? (testSequence[currentStepIdx].lcPositions[b] > testSequence[currentStepIdx].lcPositions[b+1])
                : (testSequence[currentStepIdx].lcPositions[b] < testSequence[currentStepIdx].lcPositions[b+1]);
            if (doSwap) {
                long tmp = testSequence[currentStepIdx].lcPositions[b];
                testSequence[currentStepIdx].lcPositions[b]   = testSequence[currentStepIdx].lcPositions[b+1];
                testSequence[currentStepIdx].lcPositions[b+1] = tmp;
            }
        }
    }
    unlockSeq();
        }
        return;
      }

      case StepType::DWELL:
        Serial.printf("[TEST] Step %d: DWELL %lu ms\n", currentStepIdx, (unsigned long)step.dwellMs);
        return;

      case StepType::PAUSE:
        Serial.printf("[TEST] Step %d: PAUSE — send 'resume' to continue.\n", currentStepIdx);
        LINK_UART.println("S paused");  // S = State change
        // Advance so 'resume' continues with the NEXT step after this pause marker.
        currentStepIdx++;
        stepInitialized = false;
        runState = RunState::PAUSED;
        return;

      case StepType::READ_LC:
        return;

      case StepType::SEEK_FORCE:
        memset(&g_seekRT, 0, sizeof g_seekRT);
        g_velocitySeekMode = false;
        g_velocityCommand  = 0.f;
        Serial.printf("[TEST] Step %d: SEEK_FORCE target=%.4f  Vmax=%.0f creep=%.0f\n",
                      currentStepIdx, step.sfTarget, step.sfMaxSps, step.sfCreepSps);
        return;

      case StepType::SERVO_MOVE: {
        // Fire-and-forget: resolve position, write servo, advance immediately
        float deg;
        bool doMove = true;
        if (step.servoUseNamed) {
          lockCfg();
          bool isSet = cfg.servoNamedPosSet[step.servoVarIdx];
          deg        = cfg.servoNamedPositions[step.servoVarIdx];
          unlockCfg();
          if (!isSet) {
            Serial.printf("[TEST] WARNING: S%c not set, skip step %d.\n",
                          'A'+step.servoVarIdx, currentStepIdx);
            doMove = false;
          }
        } else {
          deg = step.servoDeg;
        }
        if (doMove) {
          servo_writeDegrees(deg);
          Serial.printf("[TEST] Step %d: SERVO_MOVE -> %.1f deg\n", currentStepIdx, deg);
        }
        currentStepIdx++; stepInitialized = false;
        return;
      }

      case StepType::WAIT_BB:
        Serial.printf("[TEST] Step %d: WAIT_BB  sensor=%d  expect=%s  mode=%s",
                      currentStepIdx, step.bbSensor,
                      step.bbExpected ? "broken" : "clear",
                      step.bbWaitMode ? "wait" : "check");
        if (step.bbWaitMode && step.bbTimeoutMs)
          Serial.printf("  timeout=%lums", (unsigned long)step.bbTimeoutMs);
        Serial.println();
        return;
    }
  }

  // ---- Progress step ----
  bool loggingThisCycle = lcActiveThisCycle();

  switch (step.type) {

    case StepType::MOVE: {
      // --- In-motion LC sampling ---
      if (step.lcInMotion) {
        long pos = stepper.currentPosition();
        uint32_t now = millis();

        // Rate-based trigger
        if (step.lcRateMs > 0 && (now - lcLastRateMs) >= step.lcRateMs) {
          lcLastRateMs = now;
          if (!lc_inMotionRead(step, pos, loggingThisCycle)) return; // limit tripped
        }

        // Position-based trigger
        if (step.lcPosCount > 0 && lcNextPosIdx < step.lcPosCount) {
          long trigger = step.lcPositions[lcNextPosIdx];
          long target  = stepper.targetPosition();
          bool crossed = (target >= stepper.currentPosition())   // forward move
                         ? (pos >= trigger)
                         : (pos <= trigger);                      // reverse move
          if (crossed) {
            lcNextPosIdx++;
            if (!lc_inMotionRead(step, pos, loggingThisCycle)) return;
          }
        }
      }

      // Arrival check
      if (stepper.distanceToGo() == 0) {
        lockCfg();
        bool flushRing = cfg.lc_ring_enable;
        unlockCfg();
        if (flushRing) lc_ring_flush();
        currentStepIdx++; stepInitialized = false;
      }
      break;
    }

    case StepType::SEEK_FORCE: {
      int r = seekForceProgress(step);
      if (r == 1) {
        lockCfg();
        bool flushRing = cfg.lc_ring_enable;
        unlockCfg();
        if (flushRing) lc_ring_flush();
        currentStepIdx++; stepInitialized = false;
      }
      break;
    }

    case StepType::DWELL:
      if (millis() - stepStartMs >= step.dwellMs) {
        currentStepIdx++; stepInitialized = false;
      }
      break;

    case StepType::READ_LC: {
      // For a stationary read we want a fresh sample. Block briefly via lc_read()
      // here since the motor is stopped — this is the one place blocking is acceptable.
      long pos = stepper.currentPosition();
      float val = lc_read(step.lcReadings);
      lastLcValue = val;
      // Also update the cache so subsequent reads see the freshest value
      xSemaphoreTake(lcLatestMux, portMAX_DELAY);
      g_lcLatest.value = val; g_lcLatest.pos = pos; g_lcLatest.t_ms = millis(); g_lcLatest.fresh = false;
      xSemaphoreGive(lcLatestMux);
      Serial.printf("[TEST] Step %d: Pos=%ld LC=%.4f\n", currentStepIdx, pos, val);
      LINK_UART.printf("R R pos=%ld lc=%.4f\n", pos, val);  // R = Result (stationary read)

      if (step.checkLimits) {
        lockCfg();
        bool limitsOn = cfg.lc_limitsOn;
        unlockCfg();
        if (limitsOn) {
          bool over = (val > step.readStepUpperLimit || val < step.readStepLowerLimit);
          if (over) {
            Serial.printf("[TEST] LC LIMIT EXCEEDED! (%.4f not in [%.3f, %.3f])\n",
                          val, step.readStepLowerLimit, step.readStepUpperLimit);
            lcLimitTrip = true;
            if (handleFailGoto(step)) return;
            return;
          }
        }
      }
      currentStepIdx++; stepInitialized = false;
      break;
    }

    case StepType::WAIT_BB: {
      BreakBeamSensor &sensor = (step.bbSensor == 2) ? g_break2 : g_break1;
      bool broken   = sensor.isBroken();
      bool stateOk  = (broken == step.bbExpected);

      if (step.bbWaitMode) {
        // Wait-until-right: keep looping until expected state or timeout
        if (stateOk) {
          Serial.printf("[TEST] Step %d: BB%d reached expected state (%s).\n",
                        currentStepIdx, step.bbSensor, broken ? "broken" : "clear");
          currentStepIdx++; stepInitialized = false;
        } else if (step.bbTimeoutMs > 0 && (millis() - stepStartMs) >= step.bbTimeoutMs) {
          Serial.printf("[TEST] Step %d: BB%d TIMEOUT (expected %s, got %s).\n",
                        currentStepIdx, step.bbSensor,
                        step.bbExpected ? "broken" : "clear",
                        broken ? "broken" : "clear");
          if (handleFailGoto(step)) return;
        }
        // else: still waiting, stay in this step
      } else {
        // Check-once: advance if OK, pause/goto if not
        if (stateOk) {
          Serial.printf("[TEST] Step %d: BB%d OK (%s).\n",
                        currentStepIdx, step.bbSensor, broken ? "broken" : "clear");
          currentStepIdx++; stepInitialized = false;
        } else {
          Serial.printf("[TEST] Step %d: BB%d WRONG STATE! Expected %s, got %s.\n",
                        currentStepIdx, step.bbSensor,
                        step.bbExpected ? "broken" : "clear",
                        broken ? "broken" : "clear");
          if (handleFailGoto(step)) return;
        }
      }
      break;
    }

    case StepType::SERVO_MOVE:
      break;  // handled entirely in init; should not be reached
  }
}

// ============================================================================
// TASK: LC SAMPLE  (Core 1, Priority 2)
// ============================================================================
/*
 * taskLcSample owns all HX711 reads during test execution and ADJ seek.
 * It writes results to g_lcLatest so taskControl never blocks on the HX711.
 *
 * The task only calls lc_read() when the current step actually needs LC data:
 *   - LIVE_TEST RUNNING + current step is MOVE(lcInMotion), SEEK_FORCE, or READ_LC
 *   - ADJUSTMENTS mode with g_adjSeekActive
 * All other times it sleeps cheaply (20 ms) without touching the HX711, so a
 * sequence with no LC steps incurs zero HX711 overhead.
 *
 * lc_sampler_period_ms (set lc_sampler_ms) overrides the sample interval when
 * non-zero. Otherwise the task defaults to 13 ms (~80 SPS) or use 100 ms for
 * a 10 SPS module. Adjust LC_DEFAULT_PERIOD_MS to match your hardware.
 */
#define LC_DEFAULT_PERIOD_MS  13   // ~80 SPS; change to 100 for 10 SPS modules

void taskLcSample(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // ---- Decide whether we need to sample at all ----
    bool needSample = false;

    if (currentMode == Mode::LIVE_TEST && runState == RunState::RUNNING) {
      lockSeq();
      if (currentStepIdx < numSteps) {
        const TestStep &st = testSequence[currentStepIdx];
        needSample = (st.type == StepType::MOVE       && st.lcInMotion) ||
                     (st.type == StepType::SEEK_FORCE)                  ||
                     (st.type == StepType::READ_LC);
      }
      unlockSeq();
    }

    if (currentMode == Mode::ADJUSTMENTS && g_adjSeekActive) {
      needSample = true;
    }

    if (!needSample) {
      // Idle — sleep cheaply without touching the HX711
      vTaskDelay(pdMS_TO_TICKS(20));
      lastWake = xTaskGetTickCount();  // resync so vTaskDelayUntil won't catchup burst
      continue;
    }

    // ---- Determine sample period ----
    uint16_t per;
    lockCfg();
    per = cfg.lc_sampler_period_ms;
    unlockCfg();
    if (per == 0) per = LC_DEFAULT_PERIOD_MS;

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(per));

    // ---- Take one sample (blocks for one HX711 conversion on this task) ----
    long  pos = stepper.currentPosition();
    float val = lc_read(1);
    lastLcValue = val;

    // Write to cache — consumed non-blocking by taskControl
    xSemaphoreTake(lcLatestMux, portMAX_DELAY);
    g_lcLatest.value = val;
    g_lcLatest.pos   = pos;
    g_lcLatest.t_ms  = millis();
    g_lcLatest.fresh = true;
    xSemaphoreGive(lcLatestMux);

    // Ring push if enabled (existing behaviour preserved)
    bool ringOn;
    lockCfg();
    ringOn = cfg.lc_ring_enable;
    unlockCfg();
    if (ringOn && currentMode == Mode::LIVE_TEST && runState == RunState::RUNNING) {
      lc_ring_push(g_lcLatest.t_ms, pos, val);
    }
  }
}

// ============================================================================
// TASK: STEPPER  (Core 0, Priority 5)
// ============================================================================

void taskStepper(void *pv) {
  long     lastPos    = LONG_MIN;
  bool     jogWasMoving = false;
  uint32_t lastIdleMs = millis();

  for (;;) {
    if (g_velocitySeekMode) {
      stepper.setSpeed(g_velocityCommand);
      stepper.runSpeed();
    } else {
      stepper.run();
    }

    // ---- Jog position reporting (ADJUSTMENTS mode) ----
    // All Serial work is done by Core 1 — we only write to volatiles here.
    if (currentMode == Mode::ADJUSTMENTS) {
      long pos = stepper.currentPosition();
      bool moving = (stepper.distanceToGo() != 0);

      if (moving) {
        // Report every 50-step boundary crossing
        long boundary = pos / 400;   // integer divide — changes every 50 steps
        if (boundary != lastPos / 400 || lastPos == LONG_MIN) {
          jogReportPos = pos;       // Core 1 will print this
          lastPos      = pos;
        }
        jogWasMoving = true;
        jogMoveDone  = false;
      } else {
        // Motor just stopped — signal Core 1 to print the final position once,
        // but only if we were actually moving (guards against boot-time spam).
        // true if tru and false

        if (jogWasMoving && !jogMoveDone) {
          jogWasMoving = false;
          jogFinalPos  = pos;
          jogMoveDone  = true;
          jogReportPos = LONG_MIN;
          lastPos      = LONG_MIN;
        }
      }
    } else {
      lastPos      = LONG_MIN;
      jogWasMoving = false;
      jogMoveDone  = false;
    }

    // Watchdog-friendly yielding
    uint32_t now = millis();
    if (now - lastIdleMs >= 100) { vTaskDelay(1); lastIdleMs = now; }
    else taskYIELD();
    if (runState != RunState::RUNNING && stepper.distanceToGo() == 0) vTaskDelay(1);
  }
}

// ============================================================================
// TASK: CONTROL  (Core 1, Priority 3)
// ============================================================================

void taskControl(void *pv) {
  uint32_t lastStatusMs = 0;
  const uint32_t STATUS_MS = 200;  // 5 Hz heartbeat

  for (;;) {
    String line;
    if (readLine(line)) handleCommand(line);
    String linkLine;
    if (readLink(linkLine)) handleCommand(linkLine);

    // ---- Jog position reporting (reads volatile set by Core 0) ----
    if (currentMode == Mode::ADJUSTMENTS) {
      // Mid-move periodic update (every ~50 steps)
      long rpt = jogReportPos;
      if (rpt != LONG_MIN) {
        jogReportPos = LONG_MIN;          // clear so we don't print twice
        // Serial.printf("[JOG] pos=%ld\n", rpt);
      }
      //Final position after move completet
       if (jogMoveDone) {
         jogMoveDone  = false;
         Serial.printf("[JOG] done  pos=%ld\n", jogFinalPos);
         LINK_UART.printf("P pos=%ld\n", jogFinalPos);  // P = Position update
       }
    }

    if (currentMode == Mode::ADJUSTMENTS) serviceAdjSeekForce();

    if (currentMode == Mode::LIVE_TEST) tickTestExecution();

    uint32_t now = millis();
    if (currentMode == Mode::LIVE_TEST && runState == RunState::RUNNING) {
      lockCfg();
      bool ringEn = cfg.lc_ring_enable;
      uint16_t th = cfg.lc_ring_flush_thresh;
      unlockCfg();
      if (ringEn && lc_ring_count() >= th) lc_ring_flush();

      if (now - lastStatusMs >= STATUS_MS) {
        lastStatusMs = now;

        bool anyLc = false;
        lockSeq();
        for (int i = 0; i < numSteps; i++) if (testSequence[i].lcInMotion) { anyLc = true; break; }
        unlockSeq();

       /* 
       if (anyLc) {
          Serial.printf("[LIVE] t=%lu  cyc=%d/%d  step=%d/%d  pos=%ld  lc=%.4f  %s%s\n",
                        (unsigned long)now, cyclesDone, cyclesGoal,
                        currentStepIdx, numSteps,
                        stepper.currentPosition(),
                        lastLcValue,
                        stateStr(runState),
                        lcLimitTrip ? "  [LC_TRIP]" : "");
        } else {
          Serial.printf("[LIVE] t=%lu  cyc=%d/%d  step=%d/%d  pos=%ld  %s%s\n",
                        (unsigned long)now, cyclesDone, cyclesGoal,
                        currentStepIdx, numSteps,
                        stepper.currentPosition(),
                        stateStr(runState),
                        lcLimitTrip ? "  [LC_TRIP]" : "");
        }
      */
      }
    }

    vTaskDelay(1);
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  LINK_UART.begin(115200, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  while (!Serial) delay(10);
  
  Serial.println(F("\n=== ESP32 Test Fixture V5 ==="));

  cfgMutex   = xSemaphoreCreateMutex();
  seqMutex   = xSemaphoreCreateMutex();
  lcMutex    = xSemaphoreCreateMutex();
  lcRingMux  = xSemaphoreCreateMutex();
  lcLatestMux = xSemaphoreCreateMutex();

  if (loadConfig())   Serial.println(F("Config restored from NVS."));
  else                Serial.println(F("No saved config — using defaults."));

  if (loadSequence()) Serial.printf("Sequence restored from NVS (%d steps).\n", numSteps);
  else                Serial.println(F("No saved sequence found."));

  if (ENABLE_PIN >= 0) { pinMode(ENABLE_PIN, OUTPUT); enableMotor(true); }
  stepper.setMinPulseWidth(20);                              
  stepper.setMaxSpeed(cfg.maxSpeed_sps);
  stepper.setAcceleration(cfg.accel_sps2);
  stepper.setCurrentPosition(0);

  // HX711 electrical rate: module RATE pin (or jumper) selects ~10 vs ~80 SPS; match library 3rd arg to wiring.
  lc.begin(LC_DT_PIN, LC_SCK_PIN, false);
  lc.reset();
  bool anyCal = false;
  for (int i = 0; i < LC_INDEX_SIZE; i++) if (cfg.lc_calUsed[i]) { anyCal = true; break; }
  if (anyCal) { lc_applyCalibration(); Serial.println(F("LC calibration restored.")); }

  // Servo + break-beam init (must run AFTER loadConfig so we pick up calibrated pulses).
  servo_init();
  break_applyConfig();
  Serial.printf("[BB] pin1=%d (act_low=%d pullup=%d)  pin2=%d (act_low=%d pullup=%d)\n",
                BREAK1_PIN, cfg.break1ActiveLow, cfg.break1UsePullup,
                BREAK2_PIN, cfg.break2ActiveLow, cfg.break2UsePullup);

  delay(200);
  printHelp();
  printStatus();

  linkAnnounceProtocol(LINK_UART, cfg.link_proto_version);

  xTaskCreatePinnedToCore(taskStepper, "Stepper", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(taskControl, "Control", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(taskLcSample, "LcSamp", 3072, nullptr, 2, &taskLcSampleHandle, 1);

  LINK_UART.println("S ready");   // S = State change; lets the display ESP32 know the fixture is up
}

void loop() { vTaskDelay(portMAX_DELAY); }

// Debugging tools
// Serial.println(jogMoveDone ? "jogMoveDone true" : "jogMoveDone false");