#include <Arduino.h>
#include <FastLED.h>

// ------------------------- Hardware configuration -------------------------
constexpr uint8_t DATA_PIN = 23;   // SK9822 DI
constexpr uint8_t CLOCK_PIN = 18;  // SK9822 CI
constexpr uint8_t HALL_PIN = 27;   // KY-003 digital output

constexpr uint16_t LEDS_PER_ARM = 33;
constexpr uint16_t NUM_LEDS = LEDS_PER_ARM * 2;  // 66 total

// Arm A: LED 0..32 (bottom -> top)
constexpr uint16_t ARM_A_START = 0;
// Arm B: LED 33..65 physically wired top -> bottom
constexpr uint16_t ARM_B_START = LEDS_PER_ARM;

// POV angular resolution.
constexpr uint16_t ANGLE_SLICES = 180;  // 2 degrees per slice
constexpr int16_t ANGLE_OFFSET_SLICES = 0;  // rotate image if needed

constexpr uint8_t BRIGHTNESS = 72;
constexpr EOrder COLOR_ORDER = BGR;  // SK9822 strips are often BGR

// Hall pulse limits for filtering noise and handling restarts.
constexpr uint32_t MIN_HALF_PERIOD_US = 1500;      // reject bounce/noise
constexpr uint32_t MAX_HALF_PERIOD_US = 2000000; // reject impossible slow edge
constexpr uint32_t MIN_SIGNAL_TIMEOUT_US = 400000;
constexpr uint8_t MAX_MISSED_HALVES_BEFORE_BLANK = 8;

CRGB leds[NUM_LEDS];
float normX[ANGLE_SLICES];
float normY[LEDS_PER_ARM];

volatile uint32_t g_lastEdgeUs = 0;
volatile uint32_t g_lastPulseUs = 0;
volatile uint32_t g_halfPeriodUs = 30000;  // startup estimate
volatile uint8_t g_halfTurn = 0;            // 0: first 180 deg, 1: second 180 deg
volatile uint16_t g_syncedHalfTurns = 0;    // valid half-turn intervals measured

uint16_t g_lastRenderedSlice = 0xFFFF;
bool g_ledsAreCleared = false;

inline uint16_t armAIndexFromBottom(uint8_t y) {
  return ARM_A_START + y;
}

inline uint16_t armBIndexFromBottom(uint8_t y) {
  return ARM_B_START + (LEDS_PER_ARM - 1U - y);
}

void IRAM_ATTR onHallEdge() {
  const uint32_t nowUs = micros();

  if (g_lastEdgeUs == 0) {
    g_lastEdgeUs = nowUs;
    g_lastPulseUs = nowUs;
    g_syncedHalfTurns = 0;
    g_halfTurn = 0;
    return;
  }

  const uint32_t dt = nowUs - g_lastEdgeUs;
  if (dt < MIN_HALF_PERIOD_US) {
    return;  // debounce / EMI rejection
  }

  // Ignore implausibly short intervals after lock (noise spikes).
  if (g_syncedHalfTurns > 6 && (dt * 4U) < g_halfPeriodUs) {
    return;
  }

  if (dt > MAX_HALF_PERIOD_US) {
    // Lost lock (motor stopped or missed pulses). Re-lock from this new edge.
    g_lastEdgeUs = nowUs;
    g_lastPulseUs = nowUs;
    g_syncedHalfTurns = 0;
    g_halfTurn = 0;
    return;
  }

  // If dt is close to 2x/3x expected after lock, assume pulse(s) were skipped.
  // This keeps half-period estimation stable and preserves phase continuity.
  uint32_t halfStepsSinceLastPulse = 1U;
  uint32_t dtForFilter = dt;
  if (g_syncedHalfTurns > 6) {
    const uint32_t expectedHalfUs = g_halfPeriodUs;
    const uint32_t missedPulseThresholdUs = (expectedHalfUs * 7U) / 4U;
    if (dt > missedPulseThresholdUs) {
      const uint32_t estimatedSteps =
          (dt + (expectedHalfUs / 2U)) / expectedHalfUs;  // nearest integer
      if (estimatedSteps >= 2U && estimatedSteps <= 4U) {
        halfStepsSinceLastPulse = estimatedSteps;
        dtForFilter = dt / estimatedSteps;
      }
    }
  }

  // Low-pass filter half-turn duration so slice timing follows small RPM drift.
  // If a pulse is delayed/missed, cap filter input to avoid large timing jumps.
  if (g_syncedHalfTurns > 6) {
    const uint32_t maxFilterDt = g_halfPeriodUs * 2U;
    if (dtForFilter > maxFilterDt) {
      dtForFilter = maxFilterDt;
    }
  }
  g_halfPeriodUs = (g_halfPeriodUs * 7U + dtForFilter) / 8U;
  g_halfTurn ^= (halfStepsSinceLastPulse & 1U);
  if (g_syncedHalfTurns < (0xFFFF - halfStepsSinceLastPulse)) {
    g_syncedHalfTurns += halfStepsSinceLastPulse;
  } else {
    g_syncedHalfTurns = 0xFFFF;
  }

  g_lastEdgeUs = nowUs;
  g_lastPulseUs = nowUs;
}

void precomputeCoordinates() {
  for (uint16_t x = 0; x < ANGLE_SLICES; x++) {
    const float t = static_cast<float>(x) / static_cast<float>(ANGLE_SLICES - 1);
    normX[x] = t * 2.0f - 1.0f;
  }

  for (uint16_t y = 0; y < LEDS_PER_ARM; y++) {
    const float t = static_cast<float>(y) / static_cast<float>(LEDS_PER_ARM - 1);
    normY[y] = t * 2.0f - 1.0f;
  }
}

CRGB smileyPixel(uint16_t slice, uint8_t y) {
  const float x = normX[slice];
  const float h = normY[y];

  const float faceRadius2 = x * x + h * h;
  if (faceRadius2 > 1.0f) {
    return CRGB::Black;
  }

  // Eyes
  const float lx = x + 0.35f;
  const float ly = h - 0.35f;
  if ((lx * lx + ly * ly) < 0.020f) {
    return CRGB::Black;
  }

  const float rx = x - 0.35f;
  const float ry = h - 0.35f;
  if ((rx * rx + ry * ry) < 0.020f) {
    return CRGB::Black;
  }

  // Mouth arc (lower half ring segment).
  const float my = h + 0.15f;
  const float mouthR2 = x * x + my * my;
  if (h < -0.05f && mouthR2 > 0.22f && mouthR2 < 0.34f) {
    return CRGB::Black;
  }

  return CRGB(255, 185, 0);
}

void renderSlice(uint16_t sliceA) {
  const uint16_t sliceB = (sliceA + ANGLE_SLICES / 2U) % ANGLE_SLICES;

  for (uint8_t y = 0; y < LEDS_PER_ARM; y++) {
    leds[armAIndexFromBottom(y)] = smileyPixel(sliceA, y);
    leds[armBIndexFromBottom(y)] = smileyPixel(sliceB, y);
  }
}

void clearLedsIfNeeded() {
  if (!g_ledsAreCleared) {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    g_ledsAreCleared = true;
    g_lastRenderedSlice = 0xFFFF;
  }
}

void setup() {
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.setDither(0);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  precomputeCoordinates();

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onHallEdge, FALLING);
}

void loop() {
  uint32_t lastPulseUs;
  uint32_t halfPeriodUs;
  uint8_t halfTurn;
  uint16_t syncedHalfTurns;

  noInterrupts();
  lastPulseUs = g_lastPulseUs;
  halfPeriodUs = g_halfPeriodUs;
  halfTurn = g_halfTurn;
  syncedHalfTurns = g_syncedHalfTurns;
  interrupts();

  if (syncedHalfTurns == 0) {
    clearLedsIfNeeded();
    return;
  }

  if (halfPeriodUs < MIN_HALF_PERIOD_US) {
    halfPeriodUs = MIN_HALF_PERIOD_US;
  }

  const uint32_t nowUs = micros();
  const uint32_t sincePulseUs = nowUs - lastPulseUs;
  const uint32_t signalTimeoutUs =
      max(MIN_SIGNAL_TIMEOUT_US, halfPeriodUs * MAX_MISSED_HALVES_BEFORE_BLANK);
  if (sincePulseUs > signalTimeoutUs) {
    clearLedsIfNeeded();
    return;
  }

  // Continue phase progression even if a pulse arrives late, so the image does
  // not freeze into a bright block on one side of the cylinder.
  const uint32_t elapsedHalves = sincePulseUs / halfPeriodUs;
  const uint32_t elapsedInHalfUs = sincePulseUs - (elapsedHalves * halfPeriodUs);
  uint8_t predictedHalfTurn = halfTurn;
  if (elapsedHalves & 1U) {
    predictedHalfTurn ^= 1U;
  }

  const uint16_t slicesPerHalf = ANGLE_SLICES / 2U;
  const uint16_t sliceWithinHalf =
      static_cast<uint16_t>((static_cast<uint64_t>(elapsedInHalfUs) * slicesPerHalf) /
                            halfPeriodUs);
  uint16_t baseSlice = predictedHalfTurn ? slicesPerHalf : 0U;
  int32_t correctedSlice = static_cast<int32_t>(baseSlice + sliceWithinHalf) +
                           static_cast<int32_t>(ANGLE_OFFSET_SLICES);
  while (correctedSlice < 0) {
    correctedSlice += ANGLE_SLICES;
  }
  const uint16_t displaySlice = static_cast<uint16_t>(correctedSlice % ANGLE_SLICES);

  if (displaySlice == g_lastRenderedSlice) {
    return;
  }

  renderSlice(displaySlice);
  FastLED.show();
  g_lastRenderedSlice = displaySlice;
  g_ledsAreCleared = false;
}
