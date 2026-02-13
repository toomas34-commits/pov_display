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

// 3D ribbon effect tuning.
constexpr float EFFECT_SPIN_HZ = 0.32f;          // global animation speed
constexpr float HELIX_AMPLITUDE = 0.72f;         // vertical excursion
constexpr uint8_t TRAIL_SAMPLES = 3;             // more = longer afterglow trail
constexpr float TRAIL_ANGULAR_LAG_RAD = 0.18f;   // angular spacing between trail samples

// Hall pulse limits for filtering noise and handling restarts.
constexpr uint32_t MIN_HALF_PERIOD_US = 1500;      // reject bounce/noise
constexpr uint32_t MAX_HALF_PERIOD_US = 2000000; // reject impossible slow edge
constexpr uint32_t MIN_SIGNAL_TIMEOUT_US = 400000;
constexpr uint8_t MAX_MISSED_HALVES_BEFORE_BLANK = 8;

CRGB leds[NUM_LEDS];
float g_sliceTheta[ANGLE_SLICES];
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
    const float t = static_cast<float>(x) / static_cast<float>(ANGLE_SLICES);
    g_sliceTheta[x] = t * TWO_PI;
  }

  for (uint16_t y = 0; y < LEDS_PER_ARM; y++) {
    const float t = static_cast<float>(y) / static_cast<float>(LEDS_PER_ARM - 1);
    normY[y] = t * 2.0f - 1.0f;
  }
}

inline float gaussianFalloff(float delta, float width) {
  const float safeWidth = fmaxf(width, 0.01f);
  return expf(-(delta * delta) / (safeWidth * safeWidth));
}

CRGB helixPixel(uint16_t slice, uint8_t y, float phase) {
  const float theta = g_sliceTheta[slice];
  const float h = normY[y];

  CRGB pixel = CRGB::Black;

  for (uint8_t strand = 0; strand < 2; strand++) {
    const float strandPhase = strand ? PI : 0.0f;
    const float wavePhase = (2.0f * theta) + phase + strandPhase;
    const float yCore = HELIX_AMPLITUDE * sinf(wavePhase);

    // Depth term controls brightness and thickness for a stronger 3D look.
    const float depth = 0.5f + 0.5f * cosf(theta + phase * 0.65f + strandPhase);
    const float width = 0.05f + 0.13f * depth;

    float intensity = gaussianFalloff(h - yCore, width) * (0.28f + 0.72f * depth);
    for (uint8_t trail = 1; trail <= TRAIL_SAMPLES; trail++) {
      const float lag = TRAIL_ANGULAR_LAG_RAD * static_cast<float>(trail);
      const float trailTheta = theta - lag;
      const float trailWave = (2.0f * trailTheta) + phase + strandPhase;
      const float yTrail = HELIX_AMPLITUDE * sinf(trailWave);
      const float trailDepth = 0.5f + 0.5f * cosf(trailTheta + phase * 0.65f + strandPhase);
      const float trailWidth = 0.05f + 0.11f * trailDepth;
      intensity += gaussianFalloff(h - yTrail, trailWidth) *
                   (0.35f / static_cast<float>(trail)) *
                   (0.20f + 0.80f * trailDepth);
    }

    intensity = fminf(intensity, 1.0f);
    const uint8_t value = static_cast<uint8_t>(255.0f * intensity);
    const uint8_t baseHue = strand ? 170 : 6;  // cyan + amber strands
    const uint8_t hueDrift =
        static_cast<uint8_t>(phase * 14.0f + depth * 24.0f);
    pixel += ColorFromPalette(PartyColors_p, baseHue + hueDrift, value, LINEARBLEND);
  }

  // Dim center glow helps the helix feel volumetric while spinning.
  const float centerBoost = 1.0f - fminf(fabsf(h), 1.0f);
  const uint8_t ambience = static_cast<uint8_t>(centerBoost * 16.0f);
  pixel += CRGB(0, ambience / 2U, ambience);
  return pixel;
}

void renderSlice(uint16_t sliceA, uint32_t nowMs) {
  const uint16_t sliceB = (sliceA + ANGLE_SLICES / 2U) % ANGLE_SLICES;
  const float phase =
      fmodf((static_cast<float>(nowMs) * 0.001f) * TWO_PI * EFFECT_SPIN_HZ, TWO_PI);

  for (uint8_t y = 0; y < LEDS_PER_ARM; y++) {
    leds[armAIndexFromBottom(y)] = helixPixel(sliceA, y, phase);
    leds[armBIndexFromBottom(y)] = helixPixel(sliceB, y, phase);
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

  const uint32_t nowMs = nowUs / 1000U;
  renderSlice(displaySlice, nowMs);
  FastLED.show();
  g_lastRenderedSlice = displaySlice;
  g_ledsAreCleared = false;
}
