#include <Arduino.h>
#include <stdint.h>
#include <math.h>

/*
  STM32U585 RLC frequency-response measurement — stepped sine + PRBS, no DMA

  Wiring:
    PA4 / DAC1_OUT1 ----+---- R ---- L ----+---- Vc
                        |                  |
    PA1 / ADC1_IN6 ------+                  C
      measure Vin                           |
                                           GND

    PA0 / ADC1_IN5 -------------------- Vc
      measure Vc

  Commands (integer-only serial protocol):
    SINE <frequency_millihz> <settle_ms> <capture_ms>\n
      Example, 450 Hz:
        SINE 450000 60 120

    PRBS <order> <periods> <amplitude_mV>\n
      Current implementation supports order 13 (8191 samples/period).
      Example:
        PRBS 13 3 600

  Output frames, little endian:

    Sine frame:
      "FRF1"
      uint32 actual sample-pair rate, Hz
      uint32 sample count
      uint32 generated frequency, milli-Hz
      followed by interleaved uint16 Vin, uint16 Vc pairs

    PRBS frame:
      "PRB1"
      uint32 actual sample-pair rate, Hz
      uint32 sample count
      uint32 PRBS order
      uint32 periods captured
      uint32 amplitude, mV
      followed by interleaved uint16 Vin, uint16 Vc pairs

  ADC1 is initialized once and read directly. No DMA is used.
*/

constexpr uint8_t PIN_DAC     = PA4;
constexpr uint8_t PIN_ADC_VIN = PA1;
constexpr uint8_t PIN_ADC_VC  = PA0;

constexpr uint32_t TARGET_SAMPLE_RATE_HZ = 50000;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / TARGET_SAMPLE_RATE_HZ;

constexpr float VREF = 3.3f;
constexpr uint16_t DAC_MAX = 4095;
constexpr float DAC_OFFSET_V = 1.65f;
constexpr float DAC_AMPLITUDE_V = 0.75f;

constexpr uint16_t DAC_OFFSET_CODE =
    static_cast<uint16_t>((DAC_OFFSET_V / VREF) * DAC_MAX + 0.5f);
constexpr uint16_t DAC_AMPLITUDE_CODE =
    static_cast<uint16_t>((DAC_AMPLITUDE_V / VREF) * DAC_MAX + 0.5f);

constexpr uint32_t LUT_BITS = 10;
constexpr uint32_t LUT_SIZE = 1UL << LUT_BITS;
uint16_t sine_lut[LUT_SIZE];

// 3 x PRBS13 periods = 24,573 samples. Keep a little margin.
constexpr uint32_t MAX_CAPTURE_SAMPLES = 25000;
uint16_t vin_samples[MAX_CAPTURE_SAMPLES];
uint16_t vc_samples[MAX_CAPTURE_SAMPLES];

ADC_HandleTypeDef hadc1;
String command;

static void fatalBlink()
{
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

static void serialWriteAll(const uint8_t *data, size_t len)
{
  // USB CDC / Arduino Serial.write() is allowed to accept fewer bytes than
  // requested when its TX buffer is full.  Never discard a short write.
  size_t sent = 0;
  while (sent < len) {
    const size_t n = Serial.write(data + sent, len - sent);
    if (n > 0) {
      sent += n;
    } else {
      yield();
    }
  }
}

static void serialWriteAll(const char *data, size_t len)
{
  serialWriteAll(reinterpret_cast<const uint8_t *>(data), len);
}

static void writeU32LE(uint32_t value)
{
  const uint8_t b[4] = {
    static_cast<uint8_t>(value),
    static_cast<uint8_t>(value >> 8),
    static_cast<uint8_t>(value >> 16),
    static_cast<uint8_t>(value >> 24)
  };
  serialWriteAll(b, sizeof(b));
}

static inline void adcSelectChannel(uint32_t hal_channel)
{
  const uint32_t channel_number = __HAL_ADC_CHANNEL_TO_DECIMAL_NB(hal_channel);
  const uint32_t sq1 =
      (channel_number << ADC_SQR1_SQ1_Pos) & ADC_SQR1_SQ1_Msk;
  MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1_Msk, sq1);
}

static inline uint16_t adcReadFast(uint32_t hal_channel)
{
  adcSelectChannel(hal_channel);
  ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
  SET_BIT(ADC1->CR, ADC_CR_ADSTART);
  while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
  return static_cast<uint16_t>(ADC1->DR);
}

static void configureAdcChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef cfg = {};
  cfg.Channel = channel;
  cfg.Rank = ADC_REGULAR_RANK_1;
  cfg.SamplingTime = ADC_SAMPLETIME_12CYCLES;
  cfg.SingleDiff = ADC_SINGLE_ENDED;
  cfg.OffsetNumber = ADC_OFFSET_NONE;
  cfg.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) fatalBlink();
}

static void initAdc1Fast()
{
  pinMode(PIN_ADC_VIN, INPUT_ANALOG);
  pinMode(PIN_ADC_VC, INPUT_ANALOG);

  HAL_PWREx_EnableVddA();
  __HAL_RCC_ADC12_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) fatalBlink();

  configureAdcChannel(ADC_CHANNEL_5); // PA0 = Vc
  configureAdcChannel(ADC_CHANNEL_6); // PA1 = Vin

  if (HAL_ADCEx_Calibration_Start(
          &hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    fatalBlink();
  }

  adcSelectChannel(ADC_CHANNEL_5);
  if (HAL_ADC_Start(&hadc1) != HAL_OK) fatalBlink();
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) fatalBlink();
  (void)HAL_ADC_GetValue(&hadc1);
}

static void initSineLut()
{
  for (uint32_t i = 0; i < LUT_SIZE; ++i) {
    const float theta = 2.0f * PI * static_cast<float>(i) / static_cast<float>(LUT_SIZE);
    int32_t code = static_cast<int32_t>(DAC_OFFSET_CODE) +
                   static_cast<int32_t>(lroundf(DAC_AMPLITUDE_CODE * sinf(theta)));
    if (code < 0) code = 0;
    if (code > DAC_MAX) code = DAC_MAX;
    sine_lut[i] = static_cast<uint16_t>(code);
  }
}

static inline void dacWriteFast(uint16_t code)
{
  DAC1->DHR12R1 = code;
}

static inline uint16_t ddsSample(uint32_t phase)
{
  return sine_lut[phase >> (32U - LUT_BITS)];
}

static bool parseSineCommand(const String &line, float &freq_hz,
                             uint32_t &settle_ms, uint32_t &capture_ms)
{
  char buf[96];
  line.toCharArray(buf, sizeof(buf));

  unsigned long f_millihz = 0, s = 0, c = 0;
  if (sscanf(buf, "SINE %lu %lu %lu", &f_millihz, &s, &c) != 3) return false;

  if (f_millihz < 5000UL || f_millihz > 5000000UL) return false;
  if (s > 5000UL || c < 10UL || c > 500UL) return false;

  freq_hz = static_cast<float>(f_millihz) / 1000.0f;
  settle_ms = static_cast<uint32_t>(s);
  capture_ms = static_cast<uint32_t>(c);
  return true;
}

static bool parsePrbsCommand(const String &line, uint32_t &order,
                             uint32_t &periods, uint32_t &amplitude_mv)
{
  char buf[96];
  line.toCharArray(buf, sizeof(buf));

  unsigned long o = 0, p = 0, a = 0;
  if (sscanf(buf, "PRBS %lu %lu %lu", &o, &p, &a) != 3) return false;

  // PRBS13 is deliberately fixed for the first implementation:
  // 8191 samples/period; 3 periods fit in the current buffers.
  if (o != 13UL) return false;
  if (p < 1UL || p > 3UL) return false;
  if (a < 50UL || a > 1400UL) return false;

  order = static_cast<uint32_t>(o);
  periods = static_cast<uint32_t>(p);
  amplitude_mv = static_cast<uint32_t>(a);
  return true;
}

static void sendPairs(const char tag[4], uint32_t actual_rate_hz,
                      uint32_t capture_count,
                      uint32_t extra0, uint32_t extra1, uint32_t extra2,
                      uint32_t extra_count)
{
  serialWriteAll(tag, 4);
  writeU32LE(actual_rate_hz);
  writeU32LE(capture_count);
  writeU32LE(extra0);
  if (extra_count >= 2U) writeU32LE(extra1);
  if (extra_count >= 3U) writeU32LE(extra2);

  // Send in larger chunks.  This is both faster and, together with
  // serialWriteAll(), prevents occasional truncated frames on USB CDC.
  uint8_t txbuf[256];
  size_t used = 0;

  for (uint32_t i = 0; i < capture_count; ++i) {
    const uint16_t a = vin_samples[i];
    const uint16_t b = vc_samples[i];

    txbuf[used++] = static_cast<uint8_t>(a);
    txbuf[used++] = static_cast<uint8_t>(a >> 8);
    txbuf[used++] = static_cast<uint8_t>(b);
    txbuf[used++] = static_cast<uint8_t>(b >> 8);

    if (used == sizeof(txbuf)) {
      serialWriteAll(txbuf, used);
      used = 0;
    }
  }

  if (used > 0) {
    serialWriteAll(txbuf, used);
  }
  Serial.flush();
}

static void runSineAndSend(float requested_freq_hz,
                           uint32_t settle_ms,
                           uint32_t capture_ms)
{
  uint32_t capture_count =
      (TARGET_SAMPLE_RATE_HZ * capture_ms) / 1000UL;
  if (capture_count < 16U) capture_count = 16U;
  if (capture_count > MAX_CAPTURE_SAMPLES) capture_count = MAX_CAPTURE_SAMPLES;

  const uint32_t settle_count =
      (TARGET_SAMPLE_RATE_HZ * settle_ms) / 1000UL;

  const double phase_scale = 4294967296.0 / static_cast<double>(TARGET_SAMPLE_RATE_HZ);
  uint32_t phase_inc = static_cast<uint32_t>(
      requested_freq_hz * phase_scale + 0.5);
  if (phase_inc == 0U) phase_inc = 1U;

  dacWriteFast(DAC_OFFSET_CODE);
  delay(20);

  uint32_t phase = 0;
  uint32_t next_us = micros();

  for (uint32_t i = 0; i < settle_count; ++i) {
    while (static_cast<int32_t>(micros() - next_us) < 0) {}
    dacWriteFast(ddsSample(phase));
    phase += phase_inc;
    next_us += SAMPLE_PERIOD_US;
  }

  const uint32_t start_us = micros();
  next_us = start_us;

  for (uint32_t i = 0; i < capture_count; ++i) {
    while (static_cast<int32_t>(micros() - next_us) < 0) {}

    dacWriteFast(ddsSample(phase));
    phase += phase_inc;

    vin_samples[i] = adcReadFast(ADC_CHANNEL_6);
    vc_samples[i]  = adcReadFast(ADC_CHANNEL_5);

    next_us += SAMPLE_PERIOD_US;
  }

  const uint32_t end_us = micros();
  dacWriteFast(DAC_OFFSET_CODE);

  const uint32_t elapsed_us = end_us - start_us;
  uint32_t actual_rate_hz = TARGET_SAMPLE_RATE_HZ;
  if (elapsed_us > 0U) {
    actual_rate_hz = static_cast<uint32_t>(
        ((uint64_t)capture_count * 1000000ULL + elapsed_us / 2ULL) /
        elapsed_us);
  }

  const double actual_freq_hz =
      (static_cast<double>(phase_inc) * static_cast<double>(actual_rate_hz)) /
      4294967296.0;
  const uint32_t actual_freq_millihz =
      static_cast<uint32_t>(actual_freq_hz * 1000.0 + 0.5);

  sendPairs("FRF1", actual_rate_hz, capture_count,
            actual_freq_millihz, 0, 0, 1);
}

// PRBS13 recurrence using x^13 + x^12 + x^11 + x^8 + 1.
// With nonzero initial state this yields the full 8191-sample period.
static inline uint16_t prbs13Step(uint16_t &state)
{
  const uint16_t out = state & 1U;
  const uint16_t new_bit = static_cast<uint16_t>(
      ((state >> 12) ^ (state >> 11) ^ (state >> 10) ^ (state >> 7)) & 1U);
  state = static_cast<uint16_t>(((state << 1) & 0x1FFFU) | new_bit);
  return out;
}

static inline uint16_t prbsDacCode(uint16_t bit, uint16_t amplitude_code)
{
  int32_t code = static_cast<int32_t>(DAC_OFFSET_CODE);
  code += bit ? static_cast<int32_t>(amplitude_code)
              : -static_cast<int32_t>(amplitude_code);
  if (code < 0) code = 0;
  if (code > DAC_MAX) code = DAC_MAX;
  return static_cast<uint16_t>(code);
}

static void runPrbsAndSend(uint32_t order, uint32_t periods,
                           uint32_t amplitude_mv)
{
  const uint32_t period_len = (1UL << order) - 1UL; // 8191 for order 13
  const uint32_t capture_count = period_len * periods;
  if (capture_count > MAX_CAPTURE_SAMPLES) return;

  uint32_t amp_code_u32 =
      (amplitude_mv * static_cast<uint32_t>(DAC_MAX) + 1650UL) / 3300UL;
  if (amp_code_u32 > DAC_MAX) amp_code_u32 = DAC_MAX;
  const uint16_t amplitude_code = static_cast<uint16_t>(amp_code_u32);

  dacWriteFast(DAC_OFFSET_CODE);
  delay(20);

  // Run one complete PRBS period before capture so the RLC is in periodic
  // steady state. Restart the state before capture so all captured periods
  // are exactly aligned and directly averageable on the host.
  uint16_t state = 0x1FFFU;
  uint32_t next_us = micros();
  for (uint32_t i = 0; i < period_len; ++i) {
    while (static_cast<int32_t>(micros() - next_us) < 0) {}
    const uint16_t bit = prbs13Step(state);
    dacWriteFast(prbsDacCode(bit, amplitude_code));
    next_us += SAMPLE_PERIOD_US;
  }

  state = 0x1FFFU;
  const uint32_t start_us = micros();
  next_us = start_us;

  for (uint32_t i = 0; i < capture_count; ++i) {
    while (static_cast<int32_t>(micros() - next_us) < 0) {}

    const uint16_t bit = prbs13Step(state);
    dacWriteFast(prbsDacCode(bit, amplitude_code));

    vin_samples[i] = adcReadFast(ADC_CHANNEL_6);
    vc_samples[i]  = adcReadFast(ADC_CHANNEL_5);

    next_us += SAMPLE_PERIOD_US;
  }

  const uint32_t end_us = micros();
  dacWriteFast(DAC_OFFSET_CODE);

  const uint32_t elapsed_us = end_us - start_us;
  uint32_t actual_rate_hz = TARGET_SAMPLE_RATE_HZ;
  if (elapsed_us > 0U) {
    actual_rate_hz = static_cast<uint32_t>(
        ((uint64_t)capture_count * 1000000ULL + elapsed_us / 2ULL) /
        elapsed_us);
  }

  sendPairs("PRB1", actual_rate_hz, capture_count,
            order, periods, amplitude_mv, 3);
}

void setup()
{
  analogWriteResolution(12);
  analogWrite(PIN_DAC, DAC_OFFSET_CODE);
  initSineLut();
  initAdc1Fast();

  Serial.begin(115200);
  delay(500);
}

void loop()
{
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      command.trim();
      if (command.length() > 0) {
        float f;
        uint32_t settle_ms, capture_ms;
        uint32_t order, periods, amplitude_mv;

        if (parseSineCommand(command, f, settle_ms, capture_ms)) {
          runSineAndSend(f, settle_ms, capture_ms);
        } else if (parsePrbsCommand(command, order, periods, amplitude_mv)) {
          runPrbsAndSend(order, periods, amplitude_mv);
        }
      }
      command = "";
    } else if (command.length() < 90) {
      command += c;
    }
  }
}
