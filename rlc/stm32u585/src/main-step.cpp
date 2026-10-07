#include <Arduino.h>
#include <stdint.h>

/*
  STM32U585 RLC step-response capture — fast ADC, no DMA

  Wiring:
    PA4 / DAC1_OUT1 ----+---- 470 ohm ---- 250 mH ----+---- Vc
                        |                              |
    PA1 / ADC1_IN6 ------+                            470 nF
      measure Vin                                      |
                                                       GND

    PA0 / ADC1_IN5 -------------------------------- Vc
      measure Vc

  Host command:
    CAPTURE\n

  Binary record (little endian):
    bytes 0..3   ASCII "RLC2"
    bytes 4..7   uint32 measured average sample-pair rate (Hz)
    bytes 8..11  uint32 sample count per channel
    then, for each sample n:
      uint16 Vin ADC code
      uint16 Vc  ADC code

  Key change from the slow polling build:
    - ADC1 is initialized only once.
    - PA1 and PA0 sampling time is configured only once.
    - Each sample uses direct ADC register channel selection + conversion.
    - No analogRead() in the acquisition loop.
    - No DMA.

  Target: 50,000 two-channel sample pairs/s (20 us per pair).
*/

constexpr uint8_t PIN_DAC     = PA4;
constexpr uint8_t PIN_ADC_VIN = PA1;
constexpr uint8_t PIN_ADC_VC  = PA0;

constexpr uint32_t TARGET_SAMPLE_RATE_HZ = 50000;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / TARGET_SAMPLE_RATE_HZ; // 20 us
constexpr uint32_t CAPTURE_MS = 100;
constexpr uint32_t SAMPLE_COUNT =
    (TARGET_SAMPLE_RATE_HZ * CAPTURE_MS) / 1000UL; // 5000 pairs

constexpr float DAC_REFERENCE_V = 3.3f;
constexpr float STEP_V = 2.0f;
constexpr uint16_t DAC_MAX = 4095;
constexpr uint16_t DAC_STEP_CODE =
    static_cast<uint16_t>((STEP_V / DAC_REFERENCE_V) * DAC_MAX + 0.5f);

uint16_t vin_samples[SAMPLE_COUNT];
uint16_t vc_samples[SAMPLE_COUNT];
String command;

ADC_HandleTypeDef hadc1;

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

static void writeU32LE(uint32_t value)
{
  const uint8_t bytes[4] = {
    static_cast<uint8_t>(value),
    static_cast<uint8_t>(value >> 8),
    static_cast<uint8_t>(value >> 16),
    static_cast<uint8_t>(value >> 24)
  };
  Serial.write(bytes, sizeof(bytes));
}

static void writeU16LE(uint16_t value)
{
  const uint8_t bytes[2] = {
    static_cast<uint8_t>(value),
    static_cast<uint8_t>(value >> 8)
  };
  Serial.write(bytes, sizeof(bytes));
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
  // The previous conversion has completed before this function is entered.
  // Change the single regular rank to the desired channel.
  adcSelectChannel(hal_channel);

  // EOC/EOS/OVR are cleared by writing 1 on STM32 ADCs.
  ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;

  // ADC is already enabled; start one software-triggered conversion.
  SET_BIT(ADC1->CR, ADC_CR_ADSTART);

  while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {
    // Conversion itself is only a small fraction of our 20 us pair period.
  }

  // Reading DR obtains the conversion result.
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

  if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) {
    fatalBlink();
  }
}

static void initAdc1Fast()
{
  // Let STM32duino configure the GPIOs in analog mode.
  pinMode(PIN_ADC_VIN, INPUT_ANALOG);
  pinMode(PIN_ADC_VC, INPUT_ANALOG);

  // STM32U5 analog supply and ADC clock.
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

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    fatalBlink();
  }

  // Configure the sampling-time register for both external channels once.
  configureAdcChannel(ADC_CHANNEL_5); // PA0 = Vc
  configureAdcChannel(ADC_CHANNEL_6); // PA1 = Vin

  if (HAL_ADCEx_Calibration_Start(
          &hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
    fatalBlink();
  }

  // Warm-up conversion. HAL_ADC_Start enables ADC1 and starts one conversion.
  adcSelectChannel(ADC_CHANNEL_5);
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    fatalBlink();
  }
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) {
    fatalBlink();
  }
  (void)HAL_ADC_GetValue(&hadc1);

  // Intentionally do NOT call HAL_ADC_Stop(): keeping ADC1 enabled is what
  // makes the register-level reads fast.
}

static void captureAndSend()
{
  // Return the plant to zero-input equilibrium before each trial.
  analogWrite(PIN_DAC, 0);
  delay(30);

  // Apply the physical input step.
  analogWrite(PIN_DAC, DAC_STEP_CODE);

  const uint32_t start_us = micros();
  uint32_t next_sample_us = start_us;

  for (uint32_t i = 0; i < SAMPLE_COUNT; ++i) {
    while (static_cast<int32_t>(micros() - next_sample_us) < 0) {
      // Busy-wait for the requested 20 us sample-pair instant.
    }

    // PA1 first, then PA0, matching the GUI/CSV ordering.
    vin_samples[i] = adcReadFast(ADC_CHANNEL_6);
    vc_samples[i]  = adcReadFast(ADC_CHANNEL_5);

    next_sample_us += SAMPLE_PERIOD_US;
  }

  const uint32_t end_us = micros();
  analogWrite(PIN_DAC, 0);

  const uint32_t elapsed_us = end_us - start_us;
  uint32_t actual_rate_hz = TARGET_SAMPLE_RATE_HZ;
  if (elapsed_us > 0U) {
    actual_rate_hz = static_cast<uint32_t>(
        ((uint64_t)SAMPLE_COUNT * 1000000ULL + elapsed_us / 2ULL) /
        elapsed_us);
  }

  // USB CDC happens only after acquisition is complete.
  Serial.write("RLC2", 4);
  writeU32LE(actual_rate_hz);
  writeU32LE(SAMPLE_COUNT);

  for (uint32_t i = 0; i < SAMPLE_COUNT; ++i) {
    writeU16LE(vin_samples[i]);
    writeU16LE(vc_samples[i]);
  }
}

void setup()
{
  analogWriteResolution(12);

  // PA4 maps to the true DAC output on the present U585 board definition.
  analogWrite(PIN_DAC, 0);

  initAdc1Fast();

  Serial.begin(115200); // USB CDC; baud is nominal.
  delay(500);
}

void loop()
{
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());

    if (c == '\n' || c == '\r') {
      command.trim();

      if (command.equalsIgnoreCase("CAPTURE")) {
        captureAndSend();
      }

      command = "";
    } else if (command.length() < 32) {
      command += c;
    } else {
      command = "";
    }
  }
}
