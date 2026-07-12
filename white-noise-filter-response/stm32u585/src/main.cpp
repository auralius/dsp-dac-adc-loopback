#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ============================================================
// STM32 DAC-ADC Loopback DSP Test Bench
//
// Modes:
//   RUN   <freq_hz> <N> <settle_N>
//   NOISE <N> <settle_N> [seed]
//
// RUN mode:
//   Generates differential sine -> ADC -> filter -> stores x[n], y[n]
//
// NOISE mode:
//   Generates differential uniform white noise -> ADC -> filter -> stores x[n], y[n]
//
// Wiring:
//   PA4 / A4  -> PA0 / A0
//   PA5 / A5  -> PA1 / A1
//   GND       -> GND
// ============================================================


// ============================================================
// Pin configuration
// ============================================================

const uint8_t PIN_DAC_P = PA4;
const uint8_t PIN_DAC_N = PA5;

const uint8_t PIN_ADC_P = A0;
const uint8_t PIN_ADC_N = A1;


// ============================================================
// Sampling configuration
// ============================================================

const float FS_HZ = 1000.0f;
const uint32_t TS_US = (uint32_t)(1000000.0f / FS_HZ);

const int DAC_MID = 2048;
const int DAC_AMP = 900;

// STM32U585 has enough RAM for this.
// Two float buffers use 2 * MAX_N * 4 bytes.
// For MAX_N=65536, this is 524288 bytes.
const int MAX_N = 65536;

float x_buf[MAX_N];
float y_buf[MAX_N];

// Slow down text transfer after capture.
// This does not affect sampling accuracy because sending happens after capture.
// Set to 0 if serial transfer is stable without delay.
const int SEND_DELAY_MS = 1;


// ============================================================
// Random number generator for uniform white noise
// ============================================================

uint32_t rng_state = 123456789UL;

void rng_seed(uint32_t seed)
{
  if (seed == 0) {
    seed = 123456789UL;
  }
  rng_state = seed;
}

uint32_t xorshift32()
{
  uint32_t x = rng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rng_state = x;
  return x;
}

float uniform_noise_pm1()
{
  // Convert uint32 to approximately uniform float in [-1, +1].
  // This is white in the sense that consecutive samples are pseudo-random
  // and have approximately flat broadband spectral content.
  uint32_t r = xorshift32();
  float u = (float)r / 4294967295.0f;  // 0..1
  return 2.0f * u - 1.0f;              // -1..+1
}


// ============================================================
// Filter implementation
//
// Important:
// - run_filter(x) is called at FS_HZ.
// - Filter coefficients must be designed for this exact FS_HZ.
// - reset_filter() must clear all filter states before every test.
// ============================================================

// Example: 4-section SOS Butterworth band-pass
// Fs = 1000 Hz
// Band-pass = 0.4 Hz to 50 Hz
//
// scipy:
// signal.butter(N=4, Wn=[0.4, 50.0], btype="bandpass",
//               fs=1000.0, output="sos")
//
// Overall digital band-pass order is 8 because band-pass doubles the order.
// This is implemented as 4 cascaded biquad sections.

const int NUM_SECTIONS = 4;

// Each row: b0, b1, b2, a0, a1, a2
// a0 is assumed to be 1.0.
const float sos[NUM_SECTIONS][6] = {
  { 0.000404559f,  0.000809118f,  0.000404559f,  1.000000000f, -1.486732790f,  0.561129068f },
  { 1.000000000f,  2.000000000f,  1.000000000f,  1.000000000f, -1.704174510f,  0.791468303f },
  { 1.000000000f, -2.000000000f,  1.000000000f,  1.000000000f, -1.995308100f,  0.995314576f },
  { 1.000000000f, -2.000000000f,  1.000000000f,  1.000000000f, -1.998093640f,  0.998099981f }
};

// Direct Form II Transposed states.
// One pair of states per SOS section.
float w1[NUM_SECTIONS] = {0.0f};
float w2[NUM_SECTIONS] = {0.0f};

void reset_filter()
{
  for (int i = 0; i < NUM_SECTIONS; i++) {
    w1[i] = 0.0f;
    w2[i] = 0.0f;
  }
}

float run_filter(float x)
{
  float y = x;

  for (int i = 0; i < NUM_SECTIONS; i++) {
    const float b0 = sos[i][0];
    const float b1 = sos[i][1];
    const float b2 = sos[i][2];
    const float a1 = sos[i][4];
    const float a2 = sos[i][5];

    float out = b0 * y + w1[i];
    w1[i] = b1 * y - a1 * out + w2[i];
    w2[i] = b2 * y - a2 * out;

    y = out;
  }

  return y;
}


// ============================================================
// Helpers
// ============================================================

int clamp_dac(int v)
{
  if (v < 0) return 0;
  if (v > 4095) return 4095;
  return v;
}

void dac_write_differential_from_unit(float s)
{
  // s should be in [-1, +1].
  int dac_p = DAC_MID + (int)(DAC_AMP * s);
  int dac_n = DAC_MID - (int)(DAC_AMP * s);

  dac_p = clamp_dac(dac_p);
  dac_n = clamp_dac(dac_n);

  analogWrite(PIN_DAC_P, dac_p);
  analogWrite(PIN_DAC_N, dac_n);
}

float read_differential_adc()
{
  int adc_p = analogRead(PIN_ADC_P);
  int adc_n = analogRead(PIN_ADC_N);
  return (float)(adc_p - adc_n);
}

void wait_until_next_sample(uint32_t &next_t, int &missed_deadlines)
{
  next_t += TS_US;

  while ((int32_t)(micros() - next_t) < 0) {
    // busy wait
  }

  if ((int32_t)(micros() - next_t) > (int32_t)TS_US) {
    missed_deadlines++;
  }
}

void sanitize_N(int &N, int &settle_N)
{
  if (N > MAX_N) N = MAX_N;
  if (N < 16) N = 16;
  if (settle_N < 0) settle_N = 0;
}

float compute_actual_fs(uint32_t t0, uint32_t t1, int N)
{
  uint32_t dt = t1 - t0;
  if (dt == 0) return 0.0f;
  return 1000000.0f * (float)N / (float)dt;
}

void stop_dac_midscale()
{
  analogWrite(PIN_DAC_P, DAC_MID);
  analogWrite(PIN_DAC_N, DAC_MID);
}

void send_captured_data(
  const char *mode,
  float effective_f_hz,
  float actual_fs,
  int N,
  int missed_deadlines
)
{
  // Protocol:
  // BEGIN,<mode>,<effective_f_hz>,<actual_fs>,<N>,<missed_deadlines>
  // DATA,n,x,y
  // ...
  // END
  //
  // For sine mode, effective_f_hz is the actual generated sine frequency.
  // For noise mode, effective_f_hz is 0.0.

  Serial.print("BEGIN,");
  Serial.print(mode);
  Serial.print(",");
  Serial.print(effective_f_hz, 6);
  Serial.print(",");
  Serial.print(actual_fs, 6);
  Serial.print(",");
  Serial.print(N);
  Serial.print(",");
  Serial.println(missed_deadlines);

  for (int n = 0; n < N; n++) {
    Serial.print("DATA,");
    Serial.print(n);
    Serial.print(",");
    Serial.print(x_buf[n], 6);
    Serial.print(",");
    Serial.println(y_buf[n], 6);

    if (SEND_DELAY_MS > 0) {
      delay(SEND_DELAY_MS);
    }
  }

  Serial.println("END");
  Serial.flush();
}


// ============================================================
// RUN mode: sine sweep capture
// ============================================================

void run_capture_sine(float f_hz, int N, int settle_N)
{
  sanitize_N(N, settle_N);
  reset_filter();

  float phase = 0.0f;
  float dphase = 2.0f * PI * f_hz / FS_HZ;

  int missed_deadlines = 0;

  stop_dac_midscale();
  delay(100);

  uint32_t next_t = micros();

  // Settling samples, not stored.
  for (int n = 0; n < settle_N; n++) {
    wait_until_next_sample(next_t, missed_deadlines);

    float s = sinf(phase);
    dac_write_differential_from_unit(s);

    float x = read_differential_adc();
    run_filter(x);

    phase += dphase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
  }

  // Captured samples.
  uint32_t t0 = micros();

  for (int n = 0; n < N; n++) {
    wait_until_next_sample(next_t, missed_deadlines);

    float s = sinf(phase);
    dac_write_differential_from_unit(s);

    float x = read_differential_adc();
    float y = run_filter(x);

    x_buf[n] = x;
    y_buf[n] = y;

    phase += dphase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
  }

  uint32_t t1 = micros();

  float actual_fs = compute_actual_fs(t0, t1, N);
  float effective_f_hz = f_hz * actual_fs / FS_HZ;

  stop_dac_midscale();
  send_captured_data("SINE", effective_f_hz, actual_fs, N, missed_deadlines);
}


// ============================================================
// NOISE mode: uniform white-noise capture
// ============================================================

void run_capture_noise(int N, int settle_N, uint32_t seed)
{
  sanitize_N(N, settle_N);
  reset_filter();
  rng_seed(seed);

  int missed_deadlines = 0;

  stop_dac_midscale();
  delay(100);

  uint32_t next_t = micros();

  // Settling samples, not stored.
  for (int n = 0; n < settle_N; n++) {
    wait_until_next_sample(next_t, missed_deadlines);

    float s = uniform_noise_pm1();
    dac_write_differential_from_unit(s);

    float x = read_differential_adc();
    run_filter(x);
  }

  // Captured samples.
  uint32_t t0 = micros();

  for (int n = 0; n < N; n++) {
    wait_until_next_sample(next_t, missed_deadlines);

    float s = uniform_noise_pm1();
    dac_write_differential_from_unit(s);

    float x = read_differential_adc();
    float y = run_filter(x);

    x_buf[n] = x;
    y_buf[n] = y;
  }

  uint32_t t1 = micros();

  float actual_fs = compute_actual_fs(t0, t1, N);

  stop_dac_midscale();
  send_captured_data("NOISE", 0.0f, actual_fs, N, missed_deadlines);
}


// ============================================================
// Command parser helpers
// ============================================================

int split_tokens(String line, String tokens[], int max_tokens)
{
  line.trim();
  line.replace('\t', ' ');

  while (line.indexOf("  ") >= 0) {
    line.replace("  ", " ");
  }

  int count = 0;

  while (line.length() > 0 && count < max_tokens) {
    int p = line.indexOf(' ');

    if (p < 0) {
      tokens[count++] = line;
      break;
    }

    tokens[count++] = line.substring(0, p);
    line = line.substring(p + 1);
    line.trim();
  }

  return count;
}

void handle_run_command(String tokens[], int ntok)
{
  if (ntok < 2) {
    Serial.println("ERR,bad_RUN_format");
    return;
  }

  float f_hz = tokens[1].toFloat();
  int N = (ntok >= 3) ? tokens[2].toInt() : 4096;
  int settle_N = (ntok >= 4) ? tokens[3].toInt() : 1000;

  if (f_hz <= 0.0f) {
    Serial.println("ERR,bad_frequency");
    return;
  }

  if (N <= 0) N = 4096;
  if (settle_N < 0) settle_N = 1000;

  Serial.print("OK,RUN,");
  Serial.print(f_hz, 6);
  Serial.print(",");
  Serial.print(N);
  Serial.print(",");
  Serial.println(settle_N);

  run_capture_sine(f_hz, N, settle_N);
}

void handle_noise_command(String tokens[], int ntok)
{
  if (ntok < 2) {
    Serial.println("ERR,bad_NOISE_format");
    return;
  }

  int N = tokens[1].toInt();
  int settle_N = (ntok >= 3) ? tokens[2].toInt() : 1000;
  uint32_t seed = (ntok >= 4) ? (uint32_t)tokens[3].toInt() : 123456789UL;

  if (N <= 0) N = 4096;
  if (settle_N < 0) settle_N = 1000;

  Serial.print("OK,NOISE,");
  Serial.print(N);
  Serial.print(",");
  Serial.print(settle_N);
  Serial.print(",");
  Serial.println(seed);

  run_capture_noise(N, settle_N, seed);
}

void print_clocks()
{
  Serial.println("=== CLOCK INFO ===");

  Serial.print("F_CPU macro        = ");
  Serial.println(F_CPU);

  Serial.print("SystemCoreClock    = ");
  Serial.println(SystemCoreClock);

#ifdef HAL_RCC_MODULE_ENABLED
  Serial.print("HAL SYSCLK         = ");
  Serial.println(HAL_RCC_GetSysClockFreq());

  Serial.print("HAL HCLK           = ");
  Serial.println(HAL_RCC_GetHCLKFreq());

  Serial.print("HAL PCLK1          = ");
  Serial.println(HAL_RCC_GetPCLK1Freq());

  Serial.print("HAL PCLK2          = ");
  Serial.println(HAL_RCC_GetPCLK2Freq());
#endif

  Serial.println("==================");
}

void handle_command(String line)
{
  String tokens[8];
  int ntok = split_tokens(line, tokens, 8);

  if (ntok <= 0) {
    return;
  }

  tokens[0].toUpperCase();

  if (tokens[0] == "RUN") {
    handle_run_command(tokens, ntok);
    return;
  }

  if (tokens[0] == "NOISE") {
    handle_noise_command(tokens, ntok);
    return;
  }

  if (tokens[0] == "CLOCK") {
    print_clocks();
    return;
  }

  Serial.println("ERR,unknown_command");
}


// ============================================================
// Arduino setup / loop
// ============================================================

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(2000);

  analogWriteResolution(12);
  analogReadResolution(12);

  pinMode(PIN_DAC_P, OUTPUT);
  pinMode(PIN_DAC_N, OUTPUT);
  pinMode(PIN_ADC_P, INPUT_ANALOG);
  pinMode(PIN_ADC_N, INPUT_ANALOG);

  stop_dac_midscale();

  Serial.println("READY");
  Serial.println("Commands:");
  Serial.println("  RUN <freq_hz> <N> <settle_N>");
  Serial.println("  NOISE <N> <settle_N> [seed]");
  Serial.println("  CLOCK");
}

void loop()
{
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handle_command(line);
  }
}
