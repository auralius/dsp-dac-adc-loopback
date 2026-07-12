#include <Arduino.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// ============================================================
// Pin configuration
// Wiring:
// PA4 / A4  -> PA0 / A0
// PA5 / A5  -> PA1 / A1
// GND       -> GND
// ============================================================

const uint8_t PIN_DAC_P = PA4;
const uint8_t PIN_DAC_N = PA5;

const uint8_t PIN_ADC_P = A0;
const uint8_t PIN_ADC_N = A1;

// ============================================================
// Sampling configuration
//
// Important:
// Arduino analogWrite + analogRead is not fast enough for 5000 Hz
// on this setup. Start with 1000 Hz for better timing accuracy.
// ============================================================

const float FS_HZ = 1000.0f;
const uint32_t TS_US = (uint32_t)(1000000.0f / FS_HZ);

const int DAC_MID = 2048;
const int DAC_AMP = 900;

// Maximum captured samples
const int MAX_N = 32768;

float x_buf[MAX_N];
float y_buf[MAX_N];

// Slow down text transfer after capture.
// This does not affect sampling accuracy because sending happens after capture.
const int SEND_DELAY_MS = 1;

// ============================================================
// Replace this section with the real YAPID filter later.
// ============================================================

float w1_1 = 0.0f;
float w2_1 = 0.0f;

float w1_2 = 0.0f;
float w2_2 = 0.0f;

void reset_filter()
{
  w1_1 = 0.0f;
  w2_1 = 0.0f;
  w1_2 = 0.0f;
  w2_2 = 0.0f;
}

/*
float run_filter(float x)
{
  // Later:
  // return my_filter.update(x);

  // Temporary first-order low-pass for testing:
  const float alpha = 0.05f;
  y_state += alpha * (x - y_state);
  return y_state;
}
*/

float run_filter(float x)
{
  const float b0 =  0.111622f;
  const float b1 =  0.000000f;
  const float b2 = -0.111622f;
  const float a1 = -1.765409f;
  const float a2 =  0.776755f;

  float y1 = (b0 * x) + w1_1;
  w1_1 = (b1 * x) - (a1 * y1) + w2_1;
  w2_1 = (b2 * x) - (a2 * y1);

  float y2 = (b0 * y1) + w1_2;
  w1_2 = (b1 * y1) - (a1 * y2) + w2_2;
  w2_2 = (b2 * y1) - (a2 * y2);

  return y2;
}

// ============================================================
// Capture one frequency
// ============================================================

void run_capture(float f_hz, int N, int settle_N)
{
  if (N > MAX_N) N = MAX_N;
  if (N < 16) N = 16;
  if (settle_N < 0) settle_N = 0;

  reset_filter();

  float phase = 0.0f;
  float dphase = 2.0f * PI * f_hz / FS_HZ;

  int missed_deadlines = 0;

  // Put DACs at midscale first
  analogWrite(PIN_DAC_P, DAC_MID);
  analogWrite(PIN_DAC_N, DAC_MID);
  delay(100);

  uint32_t next_t = micros();

  // ============================================================
  // Settling samples, not stored
  // ============================================================

  for (int n = 0; n < settle_N; n++) {
    next_t += TS_US;

    while ((int32_t)(micros() - next_t) < 0) {
      // wait
    }

    if ((int32_t)(micros() - next_t) > (int32_t)TS_US) {
      missed_deadlines++;
    }

    float s = sinf(phase);

    int dac_p = DAC_MID + (int)(DAC_AMP * s);
    int dac_n = DAC_MID - (int)(DAC_AMP * s);

    if (dac_p < 0) dac_p = 0;
    if (dac_p > 4095) dac_p = 4095;
    if (dac_n < 0) dac_n = 0;
    if (dac_n > 4095) dac_n = 4095;

    analogWrite(PIN_DAC_P, dac_p);
    analogWrite(PIN_DAC_N, dac_n);

    int adc_p = analogRead(PIN_ADC_P);
    int adc_n = analogRead(PIN_ADC_N);

    float x = (float)(adc_p - adc_n);
    run_filter(x);

    phase += dphase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
  }

  // ============================================================
  // Captured samples
  // ============================================================

  uint32_t t0 = micros();

  for (int n = 0; n < N; n++) {
    next_t += TS_US;

    while ((int32_t)(micros() - next_t) < 0) {
      // wait
    }

    if ((int32_t)(micros() - next_t) > (int32_t)TS_US) {
      missed_deadlines++;
    }

    float s = sinf(phase);

    int dac_p = DAC_MID + (int)(DAC_AMP * s);
    int dac_n = DAC_MID - (int)(DAC_AMP * s);

    if (dac_p < 0) dac_p = 0;
    if (dac_p > 4095) dac_p = 4095;
    if (dac_n < 0) dac_n = 0;
    if (dac_n > 4095) dac_n = 4095;

    analogWrite(PIN_DAC_P, dac_p);
    analogWrite(PIN_DAC_N, dac_n);

    int adc_p = analogRead(PIN_ADC_P);
    int adc_n = analogRead(PIN_ADC_N);

    float x = (float)(adc_p - adc_n);
    float y = run_filter(x);

    x_buf[n] = x;
    y_buf[n] = y;

    phase += dphase;
    if (phase >= 2.0f * PI) phase -= 2.0f * PI;
  }

  uint32_t t1 = micros();

  float actual_fs = 1000000.0f * (float)N / (float)(t1 - t0);

  // If the loop cannot exactly hit FS_HZ, the actual generated sine
  // frequency is scaled by actual_fs / FS_HZ.
  float effective_f_hz = f_hz * actual_fs / FS_HZ;

  // Stop DAC at midscale
  analogWrite(PIN_DAC_P, DAC_MID);
  analogWrite(PIN_DAC_N, DAC_MID);

  // ============================================================
  // Send data after capture
  // Protocol:
  // BEGIN,effective_f_hz,actual_fs,N,missed_deadlines
  // DATA,n,x,y
  // ...
  // END
  // ============================================================

  Serial.print("BEGIN,");
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

    delay(SEND_DELAY_MS);
  }

  Serial.println("END");
  Serial.flush();
}

// ============================================================
// Robust command parser
//
// Format:
// RUN 100.0 4096 1000
// ============================================================

void handle_command(String line)
{
  line.trim();
  line.replace('\t', ' ');

  while (line.indexOf("  ") >= 0) {
    line.replace("  ", " ");
  }

  if (!line.startsWith("RUN")) {
    Serial.println("ERR,unknown_command");
    return;
  }

  int p1 = line.indexOf(' ');
  if (p1 < 0) {
    Serial.println("ERR,bad_RUN_format");
    return;
  }

  int p2 = line.indexOf(' ', p1 + 1);
  int p3 = -1;

  String f_str;
  String n_str;
  String settle_str;

  if (p2 < 0) {
    f_str = line.substring(p1 + 1);
  } else {
    f_str = line.substring(p1 + 1, p2);
    p3 = line.indexOf(' ', p2 + 1);

    if (p3 < 0) {
      n_str = line.substring(p2 + 1);
    } else {
      n_str = line.substring(p2 + 1, p3);
      settle_str = line.substring(p3 + 1);
    }
  }

  f_str.trim();
  n_str.trim();
  settle_str.trim();

  float f_hz = f_str.toFloat();
  int N = n_str.length() ? n_str.toInt() : 4096;
  int settle_N = settle_str.length() ? settle_str.toInt() : 1000;

  if (f_hz <= 0.0f) {
    Serial.println("ERR,bad_frequency");
    return;
  }

  if (N <= 0) N = 4096;
  if (settle_N < 0) settle_N = 1000;

  Serial.print("OK,");
  Serial.print(f_hz, 6);
  Serial.print(",");
  Serial.print(N);
  Serial.print(",");
  Serial.println(settle_N);

  run_capture(f_hz, N, settle_N);
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

  analogWrite(PIN_DAC_P, DAC_MID);
  analogWrite(PIN_DAC_N, DAC_MID);

  Serial.println("READY");
  Serial.println("Command: RUN <freq_hz> <N> <settle_N>");
}

void loop()
{
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handle_command(line);
  }
}