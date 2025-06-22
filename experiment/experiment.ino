#include "FspTimer.h"
#include <SoftwareSerial.h>

#define SAMPLE_RATE 16000
#define NUM_SAMPLE 2000

SoftwareSerial mySerial(3, 2);

float f = 261.63;
float amp = 0.6;
uint16_t waveform[NUM_SAMPLE];
float phase = 0.0;
float phaseInc = 0.0;

FspTimer timer;
byte b[2];
int sendData = 0;

const float doremiHz[] = {
  261.63, 293.66, 329.63, 349.23,
  392.00, 440.00, 493.88, 523.25
};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

// --- ソフトクリップ ---
float softClip(float x) {
  if (x > 0.7) return 0.7 + (x - 0.7) * 0.3;
  else if (x < -0.7) return -0.7 + (x + 0.7) * 0.3;
  else return x;
}

// --- ローパスフィルタ ---
void lowPassFilter(float *samples, int len, int passes = 2) {
  float tmp[len];
  for (int p = 0; p < passes; p++) {
    tmp[0] = (samples[0] + samples[1]) * 0.5;
    for (int i = 1; i < len - 1; i++) {
      tmp[i] = (samples[i - 1] + samples[i] + samples[i + 1]) / 3.0;
    }
    tmp[len - 1] = (samples[len - 2] + samples[len - 1]) * 0.5;
    for (int i = 0; i < len; i++) samples[i] = tmp[i];
  }
}

// --- 音色強調フィルタ群 ---
void emphasizeThickness(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.10 * sin(2.0 * PI * 3000 * i / SAMPLE_RATE); // 2500-3400Hz
  }
}

void boostHighs(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.05 * sin(2.0 * PI * 10000 * i / SAMPLE_RATE); // 9000Hz+
  }
}

void addBrightnessNoise(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.03 * sin(2.0 * PI * 7200 * i / SAMPLE_RATE); // 6459〜8182Hz
  }
}

void maskMidSharpness(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] -= 0.02 * sin(2.0 * PI * 6000 * i / SAMPLE_RATE); // 5167〜6890Hz
  }
}

// --- 波形生成 ---
void generateTrumpetWave(float freq) {
  float a1 = 1.0, a2 = 0.5, a3 = 0.3, a4 = 0.15, a5 = 0.1;
  float modIndex = 3.0;
  float modFreqRatio = 2.5;
  static float tempSamples[NUM_SAMPLE];

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;
    float angle = 2.0 * PI * t;

    float modulator = sin(modFreqRatio * angle);
    float fmWave = sin(angle + modIndex * modulator);

    float harmonics =
      a1 * sin(angle) +
      a2 * sin(2 * angle) +
      a3 * sin(3 * angle) +
      a4 * sin(4 * angle) +
      a5 * sin(5 * angle);

    float env = 1.0;
    if (t < 0.02) env = t / 0.02;

    float sample = 0.2 * fmWave + 0.8 * harmonics * env;
    sample = softClip(sample);
    tempSamples[i] = sample;
  }

  // --- 音色変化反映 ---
  emphasizeThickness(tempSamples, NUM_SAMPLE);   // 太さ
  boostHighs(tempSamples, NUM_SAMPLE);           // 明るさ・鋭さ
  addBrightnessNoise(tempSamples, NUM_SAMPLE);   // キンキン・鼓膜
  maskMidSharpness(tempSamples, NUM_SAMPLE);     // 鋭さ補正
  lowPassFilter(tempSamples, NUM_SAMPLE, 2);     // 総仕上げ

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float s = tempSamples[i];
    int tmpVal = (int)((s * amp + 1.0) * 2047.5);
    tmpVal = constrain(tmpVal, 0, 4095);
    waveform[i] = (uint16_t)tmpVal;
  }

  float vibrato = 1.0 + 0.003 * sin(2 * PI * millis() / 500.0);
  phaseInc = freq * vibrato * NUM_SAMPLE / SAMPLE_RATE;
}

// --- 再生用コールバック ---
void callback_playSound(timer_callback_args_t *arg) {
  int index = (int)phase;
  float frac = phase - index;

  uint16_t v1 = waveform[index % NUM_SAMPLE];
  uint16_t v2 = waveform[(index + 1) % NUM_SAMPLE];
  uint16_t value = (uint16_t)((1.0 - frac) * v1 + frac * v2);

  b[0] = value >> 8;
  b[1] = value & 0xFF;
  analogWrite(A0, value);

  phase += phaseInc;
  if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;

  sendData = 1;
}

void setup() {
  analogWriteResolution(12);
  Serial.begin(115200);
  mySerial.begin(115200);

  generateTrumpetWave(f);

  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;

  timer.begin(TIMER_MODE_PERIODIC, type, ch, SAMPLE_RATE, 50.0f, callback_playSound, nullptr);
  timer.setup_overflow_irq();
  timer.open();
  timer.start();
}

void loop() {
  static unsigned long lastMillis = 0;
  static int idx = 0;
  unsigned long now = millis();

  if (now - lastMillis > 1000) {
    lastMillis = now;
    f = doremiHz[idx];
    idx = (idx + 1) % numNotes;
    generateTrumpetWave(f);
  }

  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);
  }
}
