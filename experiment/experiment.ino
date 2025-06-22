#include "FspTimer.h"
#include <SoftwareSerial.h>

#define SAMPLE_RATE 16000       // サンプリング周波数（Hz）
#define NUM_SAMPLE 800          // 波形1周期のサンプル数

SoftwareSerial mySerial(3, 2);   // ソフトウェアシリアル（ピン3=RX, ピン2=TX）

float f = 233.08;                // 初期周波数（シ♭3、B♭管トランペットの「ド」）
float amp = 0.6;                 // 振幅
uint16_t waveform[NUM_SAMPLE];   // 12bit波形バッファ
float phase = 0.0;               // 現在の位相
float phaseInc = 0.0;            // 1サンプルごとの位相の進み量

FspTimer timer;                  // FSPタイマーインスタンス
byte b[2];                      // シリアル送信用バッファ
int sendData = 0;               // シリアル送信フラグ

// 元のドレミファソラシド（C4〜C5）の周波数
const float doremiHz[] = {
261.63 * 2, 293.66 * 2, 329.63 * 2, 349.23 * 2,
  392.00 * 2, 440.00 * 2, 493.88 * 2, 523.25 * 2
};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

// B♭管トランペットの移調係数（C→B♭約0.889倍）
const float BbTransposition = 233.08 / 261.63;

// 移調後の周波数配列（初期化はsetupで行う）
float doremiBbHz[numNotes];

// --- ソフトクリッピング ---
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

// --- 音色調整関数群 ---
// 必要に応じて呼び出し・調整してください（ここは省略も可能）

void emphasizeThickness(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.10 * sin(2.0 * PI * 3000 * i / SAMPLE_RATE);
  }
}

void boostHighs(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.05 * sin(2.0 * PI * 10000 * i / SAMPLE_RATE);
  }
}

void addBrightnessNoise(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.03 * sin(2.0 * PI * 7200 * i / SAMPLE_RATE);
  }
}

void maskMidSharpness(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] -= 0.02 * sin(2.0 * PI * 6000 * i / SAMPLE_RATE);
  }
}

// --- トランペット風波形生成 ---
void generateTrumpetWave(float freq) {
  float amplitudes[] = {
    1.20, 1.10, 1.05, 0.80, 0.85, 0.75,
    0.30, 0.25, 0.20, 0.18,
    0.15, 0.12, 0.10, 0.08,
    0.05, 0.03, 0.02, 0.01,
    0.005, 0.0
  };
  const int numHarmonics = sizeof(amplitudes) / sizeof(amplitudes[0]);

  float phaseOffsets[] = {
    0.0, 0.1, -0.05, 0.08, -0.1, 0.03,
    0.0, 0.05, -0.02, 0.07,
    0.04, -0.03, 0.02, 0.0,
    0.01, -0.01, 0.0, 0.0,
    0.0, 0.0
  };

  float modIndex = 3.0;
  float modFreqRatio = 2.5;
  static float tempSamples[NUM_SAMPLE];

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;
    float angle = 2.0 * PI * t;

    float modulator = sin(modFreqRatio * angle);
    float fmWave = sin(angle + modIndex * modulator);

    float attackMultiplier = (t < 0.01) ? (1.0 + 5.0 * (0.01 - t) / 0.01) : 1.0;

    float harmonics = 0.0;
    for (int n = 1; n <= numHarmonics; n++) {
      harmonics += amplitudes[n - 1] * sin(n * angle + phaseOffsets[n - 1]) * attackMultiplier;
    }

    float env = (t < 0.01) ? t / 0.01 : 1.0;
    float sample = 0.2 * fmWave + 0.8 * harmonics * env;

    sample = softClip(sample);
    tempSamples[i] = sample;
  }

  // 音色調整（必要に応じてコメントアウト可能）
  emphasizeThickness(tempSamples, NUM_SAMPLE);
  boostHighs(tempSamples, NUM_SAMPLE);
  addBrightnessNoise(tempSamples, NUM_SAMPLE);
  maskMidSharpness(tempSamples, NUM_SAMPLE);
  lowPassFilter(tempSamples, NUM_SAMPLE, 2);

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float s = tempSamples[i];
    int tmpVal = (int)((s * amp + 1.0) * 2047.5);
    tmpVal = constrain(tmpVal, 0, 4095);
    waveform[i] = (uint16_t)tmpVal;
  }

  // ビブラート（ゆらぎ）を無効化する場合は下の行を以下に変更してください
  // phaseInc = freq * NUM_SAMPLE / SAMPLE_RATE;
  float vibrato = 1.0 + 0.003 * sin(2 * PI * millis() / 500.0);
  phaseInc = freq * vibrato * NUM_SAMPLE / SAMPLE_RATE;
}

// --- DAC出力コールバック ---
void callback_playSound(timer_callback_args_t *arg) {
  int index = (int)phase;
  float frac = phase - index;

  uint16_t v1 = waveform[index % NUM_SAMPLE];
  uint16_t v2 = waveform[(index + 1) % NUM_SAMPLE];
  uint16_t value = (uint16_t)((1.0 - frac) * v1 + frac * v2);

  analogWrite(A0, value);  // Arduino UNO R4 WiFiのDAC出力

  phase += phaseInc;
  if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;

  sendData = 1;
}

// --- 初期化処理 ---
void setup() {
  analogWriteResolution(12);
  Serial.begin(115200);
  mySerial.begin(115200);

  // 移調後の周波数配列を作成
  for (int i = 0; i < numNotes; i++) {
    doremiBbHz[i] = doremiHz[i] * BbTransposition;
  }

  // タイマーセットアップ
  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;

  f = doremiBbHz[0]; // 初期音（B♭管トランペットのド）

  //generateTrumpetWave(f);

  timer.begin(TIMER_MODE_PERIODIC, type, ch, SAMPLE_RATE, 50.0f, callback_playSound, nullptr);
  timer.setup_overflow_irq();
  timer.open();
  timer.start();
}

// --- メインループ ---
void loop() {
  static unsigned long lastMillis = 0;
  static int idx = 0;
  unsigned long now = millis();

  if (now - lastMillis > 1000) {
    lastMillis = now;
    f = doremiBbHz[idx];
    idx = (idx + 1) % numNotes;
     generateTrumpetWave(f);
  }

  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);
  }
}
