#include "FspTimer.h"
#include <SoftwareSerial.h>

#define SAMPLE_RATE 32000        // サンプリング周波数（Hz）
#define NUM_SAMPLE 400           // 波形1周期のサンプル数

SoftwareSerial mySerial(3, 2);   // ソフトウェアシリアル（ピン3=RX, ピン2=TX）

float f = 261.63;                // 初期周波数（ド：C4）
float amp = 0.3;                 // 振幅
uint16_t waveform[NUM_SAMPLE];  // 12bit波形バッファ
float phase = 0.0;               // 現在の位相
float phaseInc = 0.0;            // 1サンプルごとの位相の進み量

FspTimer timer;                  // FSPタイマーインスタンス
byte b[2];                       // シリアル送信用バッファ
int sendData = 0;                // シリアル送信フラグ

// ドレミファソラシドの周波数配列
const float doremiHz[] = {
  261.63, 293.66, 329.63, 349.23,
  392.00, 440.00, 493.88, 523.25
};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

// --- ソフトクリッピング：大きすぎる波形を丸めて音割れ防止 ---
float softClip(float x) {
  if (x > 0.7) return 0.7 + (x - 0.7) * 0.3;
  else if (x < -0.7) return -0.7 + (x + 0.7) * 0.3;
  else return x;
}

// --- ローパスフィルタ：高周波を抑えることで音色を滑らかに ---
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

// --- 以下、音色調整のための周波数帯域の強調／抑制 ---
// 音の「太さ」を出す（中高域強調）
void emphasizeThickness(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.10 * sin(2.0 * PI * 3000 * i / SAMPLE_RATE);
  }
}

// 高域（きらびやかさ）を強調
void boostHighs(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.05 * sin(2.0 * PI * 10000 * i / SAMPLE_RATE);
  }
}

// チリチリした「明るさノイズ」を追加
void addBrightnessNoise(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] += 0.03 * sin(2.0 * PI * 7200 * i / SAMPLE_RATE);
  }
}

// 中高域の鋭さ（耳に刺さる感じ）を和らげる
void maskMidSharpness(float* samples, int len) {
  for (int i = 0; i < len; i++) {
    samples[i] -= 0.02 * sin(2.0 * PI * 6000 * i / SAMPLE_RATE);
  }
}

// --- トランペット風の波形を生成 ---
void generateTrumpetWave(float freq) {
  // スペクトル分析に基づく20倍音の振幅
  float amplitudes[] = {
    0.90, 1.00, 0.85, 0.60, 0.70,
    0.50, 0.60, 0.55, 0.50, 0.45,
    0.40, 0.35, 0.30, 0.28, 0.25,
    0.22, 0.20, 0.18, 0.16, 0.15
  };
  const int numHarmonics = sizeof(amplitudes) / sizeof(amplitudes[0]);

  float modIndex = 3.0;           // FM変調の深さ
  float modFreqRatio = 2.5;       // モジュレータの周波数比
  static float tempSamples[NUM_SAMPLE]; // 一時波形バッファ

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;
    float angle = 2.0 * PI * t;

    // FM合成による基本波生成
    float modulator = sin(modFreqRatio * angle);
    float fmWave = sin(angle + modIndex * modulator);

    // アタック部分を強調（tが0に近いとき強く）
    float attackMultiplier = (t < 0.01) ? (1.0 + 5.0 * (0.01 - t) / 0.01) : 1.0;

    // 複数の倍音を合成
    float harmonics = 0.0;
    for (int n = 1; n <= numHarmonics; n++) {
      harmonics += amplitudes[n - 1] * sin(n * angle) * attackMultiplier;
    }

    // 簡易的なエンベロープ（アタック）
    float env = (t < 0.01) ? t / 0.01 : 1.0;

    // FM波と倍音波をブレンド
    float sample = 0.2 * fmWave + 0.8 * harmonics * env;

    // クリッピング
    sample = softClip(sample);

    tempSamples[i] = sample;
  }

  // 音色強調フィルタ群適用
  emphasizeThickness(tempSamples, NUM_SAMPLE);
  boostHighs(tempSamples, NUM_SAMPLE);
  addBrightnessNoise(tempSamples, NUM_SAMPLE);
  maskMidSharpness(tempSamples, NUM_SAMPLE);
  lowPassFilter(tempSamples, NUM_SAMPLE, 2); // 最後に滑らかに

  // 最終的に12bit DAC用に変換
  for (int i = 0; i < NUM_SAMPLE; i++) {
    float s = tempSamples[i];
    int tmpVal = (int)((s * amp + 1.0) * 2047.5); // -1〜1を0〜4095に
    tmpVal = constrain(tmpVal, 0, 4095);
    waveform[i] = (uint16_t)tmpVal;
  }

  // ビブラート（軽く周波数変化）を追加
  float vibrato = 1.0 + 0.003 * sin(2 * PI * millis() / 500.0);
  phaseInc = freq * vibrato * NUM_SAMPLE / SAMPLE_RATE;
}


// --- コールバックで周期的に波形を出力 ---
void callback_playSound(timer_callback_args_t *arg) {
  int index = (int)phase;
  float frac = phase - index;

  // 線形補間（2サンプル間をなめらかに補間）
  uint16_t v1 = waveform[index % NUM_SAMPLE];
  uint16_t v2 = waveform[(index + 1) % NUM_SAMPLE];
  uint16_t value = (uint16_t)((1.0 - frac) * v1 + frac * v2);

  // DACに出力
  b[0] = value >> 8;
  b[1] = value & 0xFF;
  analogWrite(A0, value); // R4 WiFiのDAC出力

  // 位相更新（ループ）
  phase += phaseInc;
  if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;

  sendData = 1;
}

// --- 初期化処理 ---
void setup() {
  analogWriteResolution(12); // 12bit DAC指定
  Serial.begin(115200);
  mySerial.begin(115200);

  // 利用可能なFspTimerチャネルを取得
  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;

  // タイマー初期化
  timer.begin(TIMER_MODE_PERIODIC, type, ch, SAMPLE_RATE, 50.0f, callback_playSound, nullptr);
  timer.setup_overflow_irq();
  timer.open();
  timer.start();
}

// --- メインループ：500msごとに次の音に切り替え ---
void loop() {
  static unsigned long lastMillis = 0;
  static int idx = 0;
  unsigned long now = millis();

  // 音階を一定間隔で変更
  if (now - lastMillis > 500) {
    lastMillis = now;
    f = doremiHz[idx];
    idx = (idx + 1) % numNotes;
    generateTrumpetWave(f); // 新しい音を生成
  }

  // シリアル通信でDAC波形を送る（必要に応じて）
  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);
  }
}
