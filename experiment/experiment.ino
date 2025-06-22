#include "FspTimer.h"            // Arduino UNO R4 WiFi の FSPタイマーライブラリ
#include <SoftwareSerial.h>      // ソフトウェアシリアル通信ライブラリ

#define SAMPLE_RATE 16000        // サンプリングレート（Hz）
#define NUM_SAMPLE 800           // 波形1周期のサンプル数

SoftwareSerial mySerial(3, 2);   // ソフトウェアシリアル（ピン3=RX, ピン2=TX）

float f = 233.08;                // 初期周波数：シ♭3（B♭トランペットのC音）
float amp = 0.3;                 // 出力波形の振幅（0.0〜1.0）
uint16_t waveform[NUM_SAMPLE];  // 12bit DAC用の波形データ（0〜4095）
float phase = 0.0;              // 現在の位相（小数点を含むことで補間も可能）
float phaseInc = 0.0;           // 1サンプルごとの位相の進み幅

FspTimer timer;                 // タイマーインスタンス
byte b[2];                      // シリアル送信用（未使用の可能性あり）
int sendData = 0;               // 送信フラグ（未使用の可能性あり）

// 通常のC調でのドレミファソラシドの周波数（C4〜C5の1オクターブ）
const float doremiHz[] = {
  261.63 * 2, 293.66 * 2, 329.63 * 2, 349.23 * 2,
  392.00 * 2, 440.00 * 2, 493.88 * 2, 523.25 * 2
};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

// B♭トランペット用に移調するための係数（C音→B♭音）
const float BbTransposition = 233.08 / 261.63;
float doremiBbHz[numNotes];     // 移調後の周波数を格納する配列

// --- ソフトクリッピング（音割れ防止） ---
float softClip(float x) {
  if (x > 0.7) return 0.7 + (x - 0.7) * 0.3;
  else if (x < -0.7) return -0.7 + (x + 0.7) * 0.3;
  else return x;
}

// --- ローパスフィルタ（高域のノイズ除去） ---
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

// --- 音色調整関数（任意で適用） ---
void emphasizeThickness(float* samples, int len) {
  // 中低域の厚みを加える（3000Hzあたり）
  for (int i = 0; i < len; i++) {
    samples[i] += 0.10 * sin(2.0 * PI * 3000 * i / SAMPLE_RATE);
  }
}

void boostHighs(float* samples, int len) {
  // 高域の明るさを追加（10kHz）
  for (int i = 0; i < len; i++) {
    samples[i] += 0.05 * sin(2.0 * PI * 10000 * i / SAMPLE_RATE);
  }
}

void addBrightnessNoise(float* samples, int len) {
  // 明るさを出すためのノイズ的高周波（7200Hz）
  for (int i = 0; i < len; i++) {
    samples[i] += 0.03 * sin(2.0 * PI * 7200 * i / SAMPLE_RATE);
  }
}

void maskMidSharpness(float* samples, int len) {
  // 中域の尖りを軽減（6000Hz）
  for (int i = 0; i < len; i++) {
    samples[i] -= 0.02 * sin(2.0 * PI * 6000 * i / SAMPLE_RATE);
  }
}

// --- トランペット風波形生成関数 ---
void generateTrumpetWave(float freq) {
  float amplitudes[] = {
    1.4, 1.2, 1.0, 0.8, 0.6, 0.4,
    0.2, 0.1, 0.05, 0.03,
    0.02, 0.01, 0.005, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
  };

  const int numHarmonics = sizeof(amplitudes) / sizeof(amplitudes[0]);

  float phaseOffsets[] = {
    0.0, 0.1, -0.05, 0.08, -0.1, 0.03,
    0.0, 0.05, -0.02, 0.07,
    0.04, -0.03, 0.02, 0.0,
    0.01, -0.01, 0.0, 0.0,
    0.0, 0.0
  };

  float modIndex = 3.0;        // FM変調の深さ
  float modFreqRatio = 2.5;    // FM変調の周波数比（基本波の2.5倍）
  static float tempSamples[NUM_SAMPLE];

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;
    float angle = 2.0 * PI * t;

    // FMシンセ部分（音のゆらぎ）
    float modulator = sin(modFreqRatio * angle);
    float fmWave = sin(angle + modIndex * modulator);

    // アタック部分を強調するための乗数（出だしだけ強く）
    float attackMultiplier = (t < 0.01) ? (1.0 + 5.0 * (0.01 - t) / 0.01) : 1.0;

    // 倍音合成
    float harmonics = 0.0;
    for (int n = 1; n <= numHarmonics; n++) {
      harmonics += amplitudes[n - 1] * sin(n * angle + phaseOffsets[n - 1]) * attackMultiplier;
    }

    // エンベロープ（加速的に立ち上がる） フェイドアウト
    float env;
    if (t < 0.1) {
      // フェードイン（立ち上がり）をゆるやかに（指数カーブ）
      env = pow(t / 0.1, 3);  // 0.1秒かけてゆっくり上がる
    } else if (t > 0.8) {
      // フェードアウト（終わり）もゆるやかに
      float r = (1.0 - t) / 0.2;  // 0.2秒かけて減衰
      env = pow(r, 3);            // 滑らかに減衰
    } else {
      env = 1.0;  // 通常音量（中間部）
    }




    // FM波 + 倍音 + エンベロープ
    float sample = 0.2 * fmWave + 0.8 * harmonics * env;

    sample = softClip(sample);        // 音割れ防止処理
    tempSamples[i] = sample;          // 一時バッファに格納
  }

  // 音色調整エフェクトの適用（必要に応じて無効化可能）
  emphasizeThickness(tempSamples, NUM_SAMPLE);
  boostHighs(tempSamples, NUM_SAMPLE);
  addBrightnessNoise(tempSamples, NUM_SAMPLE);
  maskMidSharpness(tempSamples, NUM_SAMPLE);
  lowPassFilter(tempSamples, NUM_SAMPLE, 4);  // フィルタでノイズ低減

  // 最終的な波形データを0〜4095の範囲に変換し格納
  for (int i = 0; i < NUM_SAMPLE; i++) {
    float s = tempSamples[i];
    int tmpVal = (int)((s * amp + 1.0) * 2047.5);
    tmpVal = constrain(tmpVal, 0, 4095);
    waveform[i] = (uint16_t)tmpVal;
  }

  // 位相の進み量（周波数に応じて設定）
  //phaseInc = freq * NUM_SAMPLE / SAMPLE_RATE;

  // ビブラート（微小なゆらぎ）を加える場合はこちら
   float vibrato = 1.0 + 0.003 * sin(2 * PI * millis() / 500.0);
   phaseInc = freq * vibrato * NUM_SAMPLE / SAMPLE_RATE;
}

// --- DAC出力用の割り込みコールバック ---
void callback_playSound(timer_callback_args_t *arg) {
  int index = (int)phase;
  float frac = phase - index;

  // 線形補間で滑らかな出力を実現
  uint16_t v1 = waveform[index % NUM_SAMPLE];
  uint16_t v2 = waveform[(index + 1) % NUM_SAMPLE];
  uint16_t value = (uint16_t)((1.0 - frac) * v1 + frac * v2);

  analogWrite(A0, value);  // DAC出力（Arduino UNO R4 WiFi）

  // 位相を進める
  phase += phaseInc;
  if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;

  sendData = 1;  // ※未使用（シリアル送信のため？）
}

// --- 初期化処理 ---
void setup() {
  analogWriteResolution(12);  // DAC出力を12bit精度に設定
  Serial.begin(115200);
  mySerial.begin(115200);     // ソフトウェアシリアルの初期化

  // C調 → B♭調への移調
  for (int i = 0; i < numNotes; i++) {
    doremiBbHz[i] = doremiHz[i] * BbTransposition;
  }

  // FSPタイマーを使用可能なチャンネルで初期化
  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;

  f = doremiBbHz[0];  // 初期音を設定（B♭トランペットのC）

  // タイマー設定・開始（毎秒16000回、コールバック呼び出し）
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

  // 1秒ごとに次の音へ切り替え（ドレミファソラシドを順に再生）
  if (now - lastMillis > 1000) {
    lastMillis = now;
    f = doremiBbHz[idx];         // 次の周波数に設定
    idx = (idx + 1) % numNotes;  // 音を循環
    generateTrumpetWave(f);      // 波形を再生成
  }

  // ※この部分は未使用（送信用？）
  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);
  }
}
