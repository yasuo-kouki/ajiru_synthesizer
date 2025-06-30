#include "FspTimer.h"             // Arduino UNO R4 WiFi 用 FSP タイマー
#include <SoftwareSerial.h>       // ソフトウェアシリアル通信ライブラリ

#define SAMPLE_RATE 16000         // サンプリングレート（Hz）
#define NUM_SAMPLE 800            // 波形1周期あたりのサンプル数

SoftwareSerial mySerial(3, 2);    // TX=2, RX=3

float f = 261.63;                 // 現在の周波数（初期値：Bb3）
float amp = 0.3;                  // 音の振幅（0.0～1.0）
float phase = 0.0;                // 現在の位相
float phaseInc = 0.0;             // サンプルごとの位相増加量

uint16_t waveforms[5][NUM_SAMPLE];  // 5つの音の波形を格納
uint16_t* waveform = waveforms[0];  // 現在の出力波形（ポインタ）

FspTimer timer;                   // タイマーインスタンス
byte b[2];                        // シリアル送信用（未使用）
int sendData = 0;                 // シリアル送信フラグ（未使用）

const float doremiHz_1[] = {
  261.63 , 293.66, 329.63 ,
  349.23 , 392.00 
};

// ドレミファソ（C4～G4）の周波数（倍音合成の元となる）
const float doremiHz[] = {
  261.63 * 2, 293.66 * 2, 329.63 * 2,
  349.23 * 2, 392.00 * 2
};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

// B♭トランペット用の移調比（C→Bb）
const float BbTransposition = 233.08 / 261.63;

// --- ソフトクリッピング（音割れ防止） ---
float softClip(float x) {
  if (x > 0.7) return 0.7 + (x - 0.7) * 0.3;
  else if (x < -0.7) return -0.7 + (x + 0.7) * 0.3;
  else return x;
}

// --- トランペット風波形を生成して格納する関数 ---
void generateTrumpetWave(float freq, uint16_t* dest) {
  // 倍音ごとの振幅（鋭さ・太さのバランス）
  float amplitudes[] = {
    1.4, 1.2, 1.0, 0.8, 0.6, 0.4,
    0.2, 0.1, 0.05, 0.03,
    0.02, 0.01, 0.005, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
  };

  float phaseOffsets[] = {
    0.0, 0.1, -0.05, 0.08, -0.1, 0.03,
    0.0, 0.05, -0.02, 0.07,
    0.04, -0.03, 0.02, 0.0,
    0.01, -0.01, 0.0, 0.0,
    0.0, 0.0
  };

  const int numHarmonics = sizeof(amplitudes) / sizeof(amplitudes[0]);
  float modIndex = 3.0;           // FM変調の深さ
  float modFreqRatio = 2.5;       // FM変調の周波数比
  float tempSamples[NUM_SAMPLE];  // 一時波形バッファ

  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;
    float angle = 2.0 * PI * t;

    // FM変調波形でランダムな揺らぎを追加
    float modulator = sin(modFreqRatio * angle);
    float fmWave = sin(angle + modIndex * modulator);

    // アタック部分を強調（出だしがパッと鳴るように）
    float attackMultiplier = (t < 0.01) ? (1.0 + 5.0 * (0.01 - t) / 0.01) : 1.0;

    // 倍音合成（harmonics）
    float harmonics = 0.0;
    for (int n = 1; n <= numHarmonics; n++) {
      harmonics += amplitudes[n - 1] * sin(n * angle + phaseOffsets[n - 1]) * attackMultiplier;
    }

    // エンベロープ（フェードインとアウト）
    float env;
    if (t < 0.1) env = pow(t / 0.1, 3);            // フェードイン
    else if (t > 0.8) env = pow((1.0 - t) / 0.2, 3); // フェードアウト
    else env = 1.0;

    // 音の合成：FM成分 + 倍音
    float sample = 0.2 * fmWave + 0.8 * harmonics * env;
    sample = softClip(sample);       // 音割れ防止
    tempSamples[i] = sample;         // バッファへ格納
  }

  // 波形を 0〜4095（12bit DAC）に変換して格納
  for (int i = 0; i < NUM_SAMPLE; i++) {
    int tmpVal = (int)((tempSamples[i] * amp + 1.0) * 2047.5);
    tmpVal = constrain(tmpVal, 0, 4095);
    dest[i] = (uint16_t)tmpVal;
  }
}

// --- タイマー割り込み：DAC出力用 ---
void callback_playSound(timer_callback_args_t *arg) {
  int index = (int)phase;
  float frac = phase - index;

  // 線形補間で波形を滑らかに出力
  uint16_t v1 = waveform[index % NUM_SAMPLE];
  uint16_t v2 = waveform[(index + 1) % NUM_SAMPLE];
  uint16_t value = (uint16_t)((1.0 - frac) * v1 + frac * v2);

  analogWrite(A0, value);  // DAC出力（12bit）

  phase += phaseInc;
  if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;

  sendData = 1;  // ※未使用
}

// --- セットアップ処理 ---
void setup() {
  analogWriteResolution(12);   // DACの分解能を12bitに設定
  Serial.begin(9600);
  mySerial.begin(9600);

  // 各音程の波形を事前生成
  for (int i = 0; i < numNotes; i++) {
    float transFreq = doremiHz[i] * BbTransposition;
    generateTrumpetWave(transFreq, waveforms[i]);
  }


  // タイマー初期化（利用可能なチャンネルを使用）
  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;

  timer.begin(TIMER_MODE_PERIODIC, type, ch, SAMPLE_RATE, 50.0f, callback_playSound, nullptr);
  timer.setup_overflow_irq();
  timer.open();
  timer.start();
}

// --- メインループ（受信した音に応じて波形切り替え） ---
void loop() {
  if (mySerial.available()) {
    String input = mySerial.readStringUntil('\n');
    float newFreq = input.toFloat();  // 例: 39200 → 392.00Hz
    

    if (newFreq > 0) {
      newFreq = newFreq / 100.0;  // 送信側が100倍して送る前提
    Serial.println(newFreq);

      // 近い周波数を探して、該当する音を特定
      int noteIdx = -1;
      for (int i = 0; i < numNotes; i++) {
        if (abs(doremiHz_1[i] - newFreq) < 1.0) {
          noteIdx = i;
          break;
        }
      }

      // 該当する音があれば波形を切り替え
      if (noteIdx != -1) {
        float newF = doremiHz[noteIdx] * BbTransposition;
        timer.stop();
        noInterrupts();  // 割り込み禁止（安全に切り替え）
        waveform = waveforms[noteIdx];  // 対応波形を切り替え
        f = newF;
        float vibrato = 1.0 + 0.003 * sin(2 * PI * millis() / 500.0);
        phaseInc = f * vibrato * NUM_SAMPLE / SAMPLE_RATE;
        interrupts();    // 割り込み再開
        timer.start();
      }
    }
  }

  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);  // ※送信データなし（形式だけ残してある）
  }
}
