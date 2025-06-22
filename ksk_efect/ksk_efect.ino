#include "FspTimer.h"

#define SAMPLE_RATE 8000         // 1秒あたりのサンプル数（Hz）
#define NUM_SAMPLE 400           // 波形1周期あたりのサンプル数（SAMPLE_RATEで割り切れる値が望ましい）

#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2);   // TX:3, RX:2 (マスターとスレーブ間通信用ソフトシリアル)

// グローバル変数定義
volatile uint32_t sampleIndex = 0;
float f = 261.63;                // 初期周波数（ド）
float amp = 0.4;                 // 振幅（0.0〜1.0）
uint16_t waveform[NUM_SAMPLE];  // 波形データ格納配列
float phase = 0.0;               // 波形の位相
float phaseInc = 0.0;            // 位相の進み量（周波数に依存）

FspTimer timer;                  // 再生用タイマーオブジェクト
byte b[2];                       // シリアル送信用データ（12bitデータを2バイトに）
int sendData = 0;                // 送信フラグ
volatile bool isPlaying = true; // 再生ON/OFF制御フラグ

// ドレミファソラシドの周波数配列
const float doremiHz[] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]); // 音階の数

// 指定された周波数に対応するサイン波データを生成
void generateSineWave(float freq) {
  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;                         // [0,1) の正規化時間
    float sample = sin(2.0 * PI * t);                        // 正弦波
    int tmp = (int)((sample * amp + 1.0) * 2047.5);          // 12bit DAC用スケーリング（0〜4095）
    tmp = constrain(tmp, 0, 4095);                           // 範囲外排除
    waveform[i] = (uint16_t)tmp;
  }
  // 位相進行量（波形の進むスピード＝音程）
  phaseInc = freq * NUM_SAMPLE / SAMPLE_RATE;
}

// タイマー割り込み時に呼ばれるコールバック関数（1サンプルずつ出力）
void callback_playSound(timer_callback_args_t *arg) {
  if (isPlaying) {
    int index = (int)phase % NUM_SAMPLE;       // 現在の波形インデックス
    uint16_t value = waveform[index];          // 出力値取得
    b[0] = value / 256;                        // 上位バイト
    b[1] = value % 256;                        // 下位バイト
    analogWrite(A0, value);                   // DAC出力（A0ピン）
    phase += phaseInc;                         // 位相を進める
    if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE; // 繰り返し処理
  } else {
    // 再生停止中は中間値を出力（無音）
    b[0] = 2047 / 256;
    b[1] = 2047 % 256;
    analogWrite(A0, 2047);
  }
  sendData = 1;  // シリアル送信フラグを立てる
}

void setup() {
  analogWriteResolution(12);       // 12bit精度のDAC出力を設定
  Serial.begin(115200);            // PCデバッグ用シリアル
  mySerial.begin(115200);          // Arduino間通信用ソフトシリアル

  // タイマーの空きチャンネルを探して、初期化と起動
  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;

  timer.begin(TIMER_MODE_PERIODIC, type, ch, SAMPLE_RATE, 50.0f, callback_playSound, nullptr);
  timer.setup_overflow_irq();
  timer.open();
  timer.start();
}

// メインループ（周波数の送受信と再生を行う）



void loop() {
  static unsigned long lastMillis = 0;
  static int idx = 0;
  unsigned long now = millis();


  if (mySerial.available()) {
    String input = mySerial.readStringUntil('\n');
    float newFreq = input.toFloat();
    if (newFreq > 0) {
      f = newFreq;
      generateSineWave(f);
    }
  }

  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);
  }

  //delayMicroseconds(50);
}
