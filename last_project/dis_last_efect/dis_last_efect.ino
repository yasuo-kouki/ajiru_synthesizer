#include "FspTimer.h"
#include <SoftwareSerial.h>

#define SAMPLE_RATE 8000     // サンプリングレート（Hz）
#define NUM_SAMPLE 400       // 1周期あたりのサンプル数（波形の解像度）

SoftwareSerial mySerial(3, 2);  // ソフトウェアシリアル通信（RX=3, TX=2）

volatile uint32_t sampleIndex = 0;
float f = 261.63;              // 現在の周波数（Hz）
float amp = 0.6;               // 音量（0.0～1.0）

// 対応する5音の周波数（小数点付き）と、その四捨五入整数値
const float doremiHz[] = {261.63, 293.66, 329.63, 349.23, 392.00};
const int intDoremiHz[] = {262, 294, 330, 349, 392};  // 受信値との比較用整数配列
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

// 各音に対応する波形データの2次元配列（numNotes x NUM_SAMPLE）
uint16_t waveTable[numNotes][NUM_SAMPLE];
uint16_t* currentWave = waveTable[0];  // 現在再生中の波形を指すポインタ

float phase = 0.0;             // 現在の位相（波形内の位置）
float phaseInc = 0.0;          // 1サンプルあたりの位相進行量（周波数依存）

FspTimer timer;                // ハードウェアタイマー制御用
byte b[2];                    // DAC出力値を2バイトで保持（送信用）
int sendData = 0;             // 送信フラグ
volatile bool isPlaying = true; // 音再生中フラグ

// --- ピアノ風の波形を1周期分生成し、bufferに格納 ---
// freqは周波数（Hz）、bufferは波形格納配列の先頭ポインタ
void generatePianoLikeWave(uint16_t* buffer, float freq) {
  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;  // 0.0〜1.0の波形位置

    // 倍音構成（基音＋2倍音〜4倍音）
    float sample = 0.7 * sin(2.0 * PI * t) +       // 基音
                   0.5 * sin(2.0 * PI * 2 * t) +   // 2倍音（明るさ）
                   0.3 * sin(2.0 * PI * 3 * t) +   // 3倍音（厚み）
                   0.2 * sin(2.0 * PI * 4 * t);    // 4倍音（硬さ）

    sample /= 1.7;               // 振幅正規化
    sample *= amp;               // 音量適用
    sample = constrain(sample, -1.0, 1.0);  // 範囲制限

    // 0〜4095の12bit DAC値に変換して格納
    buffer[i] = (uint16_t)((sample + 1.0) * 2047.5);
  }
}

// --- タイマー割り込みコールバック ---
// 波形をDACに出力し、位相を更新する
void callback_playSound(timer_callback_args_t *arg) {
  if (isPlaying) {
    int index = (int)phase % NUM_SAMPLE;   // 現在の位相に対応する波形インデックス
    uint16_t value = currentWave[index];  // 波形値取得

    // 送信用バイト分割
    b[0] = value / 256;
    b[1] = value % 256;

    analogWrite(A0, value);    // DAC出力（Arduino UNO R4 WiFi）

    phase += phaseInc;         // 位相進行
    if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;  // 位相の範囲内循環
  } else {
    // 再生停止時は中点のDAC値を出力（無音）
    b[0] = 2047 / 256;
    b[1] = 2047 % 256;
    analogWrite(A0, 2047);
  }
  sendData = 1;  // 送信フラグセット（必要ならシリアル送信等に使用）
}

// --- 初期化処理 ---
void setup() {
  analogWriteResolution(12);    // DACを12bit精度に設定
  Serial.begin(115200);         // デバッグ用シリアル開始
  mySerial.begin(9600);         // ソフトウェアシリアル開始

  randomSeed(analogRead(A0));   // ランダムシード（不要なら削除可）

  // 事前に全音階の波形を生成しておく（処理負荷軽減）
  for (int i = 0; i < numNotes; i++) {
    generatePianoLikeWave(waveTable[i], doremiHz[i]);
  }

  // 初期波形はド（C4）
  currentWave = waveTable[0];
  f = doremiHz[0];
  phaseInc = f * NUM_SAMPLE / SAMPLE_RATE;

  // タイマー初期化
  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) return;  // タイマー使用不可の場合は処理中断

  timer.begin(TIMER_MODE_PERIODIC, type, ch, SAMPLE_RATE, 50.0f, callback_playSound, nullptr);
  timer.setup_overflow_irq();
  timer.open();
  timer.start();
}

// --- メインループ ---
// ソフトウェアシリアルから周波数の整数値を受信し、対応波形に切り替える
void loop() {
  if (mySerial.available()) {
    String input = mySerial.readStringUntil('\n');
    int recvFreq = input.toInt();  // 受信文字列を整数化

    if (recvFreq > 0) {
      // 受信値が5音階配列のどれかに完全一致するかチェック
      for (int i = 0; i < numNotes; i++) {
        if (recvFreq == intDoremiHz[i]) {
          noInterrupts();        // 波形切り替え中は割り込み停止
          currentWave = waveTable[i];
          f = doremiHz[i];
          phase = 0.0;           // 位相リセット（波形の頭から再生）
          phaseInc = f * NUM_SAMPLE / SAMPLE_RATE;
          interrupts();          // 割り込み再開

          Serial.print("切り替えた周波数: ");
          Serial.println(f);
          break;
        }
      }
    }
  }

  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);  // DAC出力値送信（必要に応じて）
  }
}
