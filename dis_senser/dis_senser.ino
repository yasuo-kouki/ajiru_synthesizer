#include <Arduino.h>
#include "FspTimer.h"
#include <SoftwareSerial.h>

#define echoPin       8
#define trigPin       9
#define buttonPin     4
#define DAC_PIN       A0

#define DISTANCE_MAX  20.0f
const float scale[] = {
  146.83f, 164.81f, 174.61f,
  220.00f, 246.94f
};
constexpr size_t SCALE_SZ = sizeof(scale) / sizeof(scale[0]);

float lastDistance = -1.0f;
float currentFreq = 0.0f;
bool isSending = false;

// ソフトウェアシリアル（TX: 3, RX: 2）
SoftwareSerial mySerial(3, 2);

FspTimer timer;

/*===== 超音波センサによる距離計測（1回） =====*/
float measureDistanceCM() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, 30000);
  return us ? (us * 0.0343f * 0.5f) : -1.0f;
}

/*===== 複数回測定して平均を取る =====*/
float getMeanDistance() {
  const uint8_t N = 5;
  float sum = 0; uint8_t ok = 0;
  for (uint8_t i = 0; i < N; ++i) {
    float d = measureDistanceCM();
    if (d > 0) { sum += d; ++ok; }
    delay(10);
  }
  return ok ? sum / ok : -1.0f;
}

/*===== 測定距離をスケール配列の周波数に変換 =====*/
float mapDistanceToFreq(float d) {
  int idx = constrain(int(d / (DISTANCE_MAX / SCALE_SZ)), 0, SCALE_SZ - 1);
  return scale[idx];
}

/*===== タイマーISR（未使用だが構文保持） =====*/
void playISR(timer_callback_args_t *) {
  // 今回は波形再生なし
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600); // ソフトウェアシリアル開始

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  analogWriteResolution(12);
  analogWrite(DAC_PIN, 2048);  // 中点

  uint8_t type;
  int8_t ch = FspTimer::get_available_timer(type);
  if (ch < 0) while (true);
  if (!timer.begin(TIMER_MODE_PERIODIC, type, ch, 8000, 0.0f, playISR, nullptr)) {
    while (true);
  }
  timer.setup_overflow_irq();
  timer.open();
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    float d = getMeanDistance();
    if (d > 0 && (lastDistance < 0 || fabsf(d - lastDistance) > 1.f)) {
      float freq = mapDistanceToFreq(d);
      lastDistance = d;

      int freqInt = (int)(freq);  // 整数化

      // デバッグ用にシリアルモニタ出力
      Serial.print("距離[cm]: ");
      Serial.print(d, 1);
      Serial.print(" -> 周波数[Hz]: ");
      Serial.println(freqInt);

      // 送信用バッファに変換
      char buf[6];  // 最大5桁+改行
      sprintf(buf, "%d\n", freqInt);
      mySerial.print(buf);
    }
    delay(100);
  } else {
    lastDistance = -1.0f;
  }
}
