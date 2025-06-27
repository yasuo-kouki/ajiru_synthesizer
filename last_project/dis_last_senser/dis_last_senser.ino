#include <Arduino.h>
#include "FspTimer.h"
#include <SoftwareSerial.h>

#define echoPin       8
#define trigPin       9
#define buttonPin     4
#define DAC_PIN       A0

#define DISTANCE_MAX  72.67sf
const float scale[] = {
  146.83f, 164.81f, 174.61f,
  220.00f, 246.94f
};
constexpr size_t SCALE_SZ = sizeof(scale) / sizeof(scale[0]);

float lastDistance = -1.0f;
float currentFreq = 0.0f;
bool isSending = false;

SoftwareSerial mySerial(3, 2);  // TX:3, RX:2
FspTimer timer;

/*===== 超音波センサによる距離測定（1回） =====*/
float measureDistanceCM() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, 30000);
  return us ? (us * 0.0343f * 0.5f) : -1.0f;
}

/*===== 最頻値（モード）を使った距離測定 =====*/
float getModeDistance() {
  const uint8_t N = 7;
  float distances[N];
  uint8_t count[N] = {0};

  for (uint8_t i = 0; i < N; ++i) {
    distances[i] = measureDistanceCM();
    delay(10);
  }

  // 最頻値を計算（誤差±0.5cm で同一とみなす）
  for (uint8_t i = 0; i < N; ++i) {
    if (distances[i] < 0) continue;
    for (uint8_t j = 0; j < N; ++j) {
      if (fabs(distances[i] - distances[j]) < 1.0) count[i]++;
    }
  }

  // 最も出現数が多い値を採用
  uint8_t maxIdx = 0;
  for (uint8_t i = 1; i < N; ++i) {
    if (count[i] > count[maxIdx]) maxIdx = i;
  }

  return (distances[maxIdx] > 0) ? distances[maxIdx] : -1.0f;
}

/*===== 測定距離 → 周波数にマッピング =====*/
float mapDistanceToFreq(float d) {
  int idx = constrain(int(d / (DISTANCE_MAX / SCALE_SZ)), 0, SCALE_SZ - 1);
  return scale[idx];
}

void playISR(timer_callback_args_t *) {}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  analogWriteResolution(12);
  analogWrite(DAC_PIN, 2048);  // DAC中点

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
    float d = getModeDistance();
    if (d > 0 && (lastDistance < 0 || fabsf(d - lastDistance) > 1.f)) {
      float freq = mapDistanceToFreq(d);
      lastDistance = d;

      int freqInt = (int)(freq);

      Serial.print("距離[cm]: ");
      Serial.print(d, 1);
      Serial.print(" -> 周波数[Hz]: ");
      Serial.println(freqInt);

      char buf[6];
      sprintf(buf, "%d\n", freqInt);
      mySerial.print(buf);
    }
    delay(100);
  } else {
    lastDistance = -1.0f;
  }
}
