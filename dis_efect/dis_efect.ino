#include "FspTimer.h"

#define SAMPLE_RATE 8000
#define NUM_SAMPLE 400  // ちょうど1周期の整数倍になるよう調整（重要）

// 通信用
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2);


volatile uint32_t sampleIndex = 0;
float f = 261.63;
float amp = 0.6;
uint16_t waveform[NUM_SAMPLE];
float phase = 0.0;
float phaseInc = 0.0;

FspTimer timer;
byte b[2];
int sendData = 0;
volatile bool isPlaying = true;

const float doremiHz[] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25};
const int numNotes = sizeof(doremiHz) / sizeof(doremiHz[0]);

void generateSineWave(float freq) {
  // 周波数に合わせてちょうど1周期分の波形を作る
  for (int i = 0; i < NUM_SAMPLE; i++) {
    float t = (float)i / NUM_SAMPLE;
    float sample = sin(2.0 * PI * t);  // 純音（ノイズゼロ）
    int tmp = (int)((sample * amp + 1.0) * 2047.5);
    tmp = constrain(tmp, 0, 4095);
    waveform[i] = (uint16_t)tmp;

  }
  phaseInc = freq * NUM_SAMPLE / SAMPLE_RATE;  // 再生速度制御（整数倍再生用）
}


void callback_playSound(timer_callback_args_t *arg) {
  if (isPlaying) {
    int index = (int)phase % NUM_SAMPLE;
    uint16_t value = waveform[index];
    b[0] = value / 256;
    b[1] = value % 256;
    analogWrite(A0, value);
    phase += phaseInc;
    if (phase >= NUM_SAMPLE) phase -= NUM_SAMPLE;
  } else {
    b[0] = 2047 / 256;
    b[1] = 2047 % 256;
    analogWrite(A0, 2047);
  }
  sendData = 1;
}



void setup() {
  analogWriteResolution(12);
  Serial.begin(115200);
  mySerial.begin(9600);    // Arduino間通信用
  randomSeed(analogRead(A0));

  //generateSineWave(f);  // 初期音（ド）

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


if (mySerial.available()) {
  String input = mySerial.readStringUntil('\n');
  float newFreq = input.toFloat();
  if (newFreq > 0) {
    f = newFreq;
    generateSineWave(f);
    Serial.print("受信した周波数: ");
    Serial.println(f);  // ← 追加
  }
}


  if (sendData == 1) {
    sendData = 0;
    Serial.write(b, 2);
  }

  //delayMicroseconds(50);
}