#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <SoftwareSerial.h>

MPU9250_asukiaaa mpu;
SoftwareSerial mySerial(3, 2); // TX:3, RX:2

const int numNotes = 8;
const float notes[numNotes] = {261.63, 293.66, 329.63, 349.23, 392.00, 440.00, 493.88, 523.25};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mySerial.begin(115200);

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  Serial.println("MPU9250 initialized");
}

void loop() {
  mpu.accelUpdate();

  float accelX = mpu.accelX();
  float normalized = (accelX + 1.0) / 2.0;  // -1〜1 → 0〜1
  if (normalized < 0) normalized = 0;
  if (normalized > 1) normalized = 1;

  int index = (int)(normalized * (numNotes - 1));
  float freq = notes[index];

  // 周波数を整数化して送信（小数点は送れないのでint化）
  int freqInt = (int)(freq);

  // シリアルモニタ表示
  Serial.print("Accel X: ");
  Serial.print(accelX, 3);
  Serial.print(" -> Freq: ");
  Serial.println(freqInt);

  // SoftwareSerialで周波数を送信（数字をASCIIで送る）
  // 例えば、3桁の数字を文字列にして送る方法
  char buf[5];
  sprintf(buf, "%d\n", freqInt);  // '\n'付きで送ると区切りやすい
  mySerial.print(buf);

  delay(500);
}