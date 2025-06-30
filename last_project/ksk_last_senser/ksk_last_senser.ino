#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <SoftwareSerial.h>
#include <math.h>

MPU9250_asukiaaa mpu;
SoftwareSerial mySerial(3, 2);  // ソフトウェアシリアル：TX=3, RX=2

const int numNotes = 5;
const float notes[numNotes] = {261.63, 293.66, 329.63, 349.23, 392.00};

const int sendButtonPin = 5;    // 送信専用ボタン（プルアップ接続）

// X軸回りの角度を計算（度）
float calculateAngleX() {
  mpu.accelUpdate();
  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float angleX_rad = atan2(ax, sqrt(ay * ay + az * az));
  return angleX_rad * 180.0 / PI;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mySerial.begin(9600);

  pinMode(sendButtonPin, INPUT_PULLUP);  // 送信ボタン入力設定

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  Serial.println("MPU9250 initialized");
}

void loop() {
  float angleX_deg = calculateAngleX();

  // 0〜90度に制限（マイナス角度は0に丸める）
  angleX_deg = constrain(angleX_deg, 0.0, 90.0);

  // 0〜1.0 に正規化
  float normalized = angleX_deg / 90.0;

  // 該当する音のインデックスを計算
  int index = (int)(normalized * (numNotes - 1));
  int freqScaled = (int)(notes[index] * 100);  // 小数第2位まで送信

  Serial.print("AngleX (0-90): ");
  Serial.print(angleX_deg, 2);
  Serial.print(" -> Freq*100: ");
  Serial.println(freqScaled);

  // 送信ボタンが押されたら送信する
  if (digitalRead(sendButtonPin) == LOW) {
    char buf[8];
    sprintf(buf, "%05d\n", freqScaled);  // 例："03920\n"
    mySerial.print(buf);
  }

  delay(500);  // 更新間隔
}
