#include <Wire.h>                  // I2C通信ライブラリ
#include <MPU9250_asukiaaa.h>     // MPU9250用ライブラリ（加速度・ジャイロ・磁気センサ）
#include <SoftwareSerial.h>       // ソフトウェアシリアル通信ライブラリ

MPU9250_asukiaaa mpu;             // MPU9250オブジェクトの作成
SoftwareSerial mySerial(3, 2);    // ソフトウェアシリアル（TX:3, RX:2）

// 8つの音階の周波数（C4～C5）
const int numNotes = 8;
const float notes[numNotes] = {261.63, 293.66, 329.63, 349.23,
                               392.00, 440.00, 493.88, 523.25};

void setup() {
  Serial.begin(115200);           // 通常のシリアル通信（モニタ表示用）
  Wire.begin();                   // I2Cバスの初期化
  mySerial.begin(115200);         // ソフトウェアシリアル通信の開始

  // MPU9250の初期設定
  mpu.setWire(&Wire);             // I2Cバスの設定
  mpu.beginAccel();               // 加速度センサの有効化
  mpu.beginGyro();                // ジャイロセンサの有効化
  mpu.beginMag();                 // 磁気センサの有効化

  Serial.println("MPU9250 initialized");  // 初期化完了の表示
}

void loop() {
  mpu.accelUpdate();              // 加速度データを更新

  float accelX = mpu.accelX();    // X軸の加速度を取得（-1.0〜1.0程度）

  // X軸加速度を0〜1の範囲に正規化（値が小さすぎ・大きすぎた場合の補正込み）
  float normalized = (accelX + 1.0) / 2.0;
  if (normalized < 0) normalized = 0;
  if (normalized > 1) normalized = 1;

  // 正規化値をインデックスに変換（0〜7）
  int index = (int)(normalized * (numNotes - 1));
  float freq = notes[index];     // 対応する周波数を取得

  // 周波数を整数に変換（小数点が送れないため）
  int freqInt = (int)(freq);
  int freqScaled = (int)(notes[index] * 100);  // 小数点第2位まで保持

  // シリアルモニタへ加速度と周波数を表示
  Serial.print("Accel X: ");
  Serial.print(accelX, 3);       // 小数点3桁まで表示
  Serial.print(" -> Freq: ");
  Serial.println(freqInt);       // 周波数の整数値を表示

  // 周波数を文字列に変換してソフトウェアシリアルで送信
  // 改行を入れることで受信側でデータの区切りがわかりやすくなる
    char buf[8];
    sprintf(buf, "%05d\n", freqScaled);
    mySerial.print(buf);

  delay(500); // 0.5秒待機（更新周期）
}
