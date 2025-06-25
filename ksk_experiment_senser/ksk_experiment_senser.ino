#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <SoftwareSerial.h>
#include <math.h>

MPU9250_asukiaaa mpu;
SoftwareSerial mySerial(3, 2);

const int numNotes = 8;
const float notes[numNotes] = {261.63, 293.66, 329.63, 349.23,
                               392.00, 440.00, 493.88, 523.25};

const int buttonPin = 7;      // ボタンピン（プルアップ接続想定）

bool buttonPressed = false;
unsigned long buttonPressStart = 0;
const unsigned long longPressDuration = 5000;  // 長押し5秒

float baseAngleX = 0;
bool baseSet = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mySerial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);

  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  Serial.println("MPU9250 initialized");
}

// X軸回りの角度を計算（度数）
float calculateAngleX() {
  mpu.accelUpdate();
  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float angleX_rad = atan2(ax, sqrt(ay*ay + az*az));
  return angleX_rad * 180.0 / PI;
}

// 基準角度をセット
void setBaseAngle() {
  baseAngleX = calculateAngleX();
  baseSet = true;
  Serial.print("Base angle set: ");
  Serial.println(baseAngleX);
}

// 基準角度をリセット
void resetBaseAngle() {
  baseSet = false;
  Serial.println("Base angle reset");
}

// ボタン状態の処理（短押しで基準セット、長押し5秒でリセット）
void handleButton() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {  // 押されている
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressStart = millis();
    } else {
      if (millis() - buttonPressStart >= longPressDuration) {
        // 長押し判定（5秒経過）
        resetBaseAngle();
        buttonPressed = false;  // 処理済みフラグ
        delay(500);             // 連続処理防止
        return;
      }
    }
  } else {  // 離された
    if (buttonPressed) {
      unsigned long pressDuration = millis() - buttonPressStart;
      buttonPressed = false;
      if (pressDuration < longPressDuration) {
        // 短押しで基準セット
        setBaseAngle();
      }
    }
  }
}

void loop() {
  handleButton();

  if (!baseSet) {
    Serial.println("Press button shortly to set base angle");
    delay(500);
    return;
  }

  float angleX_deg = calculateAngleX();
  float relativeAngle = constrain(angleX_deg - baseAngleX, -90, 90);

  float normalized = (relativeAngle + 90.0) / 180.0;
  int index = (int)(normalized * (numNotes - 1));

  int freqScaled = (int)(notes[index] * 100);  // 100倍して整数化（小数点第2位まで保持）

  Serial.print("Relative angle: ");
  Serial.print(relativeAngle, 2);
  Serial.print(" -> Freq*100: ");
  Serial.println(freqScaled);

  // 5桁にして送信（改行付き）
  char buf[8];
  sprintf(buf, "%05d\n", freqScaled);
  mySerial.print(buf);

  delay(500);
}
