import processing.serial.*;

Serial port;
int [] x = new int [2]; // データ受信用バッファ
float [] waveform;

void setup() {
  //println(Serial.list()); // 使用可能なポートをコンソールに出力
  //String portName = Serial.list()[0]; // 最初のポートを使用（必要に応じて変更）
  
  size(512, 200);
  port = new Serial(this,"/dev/cu.usbmodemF412FA9C9E482", 115200); // ポートと通信速度を指定して接続
  port.clear(); // バッファをクリア
  
  waveform = new float [width];
  // 値を0.0で初期化
  for (int i=0 ; i < waveform.length; i++){
    waveform[i] = 0.0;
  }
}

void draw(){
  background(0);
  stroke(255);
  //線を引く
  for (int i = 0; i< waveform.length-1; i++){
    line(i, 100 - waveform[i]*50,i+1,100 - waveform[i+1]*50);
  }
}

// データが送信されてきたら呼び出される関数を実装する
void serialEvent(Serial p) {
  // データが送信されたきたら呼び出される関数を実装する
  if (p.available() > 1 ){
    x[0] = p.read(); //ポートからデータを収得(上位8bit)
    x[1] = p.read(); //下位8bit
    float value = map(x[0] * 256 + x[1] , 0 ,4095, -1.0 , 1.0);
    for (int i = 0; i < waveform.length - 1; i++){
      waveform[i] = waveform[i+1];
    }
    waveform[waveform.length - 1] = value;
    println("x[0]:␣" + x[0] + ",␣x[1]:␣" + x[1] + ",␣value:␣" + value);
  }
}
