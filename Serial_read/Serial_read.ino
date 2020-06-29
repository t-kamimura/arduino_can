int lowByte = 0; // 受信データ用
int highByte = 0;
int data = 0;

void setup()
{
  Serial.begin(9600); // 9600bpsでシリアルポートを開く
}

void loop()
{
  if (Serial.available() > 0)
  {                             
    // 受信したデータが存在する
    // // 何バイトのデータが来ているか？
    // Serial.print("packet: ");
    // Serial.print(Serial.available());
    // Serial.println();

    // 受信データを読み込む
    lowByte = Serial.read();
    highByte = Serial.read();
  }
  data = makeWord(highByte, lowByte);
  data = (highByte << 8) + lowByte;

  Serial.print("I received: "); // 受信データを送りかえす
  Serial.flush();
  /*
  Serial.print(lowByte, HEX);
  Serial.print("\t");
  Serial.print(highByte, HEX);
  Serial.print("\t");
  */
  Serial.println(data);
  Serial.flush();
}
