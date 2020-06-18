/* freqency responce*/
/* 周波数応答を調べるプログラム */

#include <mcp_can.h>
#include <SPI.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

//Define LED pins
#define LED2 8
#define LED3 7

#define MOTOR_ADDRESS 0x01

#define LOOPTIME 40

int A = 0.25*1024;
double freq = 10; // [Hz]
double omega = 3.14*2/freq;

int pos = 0;
int vel = 0;
int kp = 100;
int kd = 25;
int ff = 0;

unsigned int upos = pos - 32768; // 16 bit
unsigned int uvel = vel - 2048;  // 12 bit
unsigned int ukp = kp;           // 12 bit (defined as positive value only)
unsigned int ukd = kd;           // 12 bit (defined as positive value only)
unsigned int uff = ff - 2048;    // 12 bit

unsigned long timer[3];

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

unsigned char can_msg[8];

void setup()
{
  SERIAL.begin(9600);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  delay(1000);

  while (CAN_OK != CAN.begin(CAN_1000KBPS)) //init can bus : baudrate = 500k
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }

  // SERIAL.println("CAN BUS Shield init ok!");
  digitalWrite(LED2, HIGH);
  delay(1000);

  // 現在位置をゼロにする
  // SERIAL.print("position initializing...");
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFE;
  // SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  digitalWrite(LED3, HIGH);
  delay(1000);

  // モーターモードに入る
  // SERIAL.print("Motor ctrl ON...");
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFC;
  // SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  delay(1000);

  SERIAL.println("TIMER, TGTPOS, TGTVEL, TGTFF, CURPOS, CURVEL, CURCUR");

  timer[0] = millis();
}

void loop()
{
  unsigned char len = 0;
  unsigned char buf[6];

  for (int i = 0; i < 150; i++)
  {
    timer[1] = millis();

    pos = A * sin(omega * (timer[1] - timer[0]));
    upos = pos - 32768;

    // 入力をプロンプトに合わせて変換
    can_msg[0] = upos >> 8;
    can_msg[1] = upos & 0x00FF;
    can_msg[2] = (uvel >> 4) & 0xFF;
    can_msg[3] = ((uvel & 0x000F) << 4) + ((ukp >> 8) & 0xFF);
    can_msg[4] = ukp & 0xFF;
    can_msg[5] = ukd >> 4;
    can_msg[6] = ((ukd & 0x000F) << 4) + (uff >> 8);
    can_msg[7] = uff & 0xff;

    SERIAL.print(timer[1]-timer[0]);
    SERIAL.print(", ");
    SERIAL.print(pos);
    SERIAL.print(", ");
    SERIAL.print(vel);
    SERIAL.print(", ");
    SERIAL.print(ff);
    // SERIAL.println();
    CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);

    // SERIAL.println("receiving CAN msg...");

    if (CAN_MSGAVAIL == CAN.checkReceive()) //check if data coming
    {
      CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf

      unsigned long canId = CAN.getCanId();
      unsigned int id = buf[0];
      unsigned int upos_cur = (buf[1] << 8) + buf[2];
      unsigned int uvel_cur = (buf[3] << 4) + ((buf[4] & 0xF0) >> 4);
      unsigned int ucur_cur = ((buf[4] & 0x0F) << 8) + buf[5];

      int pos_cur = upos_cur + 32768;
      int vel_cur = uvel_cur - 2048;
      int cur_cur = ucur_cur - 2048;

      // SERIAL.print("CUR: ID: ");
      // SERIAL.print(id);
      SERIAL.print(", ");
      SERIAL.print(pos_cur);
      SERIAL.print(", ");
      SERIAL.print(vel_cur);
      SERIAL.print(",  ");
      SERIAL.print(cur_cur);
      SERIAL.println();

      timer[2] = millis() - timer[1];
      if (timer[2] < LOOPTIME)
      {
        delay(LOOPTIME - timer[2]);
      }
      else
      {
        SERIAL.print("time shortage: ");
        SERIAL.println(timer[2] - LOOPTIME);
      }
    }
  }

  // SERIAL.print("Motor ctrl OFF...");
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFD;

  // SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  delay(500);
  // SERIAL.println("Program finish!");
  digitalWrite(LED3, LOW);
  while (true)
  {
    delay(100);
  }
}
