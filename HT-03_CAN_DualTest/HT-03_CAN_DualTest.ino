/* Dual motor test*/
/* 複数モーターの制御に挑戦 */

/*-------------------------------------
ID  role
1   Root joint
2   Knee joint
---------------------------------------*/

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
#define CLICK A4

#define MOTOR_ADDRESS_1 0x01
#define MOTOR_ADDRESS_2 0x02

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 10

int pos = 0;
int vel = 0;
int kp = 50;
int kd = 0;
int ff = 0;

unsigned int upos = pos - 32768; // 16 bit
unsigned int uvel = vel - 2048;  // 12 bit
unsigned int ukp = kp;           // 12 bit (defined as positive value only)
unsigned int ukd = kd;           // 12 bit (defined as positive value only)
unsigned int uff = ff - 2048;    // 12 bit

struct motor
{
  int pos_cur = 0;
  int vel_cur = 0;
  int cur_cur = 0;
} motor1, motor2;


unsigned long timer[3];

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

unsigned char can_msg[8];
unsigned char len = 0;
unsigned char buf[6];

void motor_enable(int motor_address_)
{
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFC;
  // SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(motor_address_, 0, 8, can_msg);
  delay(1000);
}

void motor_disable(int motor_address_)
{
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFD;
  // SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(motor_address_, 0, 8, can_msg);
  delay(500);
}

void motor_zeroPosition(int motor_address_)
{
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFE;
  // SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(motor_address_, 0, 8, can_msg);
}

void motor_writeCmd(int motor_address_, int pos_, int vel_, int kp_, int kd_, int ff_)
{
  // 符号なし変数に変換
  upos = (unsigned int)(pos_ - 32768);             // 16 bit
  uvel = (unsigned int)(vel_ - 2048); // 12 bit
  ukp = (unsigned int)(kp_);          // 12 bit (defined as positive value only)
  ukd = (unsigned int)(kd_);          // 12 bit (defined as positive value only)
  uff = (unsigned int)(ff_ - 2048);   // 12 bit

  // 入力をプロンプトに合わせて変換
  can_msg[0] = upos >> 8;
  can_msg[1] = upos & 0x00FF;
  can_msg[2] = (uvel >> 4) & 0xFF;
  can_msg[3] = ((uvel & 0x000F) << 4) + ((ukp >> 8) & 0xFF);
  can_msg[4] = ukp & 0xFF;
  can_msg[5] = ukd >> 4;
  can_msg[6] = ((ukd & 0x000F) << 4) + (uff >> 8);
  can_msg[7] = uff & 0xff;

  // CAN通信で送る
  CAN.sendMsgBuf(motor_address_, 0, 8, can_msg);

  SERIAL.print(", ");
  SERIAL.print(pos);
  SERIAL.print(", ");
  SERIAL.print(vel);
  SERIAL.print(", ");
  SERIAL.print(ff);
  SERIAL.print(", ");
  SERIAL.print(kp);
  SERIAL.print(", ");
  SERIAL.print(kd);
}

void motor_readState()
{
  if (CAN_MSGAVAIL == CAN.checkReceive()) //check if data coming
  {
    CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf

    // unsigned long canId = CAN.getCanId();
    unsigned int id_ = buf[0];
    unsigned int upos_cur_ = (buf[1] << 8) + buf[2];
    unsigned int uvel_cur_ = (buf[3] << 4) + ((buf[4] & 0xF0) >> 4);
    unsigned int ucur_cur_ = ((buf[4] & 0x0F) << 8) + buf[5];

    int pos_cur_ = upos_cur_ + 32768;
    int vel_cur_ = uvel_cur_ - 2048;
    int cur_cur_ = ucur_cur_ - 2048;

    if (id_ == 1)
    {
      motor1.pos_cur = pos_cur_;
      motor1.vel_cur = vel_cur_;
      motor1.cur_cur = cur_cur_;
    }
    else if (id_==2)
    {
      motor2.pos_cur = pos_cur_;
      motor2.vel_cur = vel_cur_;
      motor2.cur_cur = cur_cur_;
    }

    SERIAL.print(", CUR ID=");
    SERIAL.print(id_);
    SERIAL.print(", ");
    SERIAL.print(pos_cur_);
    SERIAL.print(", ");
    SERIAL.print(vel_cur_);
    SERIAL.print(",  ");
    SERIAL.print(cur_cur_);
    // SERIAL.println();
  }
}

void serialWriteTerminator()
{
  SERIAL.write(13);
  SERIAL.write(10);
}

void setup()
{
  SERIAL.begin(BAUDRATE);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(CLICK, INPUT);
  digitalWrite(CLICK, HIGH);
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

  // SERIAL.print("position initializing...");
  motor_zeroPosition(MOTOR_ADDRESS_1); // 現在位置をゼロということにする
  motor_zeroPosition(MOTOR_ADDRESS_2); // 現在位置をゼロということにする
  digitalWrite(LED3, HIGH);
  delay(1000);

  motor_enable(MOTOR_ADDRESS_1); // モーターモードに入る
  motor_enable(MOTOR_ADDRESS_2); // モーターモードに入る
  delay(1000);

  SERIAL.print("TIMER, TGTPOS, TGTVEL, TGTFF, CURPOS, CURVEL, CURCUR");
  serialWriteTerminator();

  timer[0] = millis();
}

void loop()
{

  while (1)
  {
    if (digitalRead(CLICK)==LOW)
    {
      break;
    }
    timer[1] = millis();
    SERIAL.print(timer[1] - timer[0]);

    motor_writeCmd(MOTOR_ADDRESS_1, pos, vel, kp, kd, ff);
    motor_writeCmd(MOTOR_ADDRESS_2, pos, vel, kp, kd, ff);

    // SERIAL.println("receiving CAN msg...");
    motor_readState();
    motor_readState();
    serialWriteTerminator();

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

  motor_disable(MOTOR_ADDRESS_1);
  motor_disable(MOTOR_ADDRESS_2);
  delay(500);
  SERIAL.println("Program finish!");
  digitalWrite(LED3, LOW);
  while (true)
  {
    delay(100);
  }
}
