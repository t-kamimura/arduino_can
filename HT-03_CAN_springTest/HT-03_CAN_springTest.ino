/* spring test */
/* 2リンク脚モジュールを直動バネのように動かしたい */

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

#define LOOPTIME 35

#define K 0.1  // stiffness of virtual spring
#define TGT_POS 0 // target position

/* Control table
---------------------------------------------------------
MIT-Cheetah motor is controlled with 5 variables:
position... target position
velocity... target velocity
kp... PD control P gain
kd... PD control D gain
ff... Feedforward torque bias input
---------------------------------------------------------
output_torque = kp*(position_error) + kd*velocity_error
                 + Feedforward_torque
---------------------------------------------------------
*/

int pos = 0;    // target position
int vel = 0;    // target velocity
int kp = 0;   // PD control P gain
int kd = 0;   // PD control D gain
int ff = 0;     // target feedforward torque

unsigned int upos = pos - 32768; // 16 bit
unsigned int uvel = vel - 2048;  // 12 bit
unsigned int ukp = kp;           // 12 bit (defined as positive value only)
unsigned int ukd = kd;           // 12 bit (defined as positive value only)
unsigned int uff = ff - 2048;    // 12 bit

unsigned long timer[3];
int pos_cur = TGT_POS;

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

unsigned char can_msg[8];
unsigned char len = 0;
unsigned char buf[6];

void motor_enable()
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
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  delay(1000);
}

void motor_disable()
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
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  delay(500);
}

void motor_zeroPosition()
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
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
}

void motor_writeCmd(int pos, int vel, int kp, int kd, int ff)
{
  // 符号なし変数に変換
  upos = (unsigned int)(pos - 32768); // 16 bit
  uvel = (unsigned int)(vel - 2048);  // 12 bit
  ukp = (unsigned int)(kp);           // 12 bit (defined as positive value only)
  ukd = (unsigned int)(kd);           // 12 bit (defined as positive value only)
  uff = (unsigned int)(ff - 2048);    // 12 bit

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
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);

  SERIAL.print(", ");
  SERIAL.print(pos);
  SERIAL.print(", ");
  SERIAL.print(vel);
  SERIAL.print(", ");
  SERIAL.print(ff);
}

void motor_readState()
{
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
    // SERIAL.println();
  }
}

int motor_readPos()
{
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

    return pos_cur;
  }
}

void serialWriteTerminator()
{
  SERIAL.write(13);
  SERIAL.write(10);
}

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

  // SERIAL.print("position initializing...");
  motor_zeroPosition(); // 現在位置をゼロということにする
  digitalWrite(LED3, HIGH);
  delay(1000);

  motor_enable(); // モーターモードに入る
  delay(1000);

  SERIAL.print("TIMER, TGTPOS, TGTVEL, TGTFF, CURPOS, CURVEL, CURCUR");
  serialWriteTerminator();

  timer[0] = millis();
}

void loop()
{

  for (int i = 0; i < 150; i++)
  {
    timer[1] = millis();
    SERIAL.print(timer[1] - timer[0]);

    int pos_cur = motor_readPos();
    serialWriteTerminator();
    double phi0 = 0.5*pi;
    double theta = phi0 + pos_cur*pi/1024;

    kp = K*(sin(theta) - sin(phi0))/theta;
    motor_writeCmd(pos, vel, kp, kd, ff);

    
    pos_cur = motor_readPos();
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

  motor_disable();
  delay(500);
  // SERIAL.println("Program finish!");
  digitalWrite(LED3, LOW);
  while (true)
  {
    delay(100);
  }
}
