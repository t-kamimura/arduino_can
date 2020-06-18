// demo: CAN-BUS Shield, send data
// loovee@seeed.cc

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

unsigned int pos = 32767; // 16 bit
unsigned int vel = 2048; // 12 bit
unsigned int kp  = 100;  // 12 bit
unsigned int kd  = 50;  // 12 bit
unsigned int ff  = 2047;  // 12 bit



//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);    //set CS PIN

unsigned char can_msg[8];

void setup()
{
  SERIAL.begin(9600);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  delay(1000);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))   //init can bus : baudrate = 500k
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }

  SERIAL.println("CAN BUS Shield init ok!");
  digitalWrite(LED2, HIGH);
  delay(1000);

  // 現在位置をゼロにする
  SERIAL.println("position initializing...");
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFE;
  SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  digitalWrite(LED3, HIGH);
  delay(1000);
  
  // モーターモードに入る
  SERIAL.println("Motor torque ON");
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFC;
  SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  delay(1000);


}


void loop()
{
  unsigned char len = 0;
  unsigned char buf[6];

  for (int i = 0; i < 50; i++)
  {
    // 入力をプロンプトに合わせて変換
    can_msg[0] = pos >> 8;
    can_msg[1] = pos & 0x00FF;
    can_msg[2] = (vel >> 4) & 0xFF;
    can_msg[3] = ((vel & 0x000F) << 4) + ((kp >> 8) & 0xFF);
    can_msg[4] = kp & 0xFF;
    can_msg[5] = kd >> 4;
    can_msg[6] = ((kd & 0x000F) << 4) + (ff >> 8);
    can_msg[7] = ff & 0xff;
    // SERIAL.print("TGT: ID: ");
    // SERIAL.print(MOTOR_ADDRESS);
    // SERIAL.print(" POS: ");
    // SERIAL.print(pos);
    // SERIAL.print(" VEL:");
    // SERIAL.print(vel);
    // SERIAL.print(" CUR:");
    // SERIAL.print(ff);
    // SERIAL.println();
    CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);

    // SERIAL.println("receiving CAN msg...");

    if (CAN_MSGAVAIL == CAN.checkReceive()) //check if data coming
    {
      CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf

      unsigned long canId = CAN.getCanId();

      // SERIAL.println("-------------------------");
      // SERIAL.print("Get dataID from : 0x");
      // SERIAL.println(canId, HEX);
      // for (int i = 0; i < len; i++) //print the data
      // {
      //   SERIAL.print(buf[i], HEX);
      //   SERIAL.print("\t");
      // }
      // SERIAL.println();

      unsigned int id = buf[0];
      unsigned int pos_cur = (buf[1] << 8) + buf[2];
      unsigned int vel_cur = (buf[3] << 4) + ((buf[4] & 0xF0) >> 4);
      unsigned int cur_cur = ((buf[4] & 0x0F) << 8) + buf[5];

      SERIAL.print("CUR: ID: ");
      SERIAL.print(id);
      SERIAL.print(" POS: ");
      SERIAL.print(pos_cur);
      SERIAL.print(" VEL:");
      SERIAL.print(vel_cur);
      SERIAL.print(" CUR:");
      SERIAL.print(cur_cur);
      SERIAL.println();
      pos += 10;
      delay(100);
    }
  }
  for (int i = 0; i < 50; i++)
  {
    // 入力をプロンプトに合わせて変換
    can_msg[0] = pos >> 8;
    can_msg[1] = pos & 0x00FF;
    can_msg[2] = (vel >> 4) & 0xFF;
    can_msg[3] = ((vel & 0x000F) << 4) + ((kp >> 8) & 0xFF);
    can_msg[4] = kp & 0xFF;
    can_msg[5] = kd >> 4;
    can_msg[6] = ((kd & 0x000F) << 4) + (ff >> 8);
    can_msg[7] = ff & 0xff;
    // SERIAL.print("TGT: ID: ");
    // SERIAL.print(MOTOR_ADDRESS);
    // SERIAL.print(" POS: ");
    // SERIAL.print(pos);
    // SERIAL.print(" VEL:");
    // SERIAL.print(vel);
    // SERIAL.print(" CUR:");
    // SERIAL.print(ff);
    // SERIAL.println();
    CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);

    // SERIAL.println("receiving CAN msg...");

    if (CAN_MSGAVAIL == CAN.checkReceive()) //check if data coming
    {
      CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf

      unsigned long canId = CAN.getCanId();

      // SERIAL.println("-------------------------");
      // SERIAL.print("Get dataID from : 0x");
      // SERIAL.println(canId, HEX);
      // for (int i = 0; i < len; i++) //print the data
      // {
      //   SERIAL.print(buf[i], HEX);
      //   SERIAL.print("\t");
      // }
      // SERIAL.println();

      unsigned int id = buf[0];
      unsigned int pos_cur = (buf[1] << 8) + buf[2];
      unsigned int vel_cur = (buf[3] << 4) + ((buf[4] & 0xF0) >> 4);
      unsigned int cur_cur = ((buf[4] & 0x0F) << 8) + buf[5];

      SERIAL.print("CUR: ID: ");
      SERIAL.print(id);
      SERIAL.print(" POS: ");
      SERIAL.print(pos_cur);
      SERIAL.print(" VEL:");
      SERIAL.print(vel_cur);
      SERIAL.print(" CUR:");
      SERIAL.print(cur_cur);
      SERIAL.println();
      pos -= 10;
      delay(100);
    }
  }

  SERIAL.println("Motor torque OFF");
  can_msg[0] = 0xFF;
  can_msg[1] = 0xFF;
  can_msg[2] = 0xFF;
  can_msg[3] = 0xFF;
  can_msg[4] = 0xFF;
  can_msg[5] = 0xFF;
  can_msg[6] = 0xFF;
  can_msg[7] = 0xFD;

  SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg);
  SERIAL.println("Program finish!");
  while(true)
  {
    delay(100);
  }
}
