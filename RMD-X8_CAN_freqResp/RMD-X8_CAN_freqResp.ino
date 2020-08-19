/* 周波数応答確認用*/

#include <mcp_can.h>
#include <SPI.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

#define MOTOR_ADDRESS 0x141 //0x140 + ID(1~32)

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 5 //[ms] 

unsigned long timer[3];

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

unsigned char len = 0;
unsigned char buf[8], 
cmd_buf[8], reply_buf[8];

int A = 65536*0.3;
double f = 12.8; //[Hz]
double omega = 2*3.14*f;

int offset = 65536*0.5;  //A < offset

void setup()
{
  SERIAL.begin(BAUDRATE);
  delay(1000);

  while (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println("Init CAN BUS Shield again");
    delay(100);
  }

  SERIAL.println("CAN BUS Shield init ok!");
  delay(1000);

  // motor_readPID(MOTOR_ADDRESS);
//  motor_clear(MOTOR_ADDRESS);
//  motor_writeEncoderOffset(MOTOR_ADDRESS, -10000);
  delay(1000);

  // SERIAL.print("TIMER, TGTPOS, CURPOS, CURVEL, CURCUR");
  serialWriteTerminator();

  timer[0] = millis();
  motor_writePosition(MOTOR_ADDRESS, offset);
}

void loop()
{
  while (millis() - timer[0] < 5000)
  {
    timer[1] = millis();
    SERIAL.print(timer[1] - timer[0]);

    int32_t tgt_pos = A * sin(omega * (timer[1] - timer[0]) * 0.001) + offset;

    motor_writePosition(MOTOR_ADDRESS, tgt_pos);
    motor_readAngle(MOTOR_ADDRESS);
    serialWriteTerminator();

    timer[2] = millis() - timer[1];
    if (timer[2] < LOOPTIME)
    {
      delay(LOOPTIME - timer[2]);
    }
  }

//  motor_clear(MOTOR_ADDRESS);
  delay(500);
  SERIAL.println("Program finish!");
  while (true)
  {
    delay(100);
  }
}



// Function ----------------------------------------------------
void serialWriteTerminator()
{
  SERIAL.write(13);
  SERIAL.write(10);
}

void writeCmd(unsigned char *addr, unsigned char *buf)
{
  // CAN通信で送る
  unsigned char sendState = CAN.sendMsgBuf(addr, 0, 8, buf);
  if (sendState != CAN_OK)
  {
    SERIAL.println("Error Sending Message...");
    SERIAL.println(sendState);
  }
}

// Motor Command ----------------------------------------------
void motor_readState(unsigned char *addr)
{
  //check if data coming
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, reply_buf); //read data, len: data length, buf: data buf

    unsigned char cmd_byte = reply_buf[0];
    uint8_t temperature = reply_buf[1];
    int16_t cur = reply_buf[2] + (reply_buf[3] << 8);
    int16_t vel = reply_buf[4] + (reply_buf[5] << 8);
    // int16_t pos = reply_buf[6] + (reply_buf[7] << 8);    // 16bit以下のエンコーダならこれで読み取れる
    // long pos = motor_readAngle(addr);
//
//    SERIAL.print(",");
//    SERIAL.print(pos);
//    SERIAL.print(",");
//    SERIAL.print(vel);
//    SERIAL.print(",");
//    SERIAL.print(cur);
  }
}

void motor_readPID(unsigned char *addr)
{
  cmd_buf[0] = 0x30;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = 0x00;
  cmd_buf[5] = 0x00;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  // Send message
  writeCmd(addr, cmd_buf);
  delay(100);

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, reply_buf); //read data, len: data length, buf: data buf

    unsigned char cmd_byte = reply_buf[0];
    int posKp = reply_buf[2];
    int posKi = reply_buf[3];
    int velKp = reply_buf[4];
    int velKi = reply_buf[5];
    int iqKp  = reply_buf[6];
    int iqKi  = reply_buf[7];

    SERIAL.print("posKp:");
    SERIAL.print(posKp);
    SERIAL.print("posKi:");
    SERIAL.print(posKi);
    SERIAL.print("velKp:");
    SERIAL.print(velKp);
    SERIAL.print("velKi:");
    SERIAL.print(velKi);
    SERIAL.print("iqKp:");
    SERIAL.print(iqKp);
    SERIAL.print("iqKi:");
    SERIAL.println(iqKi);
  }
}

// to RAM (temp)
void motor_writePID(unsigned char *addr, int posKp, int posKi, int velKp, int velKi, int iqKp, int iqKi)
{
  cmd_buf[0] = 0x31;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = posKp;
  cmd_buf[3] = posKi;
  cmd_buf[4] = velKp;
  cmd_buf[5] = velKi;
  cmd_buf[6] = iqKp;
  cmd_buf[7] = iqKi;

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writeEncoderOffset(unsigned char *addr, uint16_t encoderOffset) {
  // encoder offset is uint16_t type.
  cmd_buf[0] = 0x91;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = 0x00;
  cmd_buf[5] = 0x00;
  cmd_buf[6] = encoderOffset & 0xFF;
  cmd_buf[7] = (encoderOffset >> 8) & 0xFF;

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_readAngle(unsigned char *addr)
{
  cmd_buf[0] = 0x92;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = 0x00;
  cmd_buf[5] = 0x00;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  // Send message
  writeCmd(addr, cmd_buf);
  delay(1);

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf
    if (buf[0] == 0x92)
    {
      reply_buf[0] = buf[0];
      reply_buf[1] = buf[1];
      reply_buf[2] = buf[2];
      reply_buf[3] = buf[3];
      reply_buf[4] = buf[4];
      reply_buf[5] = buf[5];
      reply_buf[6] = buf[6];
      reply_buf[7] = buf[7];
      
    }

    SERIAL.print(",");
    SERIAL.print(reply_buf[0]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[1]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[2]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[3]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[4]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[5]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[6]);
    SERIAL.print(",");
    SERIAL.print(reply_buf[7]);
  }
}

void motor_clear(unsigned char *addr)
{
  cmd_buf[0] = 0x80;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = 0x00;
  cmd_buf[5] = 0x00;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  writeCmd(addr, cmd_buf);
  SERIAL.println("Motor Clear ...");
}

void motor_writeCurrent(unsigned char *addr, int16_t current)
{
  // current control is int16_t type. (2byteの符号付き整数)
  cmd_buf[0] = 0xA1;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = current & 0xFF;
  cmd_buf[5] = (current >> 8) & 0xFF;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  SERIAL.print(",");
  SERIAL.print(current);

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writeVelocity(unsigned char *addr, int32_t velocity)
{
  // velocity control is int32_t type. (4byteの符号付き整数)
  cmd_buf[0] = 0xA2;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = velocity & 0xFF;
  cmd_buf[5] = (velocity >> 8) & 0xFF;
  cmd_buf[6] = (velocity >> 16) & 0xFF;
  cmd_buf[7] = (velocity >> 24) & 0xFF;

  SERIAL.print(",");
  SERIAL.print(velocity);

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writePosition(unsigned char *addr, int32_t position)
{
  // position control is int32_t type. (4byteの符号付き整数)
  cmd_buf[0] = 0xA3;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = position & 0xFF;
  cmd_buf[5] = (position >> 8) & 0xFF;
  cmd_buf[6] = (position >> 16) & 0xFF;
  cmd_buf[7] = (position >> 24) & 0xFF;

  SERIAL.print(",");
  SERIAL.print(position);

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writePosition_with_speedLimit(unsigned char *addr, uint16_t speedLimit, int32_t position) {
  // position control is int32_t type. (4byteの符号付き整数)
  cmd_buf[0] = 0xA4;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = speedLimit & 0xFF;
  cmd_buf[3] = (speedLimit >> 8) & 0xFF;
  cmd_buf[4] = position & 0xFF;
  cmd_buf[5] = (position >> 8) & 0xFF;
  cmd_buf[6] = (position >> 16) & 0xFF;
  cmd_buf[7] = (position >> 24) & 0xFF;

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writePosition_with_direction(unsigned char *addr, uint8_t direction, uint16_t position) {
  // position control is uint16_t type. (4byteの符号付き整数)
  // regarding direction, 0x00 is clockwise, 0x01 is counterclockwise.
  cmd_buf[0] = 0xA5;
  cmd_buf[1] = direction & 0xFF ;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = position & 0xFF;
  cmd_buf[5] = (position >> 8) & 0xFF;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writePosition_with_direction_and_speedLimit(unsigned char *addr, uint8_t direction, uint16_t speedLimit, uint16_t position) {
  // position control is uint16_t type. (4byteの符号付き整数)
  // regarding direction, 0x00 is clockwise, 0x01 is counterclockwise.
  cmd_buf[0] = 0xA6;
  cmd_buf[1] = direction & 0xFF ;
  cmd_buf[2] = speedLimit & 0xFF;
  cmd_buf[3] = (speedLimit >> 8) & 0xFF;
  cmd_buf[4] = position & 0xFF;
  cmd_buf[5] = (position >> 8) & 0xFF;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  // Send message
  writeCmd(addr, cmd_buf);
}
