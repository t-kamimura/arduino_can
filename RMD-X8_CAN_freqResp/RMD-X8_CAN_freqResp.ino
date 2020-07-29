/* 動作確認用*/

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
unsigned char cmd_buf[8], reply_buf[8];

int A = 25;
double f = 1; //[Hz]
double omega = 2*3.14*f;

void setup() 
{
  SERIAL.begin(BAUDRATE);
  delay(1000);

  while (CAN_OK != CAN.begin(CAN_1000KBPS)) 
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }

  SERIAL.println("CAN BUS Shield init ok!");
  delay(1000);

  motor_readPID(MOTOR_ADDRESS);
  motor_clear(MOTOR_ADDRESS);
  delay(1000);

  SERIAL.print("TIMER, TGTPOS, CURPOS, CURVEL, CURCUR");
  serialWriteTerminator();

  timer[0] = millis();
}

void loop() 
{
  while (millis() - timer[0] < 5000) 
  {
    timer[1] = millis();
    SERIAL.print(timer[1] - timer[0]);

    int16_t cur = A * sin(omega * (timer[1] - timer[0]) * 0.001);
    motor_writeCurrent(MOTOR_ADDRESS, cur);

    motor_readState();
    serialWriteTerminator();

    timer[2] = millis() - timer[1];
    if (timer[2] < LOOPTIME)
    {
      delay(LOOPTIME - timer[2]);
    }
  }
  
  motor_clear(MOTOR_ADDRESS);
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
void motor_readState() 
{
  //check if data coming
  if (CAN_MSGAVAIL == CAN.checkReceive()) 
  {
    CAN.readMsgBuf(&len, reply_buf); //read data, len: data length, buf: data buf
    
    unsigned char cmd_byte = reply_buf[0];
    uint8_t temperature = reply_buf[1];
    uint16_t ucur = (reply_buf[2] << 8) + reply_buf[3];
    uint16_t uvel = (reply_buf[4] << 8) + reply_buf[5];
    uint16_t upos = (reply_buf[6] << 8) + reply_buf[7];

    // // uint -> int
    // int cur = ucur - 2048;
    // int vel = uvel - 2048;
    // int pos = upos - 2048;

    SERIAL.print(",");
    SERIAL.print(upos);
    SERIAL.print(",");
    SERIAL.print(uvel);
    SERIAL.print(",");
    SERIAL.print(ucur);
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
