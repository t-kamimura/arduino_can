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
#define LOOPTIME 5

int pos = 300;

unsigned long timer[3];

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

unsigned char len = 0;
unsigned char cmd_buf[8], reply_buf[8];

void setup() {
  SERIAL.begin(BAUDRATE);
  delay(1000);

  while (CAN_OK != CAN.begin(CAN_1000KBPS)) {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }

  SERIAL.println("CAN BUS Shield init ok!");
  delay(1000);

  timer[0] = millis();
}

void loop() {
  for (int16_t i = 0; i < 1000; i++) {
    timer[1] = millis();
    SERIAL.print("Time:");
    SERIAL.print(timer[1] - timer[0]);
    SERIAL.print("\t");

    motor_writeCurrent(MOTOR_ADDRESS, 0);
    motor_readStatus();
    delay(1000);
  }
  
  SERIAL.println("Program finish!");
  while (true) {
    delay(100);
  }
}



// Function ----------------------------------------------------
void writeCmd(unsigned char *addr, unsigned char *buf) {
  // CAN通信で送る
  unsigned char sendState = CAN.sendMsgBuf(addr, 0, 8, buf);
  if (sendState != CAN_OK) {
    SERIAL.println("Error Sending Message...");
    SERIAL.println(sendState);
  }
}
void motor_readStatus() {
  //check if data coming
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, reply_buf); //read data, len: data length, buf: data buf
    
    unsigned char cmd_byte = reply_buf[0];
    uint8_t temperature = reply_buf[1];
    int16_t cur = reply_buf[2] + (reply_buf[3] << 8);
    int16_t vel = reply_buf[4] + (reply_buf[5] << 8);
    uint16_t upos = reply_buf[6] + (reply_buf[7] << 8);

    SERIAL.print("Cur:");
    SERIAL.print(cur);
    SERIAL.print("\t");
    SERIAL.print("Vel:");
    SERIAL.print(vel);
    SERIAL.print("\t");
    SERIAL.print("Pos:");
    SERIAL.print(upos);
    SERIAL.println();

  } else {
    SERIAL.println("No Data");
  }
}

void motor_writeCurrent(unsigned char *addr, int16_t current) {
  // current control is int16_t type. (2byteの符号付き整数)
  cmd_buf[0] = 0xA1;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = current & 0xFF;
  cmd_buf[5] = (current >> 8) & 0xFF;
  cmd_buf[6] = 0x00;
  cmd_buf[7] = 0x00;

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writeVelocity(unsigned char *addr, int32_t velocity) {
  // velocity control is int32_t type. (4byteの符号付き整数)
  cmd_buf[0] = 0xA2;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = velocity & 0xFF;
  cmd_buf[5] = (velocity >> 8) & 0xFF;
  cmd_buf[6] = (velocity >> 16) & 0xFF;
  cmd_buf[7] = (velocity >> 24) & 0xFF;

  // Send message
  writeCmd(addr, cmd_buf);
}

void motor_writePosition(unsigned char *addr, int32_t position) {
  // position control is int32_t type. (4byteの符号付き整数)
  cmd_buf[0] = 0xA3;
  cmd_buf[1] = 0x00;
  cmd_buf[2] = 0x00;
  cmd_buf[3] = 0x00;
  cmd_buf[4] = position & 0xFF;
  cmd_buf[5] = (position >> 8) & 0xFF;
  cmd_buf[6] = (position >> 16) & 0xFF;
  cmd_buf[7] = (position >> 24) & 0xFF;

  // Send message
  writeCmd(addr, cmd_buf);
}

