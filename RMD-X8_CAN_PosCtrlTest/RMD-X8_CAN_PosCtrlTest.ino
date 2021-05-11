/* 動作確認用 */

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

int32_t position = 0;
unsigned char len = 0;
unsigned char cmd_buf[8], reply_buf[8];

uint8_t temperature = 0;
int16_t present_current = 0;
int16_t present_velocity = 0;
int16_t encoder_pos = 0;

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

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

}

void loop() {
  for (int16_t i = 0; i < 100; i++) {

    // POSITION control command is int32_t type. (4byteの符号付き整数)
     int A = 15000; 
     position = A*sin(0.1*i);
     cmd_buf[0] = 0xA3;
     cmd_buf[1] = 0x00;
     cmd_buf[2] = 0x00;
     cmd_buf[3] = 0x00;
     cmd_buf[4] = position & 0xFF;
     cmd_buf[5] = (position >> 8) & 0xFF;
     cmd_buf[6] = (position >> 16) & 0xFF;
     cmd_buf[7] = (position >> 24) & 0xFF;

    // Send message
    unsigned char sendState = CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, cmd_buf);
    if (sendState != CAN_OK) {
      SERIAL.println("Error Sending Message...");
      SERIAL.println(sendState);
    }

    delay(50); // reply_bufを受け取るための遅延

    //check if data coming
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      CAN.readMsgBuf(&len, reply_buf); //read data, len: data length, buf: data buf
      
      unsigned char cmd_byte = reply_buf[0];
      temperature = reply_buf[1];
      present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
      present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
      encoder_pos = ((int16_t)reply_buf[7] << 8) + reply_buf[6];

      SERIAL.print("Cur:");
      SERIAL.print(present_current);
      SERIAL.print("\t");
      SERIAL.print("Vel:");
      SERIAL.print(present_velocity);
      SERIAL.print("\t");
      SERIAL.print("Pos:");
      SERIAL.print(encoder_pos);
      SERIAL.println();
    }
    delay(50);
  }
  
  SERIAL.println("Program finish!");
  while (true) {
    delay(100);
  }
}
