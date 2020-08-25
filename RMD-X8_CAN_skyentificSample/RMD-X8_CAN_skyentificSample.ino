// YoutubeのSkyentificさんの動画をもとに作成したサンプルプログラム
// demo: CAN-BUS Shield, send data

#include <mcp_can.h>
#include <SPI.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif

// Define Joystick connection pins
#define UP    A1
#define DOWN  A3
#define LEFT  A2
#define RIGHT A5
#define CLICK A4

// Define LED pins
#define LED2 8
#define LED3 7

#define StepValue 130

const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);

void setup()
{
  SERIAL.begin(115200);
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS))
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println("Init CAN BUS Shield again");
    delay(100);
  }
  SERIAL.println("CAN BUS Shield init ok!");

  // Initialize pins as necessary
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // Pull analog pins high to enable reading of joystick movements
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);

  // Write LED pins low to turn them off by default
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

}

long GenPos = 0;
void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];

  if (digitalRead(UP)==LOW)
  {
    GenPos = GenPos + StepValue;
    if (GenPos > 35999) {
        GenPos = max(GenPos - 35999, 0);
    }
    if (GenPos < 0) {
        GenPos = 0;
    }
  SERIAL.print("GenPos:");
  SERIAL.println(GenPos);

  buf[0] = 0xA6;
  buf[1] = 0x00;
  buf[2] = 0x02;
  buf[3] = 0x02;
  buf[4] = GenPos;
  buf[5] = GenPos >> 8;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  }

  if (digitalRead(DOWN)==LOW)
  {
    GenPos = GenPos - StepValue;
    if (GenPos >= 36000) {
        GenPos = 35999;
    }
    if (GenPos < 0) {
        GenPos = min(GenPos + 35999, 35999);
    }
  SERIAL.print("GenPos:");
  SERIAL.println(GenPos);

  buf[0] = 0xA6;
  buf[1] = 0x01;
  buf[2] = 0x02;
  buf[3] = 0x02;
  buf[4] = GenPos;
  buf[5] = GenPos >> 8;
  buf[6] = 0x00;
  buf[7] = 0x00;
  CAN.sendMsgBuf(0x141, 0, 8, buf);
  }

  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf
    
    unsigned long canId = CAN.getCanId();

    SERIAL.print("CMD");
    SERIAL.print(buf[0]);
    SERIAL.print("\t");
    
    SERIAL.print("Cur");
    SERIAL.print(buf[2] + buf[3] << 8);
    SERIAL.print("\t");

    SERIAL.print("Vel");
    SERIAL.print(buf[4] + buf[5] << 8);
    SERIAL.print("\t");

    SERIAL.print("Pos");
    SERIAL.print(buf[6] + buf[7] << 8);
    SERIAL.print("\t");
    
    SERIAL.println();
  }
}
