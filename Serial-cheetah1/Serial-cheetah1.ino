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

int a ;

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);    //set CS PIN

void setup()
{
  SERIAL.begin(115200);
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS))   //init can bus : baudrate = 500k
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }
  SERIAL.println("CAN BUS Shield init ok!");

}

unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void loop()
{
  unsigned char len = 0;
  unsigned char buf[8];

  // send data: id = 0x00, standrad frame, data len = 8, stamp: data buf
  stmp[0] = 0xFF;
  stmp[1] = 0x01;
  stmp[2] = 0x00;
  stmp[3] = 0x24; //加速度
  stmp[4] = 0x00;
  stmp[5] = 0x24; //速度
  stmp[6] = 0;

  if(Serial.available() > 0){
    a = Serial.read();
    stmp[7] = a;
    }   //Pythonからのデータを受信
 
  
  CAN.sendMsgBuf(0x01, 0, 8, stmp);

  if (CAN_MSGAVAIL == CAN.checkReceive())   //check if data coming
  {
    CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf

    unsigned long canId = CAN.getCanId();

    SERIAL.println("-------------------------");
    SERIAL.print("Get dataID from : 0x");
    SERIAL.println(canId, HEX);

    for (int i = 0; i < len; i++) //print the data
    {
      SERIAL.print(buf[i], HEX);
      SERIAL.print("\t");
    }
    SERIAL.println();

  }
}
