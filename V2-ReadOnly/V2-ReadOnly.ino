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

//Define Joystick connection pins
#define UP A1
#define DOWN A3
#define LEFT A2
#define RIGHT A5
#define CLICK A4

//Define LED pins
#define LED2 8
#define LED3 7
#define MOTOR_ADDRESS 0x01
#define LOOPTIME 100

int a ;

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);    //set CS PIN

unsigned long timer[3];   // for controller
unsigned char can_msg6[6]; // 6 bytes CAN message
unsigned char can_msg8[8] = {0,0,0,0,0,0,0,0}; // 8 bytes CAN message

void setup()
{
  SERIAL.begin(9600);
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS))   //init can bus : baudrate = 500k
  {
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }
  SERIAL.println("CAN BUS Shield init ok!");
  
  // モーターモードに入り，トルクをオンにする
  SERIAL.print("Motor Ctrl ON...");
  can_msg6[0] = 0xFF;
  can_msg6[1] = 0x06;
  can_msg6[2] = 0x01;
  can_msg6[3] = 0x01;
  can_msg6[4] = 0x01;
  can_msg6[5] = 0x00;
  SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 6, can_msg6);
  delay(1000);
  /*
  // モーターモードをオフにする
  SERIAL.print("Motor Ctrl OFF...");
  can_msg6[0] = 0xFF;
  can_msg6[1] = 0x06;
  can_msg6[2] = 0x01;
  can_msg6[3] = 0x01;
  can_msg6[4] = 0x00;
  can_msg6[5] = 0x00;
  SERIAL.println("sending CAN msg...");
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 6, can_msg6);
  delay(1000);
  */
  
  //Initialize pins as necessary
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  digitalWrite(UP,HIGH);
  digitalWrite(DOWN,HIGH);
  
  timer[0] = millis();
}

void loop()
{
  timer[1] = millis();
  
  unsigned char len = 0;
  unsigned char buf[8];
  
  // send data: id = 0x00, standrad frame, data len = 8, stamp: data buf
  can_msg8[0] = 0xFF; // host address
  can_msg8[1] = 0x01; // mode
  can_msg8[2] = 0x00; // (retain)
  can_msg8[3] = 0x01; // Acceleration
  can_msg8[4] = 0x00; // (retain)
  can_msg8[5] = 0x01; // Velocity
  can_msg8[6] = 0; // Position (上8bit)
  //can_msg8[7] = 128; // Position (下8bit)
  if (digitalRead(UP)==LOW)
  {
    can_msg8[7] = can_msg8[7] + 1;
    if (can_msg8[7] == 255) {
      can_msg8[7] = 254;
    }
  }
  if (digitalRead(DOWN)==LOW)
  {
    can_msg8[7] = can_msg8[7] - 1;
    if (can_msg8[7] == 0){
      can_msg8[7] = 1;
    }
  }

  unsigned int pos_tgt = (can_msg8[6] << 8) + can_msg8[7];
      
  SERIAL.print(" MODE:");
  SERIAL.print(can_msg8[1]);
  SERIAL.print(" PosTGT:");
  SERIAL.print(pos_tgt);
  SERIAL.print(" VelTGT:");
  SERIAL.print(can_msg8[5]);
  SERIAL.print(" AccTGT:");
  SERIAL.print(can_msg8[3]);
  SERIAL.print("\t");
  
  CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, can_msg8);

  
  if (CAN_MSGAVAIL == CAN.checkReceive())   //check if data coming
  {
    CAN.readMsgBuf(&len, buf); //read data, len: data length, buf: data buf

    /*
    unsigned long canId = CAN.getCanId();
    //SERIAL.println("-------------------------");
    SERIAL.print("ID: 0x");
    SERIAL.print(canId, HEX);
    SERIAL.print("\t");

    for (int i = 0; i < len; i++) //print the data
    {
      //SERIAL.print(buf[i], HEX);
      SERIAL.print(buf[i]);
      SERIAL.print("\t");
    }
      SERIAL.println();
    */
    SERIAL.print(buf[0]);
    SERIAL.print("\t");
    SERIAL.print(buf[1]);
    SERIAL.print("\t");
    SERIAL.print(buf[2]);
    SERIAL.print("\t");
    SERIAL.print(buf[3]);
    SERIAL.print("\t");
    SERIAL.print(buf[4]);
    SERIAL.print("\t");
    SERIAL.print(buf[5]);
    SERIAL.print("\t");
    SERIAL.print(buf[6]);
    SERIAL.print("\t");
    SERIAL.print(buf[7]);
    SERIAL.println();
    /*
    if (buf[1]==1){
      int cur_cur = buf[3];
      int vel_cur = (buf[4] << 8) + buf[5];
      int pos_cur = (buf[6] << 8) + buf[7];
        
      SERIAL.print(" ID:");
      SERIAL.print(buf[0]);
      SERIAL.print(" MODE:");
      SERIAL.print(buf[1]);
      SERIAL.print(" Pos:");
      SERIAL.print(pos_cur);
      SERIAL.print(" Vel:");
      SERIAL.print(vel_cur);
      SERIAL.print(" Cur:");
      SERIAL.print(cur_cur);
      SERIAL.println();
      
    }else if (buf[1]==2){
      int cur_cur = buf[3];
      unsigned int pos_cur = (buf[6] << 8) + buf[7];
        
      SERIAL.print(" ID:");
      SERIAL.print(buf[0]);
      SERIAL.print(" MODE:");
      SERIAL.print(buf[1]);
      SERIAL.print(" TEMP:");
      SERIAL.print(buf[4]);
      SERIAL.print(" Pos:");
      SERIAL.print(pos_cur);
      SERIAL.print(" Cur:");
      SERIAL.print(cur_cur);
      SERIAL.println();
    }
    
    */

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
