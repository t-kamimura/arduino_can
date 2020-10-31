#include <HT03.h>

/*-------------------------------------
ID  role
1   Root joint
2   Knee joint
---------------------------------------*/

/* -----------------------------------------
DEFINITIONS OF VARIABLES
-------------------------------------------*/

//Define LED pins
#define LED2 8
#define LED3 7
#define CLICK A4

#define MOTOR_ADDRESS_1 0x01
#define MOTOR_ADDRESS_2 0x02

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 10

#define K 500     // stiffness of virtual spring
#define PHI0 256
#define A 64
#define OMEGA 1

//the cs pin of the version after v1.1 is default to D9
//v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN

HT03 myMotor1(MOTOR_ADDRESS_1);
HT03 myMotor2(MOTOR_ADDRESS_2);

unsigned long timer[3];

/* -----------------------------------------
DEFINITIONS OF FUNCTIONS
-------------------------------------------*/
void serialWriteTerminator()
{
  Serial.write(13);
  Serial.write(10);
}

/* -----------------------------------------
MAIN
-------------------------------------------*/
void setup()
{
  Serial.begin(BAUDRATE);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(CLICK, INPUT);
  digitalWrite(CLICK, HIGH);
  myMotor1.pos_tgt = 0.5*PHI0;
  myMotor2.pos_tgt = -PHI0;
  myMotor1.kp = 0;//50;
  myMotor2.kp = 0;
  myMotor1.kd = 0;//50;
  myMotor2.kd = 0;
  delay(1000);
  while (CAN_OK != CAN.begin(CAN_1000KBPS)) //init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
  delay(1000);

  // モータートルクをオフにする
  myMotor1.disable(CAN);
  myMotor2.disable(CAN);
  delay(1000);

  Serial.println("Please set the leg straight...then CLICK");
  while (1)
  {
    if (digitalRead(CLICK) == LOW)
    {
      break;
    }
  }
  // Serial.print("position initializing...");
  myMotor1.zeroPosition(CAN); // 現在位置をゼロということにする
  myMotor2.zeroPosition(CAN); // 現在位置をゼロということにする
  digitalWrite(LED2, HIGH);
  delay(1000);

  Serial.println("Please set the initial position...then CLICK");
  while (1)
  {
    if (digitalRead(CLICK) == LOW)
    {
      break;
    }
  }
  myMotor1.writeCmd(CAN);
  myMotor2.writeCmd(CAN);
  serialWriteTerminator();
  delay(1000);

  myMotor2.enable(CAN); // モーターモードに入る
  myMotor1.enable(CAN); // モーターモードに入る
  digitalWrite(LED3, HIGH);
  delay(1000);

  Serial.print("TIMER, ID1, TGTPOS1, TGTKP1, TGTKD1, ID2, TGTPOS2, TGTKP2, TGTKD2, ID1, CURPOS1, CURVEL1, CURCUR1, ID2, CURPOS2, CURVEL2, CURCUR2");
  serialWriteTerminator();

  timer[0] = millis();
}

void loop()
{
  while (1)
  {
    if (digitalRead(CLICK) == LOW)
    {
      break;
    }
    timer[1] = millis();
    Serial.print(timer[1] - timer[0]);

    myMotor1.pos_tgt = -0.5*myMotor2.pos_cur;
    myMotor1.kp = K;
    myMotor1.kd = 0.5*K;
    myMotor1.writeCmd(CAN);
    
    myMotor2.pos_tgt = -PHI0 + A*cos(OMEGA*(timer[1] - timer[0])*0.001);
    myMotor2.kp = K;
    myMotor2.kd = 0;
    myMotor2.writeCmd(CAN);

    // Serial.println("receiving CAN msg...");
    myMotor2.readState(CAN);
    myMotor1.readState(CAN);

    serialWriteTerminator();

    timer[2] = millis() - timer[1];
    if (timer[2] < LOOPTIME)
    {
      delay(LOOPTIME - timer[2]);
    }
    else
    {
      Serial.print("time shortage: ");
      Serial.println(timer[2] - LOOPTIME);
    }
  }

  myMotor1.disable(CAN);
  myMotor2.disable(CAN);
  delay(500);
  Serial.println("Program finish!");
  digitalWrite(LED3, LOW);
  while (true)
  {
    delay(100);
  }
}
