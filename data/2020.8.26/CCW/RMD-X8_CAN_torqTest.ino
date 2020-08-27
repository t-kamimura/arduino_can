/* 電流-トルク変換の確認*/

#include <mcp_can.h>
#include <SPI.h>
#include <RMDx8Arduino.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

// Define Joystick connection pins
#define UP    A1
#define DOWN  A3
#define LEFT  A2
#define RIGHT A5
#define CLICK A4

#define StepValue 10

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 5 //[ms] 

unsigned long timer[3], t;

const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1~32)
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN
RMDx8Arduino rmd(CAN);

void setup()
{
  SERIAL.begin(BAUDRATE);
  delay(1000);
  rmd.canSetup();
  delay(1000);
  rmd.clearState(MOTOR_ADDRESS);
  rmd.writePID(MOTOR_ADDRESS, 40, 100, 50, 40, 10, 50);
//  rmd.writePosition(MOTOR_ADDRESS, 100);

  delay(1000);

  // Initialize pins as necessary
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(CLICK, INPUT);
  // Pull analog pins high to enable reading of joystick movements
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(CLICK, HIGH);

  rmd.serialWriteTerminator();
  timer[0] = millis();
}

int16_t GenCur = 0;
bool exit_tf = false;


void loop()
{
  while (exit_tf==false)
  {
    timer[1] = millis();
    t = timer[1] - timer[0];

    if (digitalRead(CLICK)==LOW)
    {
      exit_tf = true;
    }

    if (t < 1000)
    {
      GenCur = -0.2*(t);
    }
    
    else if (1000 <= t && t < 5000)
    {
      GenCur = -200;
    }

    else if (5000 <= t && t < 6000)
    {
      GenCur = -0.2*(t-5000) -200;
    } 

    else if (6000 <= t && t < 10000)
    {
      GenCur = -400;
    }

    else if (10000 <= t && t < 11000)
    {
      GenCur = -0.2*(t-10000) - 400;
    } 

    else if (11000 <= t && t < 15000)
    {
      GenCur = -600;
    }

    else if (t > 15000) {
      GenCur = 0;
    }

    int16_t tgt_cur = GenCur;
    rmd.writeCurrent(MOTOR_ADDRESS, tgt_cur);

    // print
    SERIAL.print(t);
    SERIAL.print(",");
    SERIAL.print(tgt_cur);
    serialDisp(rmd.reply_buf, rmd.pos_buf);

    rmd.serialWriteTerminator();

    timer[2] = millis() - timer[1];
    if (timer[2] < LOOPTIME)
    {
      delay(LOOPTIME - timer[2]);
    }
  }

  rmd.clearState(MOTOR_ADDRESS);
  delay(500);
  SERIAL.println("Program finish!");
  while (true)
  {
    delay(100);
  }
}

void serialDisp(unsigned char *buf, unsigned char *pos_buf)
{
    // Serial Communication
    SERIAL.print(",");
    SERIAL.print(buf[0]);
    SERIAL.print(",");
    SERIAL.print(buf[1]);
    SERIAL.print(",");
    SERIAL.print(buf[2]);
    SERIAL.print(",");
    SERIAL.print(buf[3]);
    SERIAL.print(",");
    SERIAL.print(buf[4]);
    SERIAL.print(",");
    SERIAL.print(buf[5]);
    SERIAL.print(",");
    SERIAL.print(buf[6]);
    SERIAL.print(",");
    SERIAL.print(buf[7]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[0]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[1]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[2]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[3]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[4]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[5]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[6]);
    SERIAL.print(",");
    SERIAL.print(pos_buf[7]);
}
