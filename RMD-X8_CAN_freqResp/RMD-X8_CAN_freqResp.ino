/* 周波数応答確認用*/

#include <mcp_can.h>
#include <SPI.h>
#include <RMDx8Arduino.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 115200 //シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 5 //[ms] 

unsigned long timer[3];

const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1~32)
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN
RMDx8Arduino rmd(CAN, MOTOR_ADDRESS;

int A = 20000;
double f = 1.0; //[Hz]
double omega = 2*3.14*f;

void setup()
{
  SERIAL.begin(BAUDRATE);
  delay(1000);
  rmd.canSetup();
  delay(1000);

  rmd.clearState();
  delay(1000);

  rmd.writePID(40, 100, 50, 40, 50, 50);
  delay(1000);

  rmd.serialWriteTerminator();
  timer[0] = millis();
}

void loop()
{
  while (millis() - timer[0] < 5000)
  {
    timer[1] = millis();
    int32_t tgt_pos = A * sin(omega * (timer[1] - timer[0]) * 0.001);

    rmd.writePosition(tgt_pos);
    rmd.readPosition();

    // print
    SERIAL.print(timer[1] - timer[0]);
    SERIAL.print(",");
    SERIAL.print(tgt_pos);

    // byteならこっち
    // serialDisp(rmd.reply_buf, rmd.pos_buf);
    
    // 10進数ならこっち
    SERIAL.print(",");
    SERIAL.print(rmd.present_current);
    SERIAL.print(",");
    SERIAL.print(rmd.present_velocity);
    SERIAL.print(",");
    SERIAL.println(rmd.present_position);    //32bitまで


    rmd.serialWriteTerminator();

    timer[2] = millis() - timer[1];
    if (timer[2] < LOOPTIME)
    {
      delay(LOOPTIME - timer[2]);
    }
  }

  rmd.clearState();
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
