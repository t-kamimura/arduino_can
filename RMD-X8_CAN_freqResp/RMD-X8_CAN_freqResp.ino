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
RMDx8Arduino rmd(CAN);

int A = 65536*0.3;
double f = 1.0; //[Hz]
double omega = 2*3.14*f;

int offset = 65536*0.5;  //A < offset

void setup()
{
  SERIAL.begin(BAUDRATE);
  delay(1000);
  rmd.canSetup();
  delay(1000);
  rmd.writePID(MOTOR_ADDRESS, 40, 100, 50, 40, 50, 50);
  rmd.writePosition(MOTOR_ADDRESS, offset);
  delay(1000);

  rmd.serialWriteTerminator();
  timer[0] = millis();
}

void loop()
{
  while (millis() - timer[0] < 5000)
  {
    timer[1] = millis();
    int32_t tgt_pos = A * sin(omega * (timer[1] - timer[0]) * 0.001) + offset;

    rmd.writePosition(MOTOR_ADDRESS, tgt_pos);
    rmd.readAngle(MOTOR_ADDRESS, 1);

    // print
    SERIAL.print(timer[1] - timer[0]);
    SERIAL.print(",");
    SERIAL.print(tgt_pos);
    serialDisp(rmd.reply_buf, rmd.pos_buf);

    rmd.serialWriteTerminator();

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
