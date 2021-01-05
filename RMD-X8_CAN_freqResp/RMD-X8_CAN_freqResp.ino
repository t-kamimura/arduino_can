/* Frequency responce test                        */
/* Original version developed by T. Kamimura      */
/* kamimura.tomoya@nitech.ac.jp                   */

/*------------------------------------------------*/
/* NOTICE                                         */
/* (1)                                            */
/* Before use, download RMDx8Arduino Library      */
/* from https://github.com/bump5236/RMDx8Arduino  */
/* developed by bump5235.                         */
/* (2)                                            */
/* Before use, install can-bus shield library     */
/* for details, please see                        */
/* https://learn.sparkfun.com/tutorials/can-bus-shield-hookup-guide?_ga=2.199310185.651624511.1609811680-1530731643.1609811680 */
/*------------------------------------------------*/

#include <mcp_can.h>
#include <SPI.h>
#include <RMDx8Arduino.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 115200
#define LOOPTIME 5 //[ms] 

unsigned long timer[3];

const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1 to 32)
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN); //set CS PIN
RMDx8Arduino rmd(CAN, MOTOR_ADDRESS);

int A = 20 * 6 * 100; // (servoHornDegree) * (gearRatio) * (encorderResolution)
double f = 2.0; // frequency [Hz]
double omega = 2*3.14*f;

int offset = 5 * 6 * 100;  // (OffsetAngle) * (gearRatio) * (encorderResolution)

void setup()
{
  SERIAL.begin(BAUDRATE);
  delay(1000);
  rmd.canSetup();
  delay(1000);
//  rmd.writePID(40, 100, 50, 40, 50, 50);
  rmd.writePosition(offset);
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
    int32_t angle = rmd.present_position * 0.01 / 6; // Calc present angle [deg] 

    // print
    SERIAL.print(timer[1] - timer[0]);
    SERIAL.print(",");
    SERIAL.print(angle);
//    serialDisp(rmd.reply_buf, rmd.pos_buf); // for debug
    rmd.serialWriteTerminator();  // Carriage Return & Line Feed
    SERIAL.print(tgt_pos);

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
