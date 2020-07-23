/*************************************************************************
* File Name          : Firmware_for_MegaPi.ino
* Author             : myan
* Updated            : myan
* Version            : V0e.01.017
* Date               : 03/19/2020
* Description        : Firmware for Makeblock Electronic modules with Scratch.  
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
* History:
* <Author>         <Time>         <Version>        <Descr>
* Mark Yan         2016/03/12     0e.01.001        build the new.
* Mark Yan         2016/05/04     0e.01.002        Added encoder and compass driver and fix some bugs.
* Mark Yan         2016/05/07     0e.01.003        Delete watchdog and add On board stepper driver.
* Mark Yan         2016/05/24     0e.01.004        Fix issue MBLOCK-1 and MBLOCK-12(JIRA issue).
* Mark Yan         2016/06/07     0e.01.005        Fix encoder speed issue.
* Mark Yan         2016/06/22     0e.01.006        Fix issue MAK-187 (bluetooth fatal error from MBLOCK-12)
* Mark Yan         2016/06/25     0e.01.007        Fix issue MBLOCK-38(limit switch return value).
* Mark Yan         2016/07/06     0e.01.008        Fix issue MBLOCK-61(ultrasonic distance limitations bug).
* Mark Yan         2016/07/27     0e.01.009        Add position parameters for encoder motor,fix issue MBLOCK-77.
* Mark Yan         2016/08/01     0e.01.010        Fix issue MBLOCK-109 MBLOCK-110(encoder motor exception handling negative).
* Mark Yan         2016/08/10     0e.01.011        Fix issue MBLOCK-126(on board encoder motor speed symbol).
* Mark Yan         2016/08/24     0e.01.012        Fix issue MBLOCK-171(Stepper online execution slow), MBLOCK-189(on board encoder motor reset issue).
* Mark Yan         2017/03/01     0e.01.013        fix RGB lights issue.
* Mark Yan         2017/06/21     0e.01.014        fix JIRA issue 668 710.
* Mark Yan         2018/01/03     0e.01.015        add the absolute motor move for encode motor & add new stepper command.
* payton           2018/07/30     0e.01.016        The "megapi_mode" is no longer saved when the power is broken. Default is "BLUETOOTH_MODE"
* Payton           2020/03/19     0e.01.017        Support raspberry pi python lib.
* Payton           2020/04/01     0e.01.018        Repair encoder motor bug.
**************************************************************************/
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//#define DEBUG_INFO
//#define DEBUG_INFO1

Servo servos[12];  
MeMegaPiDCMotor dc;
MeRGBLed led;
MeUltrasonicSensor *us = NULL;     //PORT_7
MePort generalDevice;
MeJoystick joystick;
MeStepperOnBoard steppers[4] = {MeStepperOnBoard(1),MeStepperOnBoard(2),MeStepperOnBoard(3),MeStepperOnBoard(4)};
MeBuzzer buzzer;
MeEncoderOnBoard encoders[4];
MeLineFollower line(PORT_8);
MeColorSensor *colorsensor  = NULL;

typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

union
{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
}val;

union
{
  uint8_t byteVal[8];
  double doubleVal;
}valDouble;

union
{
  uint8_t byteVal[2];
  int16_t shortVal;
}valShort;
MeModule modules[12];
#if defined(__AVR_ATmega32U4__) 
  int16_t analogs[12]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
#endif
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
  int16_t analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};
#endif
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
  int16_t analogs[16]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
#endif

int16_t len = 52;
int16_t servo_pins[12]={0,0,0,0,0,0,0,0,0,0,0,0};
//Just for MegaPi
int16_t moveSpeed = 180;
int16_t turnSpeed = 180;
int16_t minSpeed = 45;
int16_t factor = 23;
int16_t distance=0;
int16_t randnum = 0;                                                                               
int16_t LineFollowFlag=0;

#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define LINE_FOLLOW_MODE                     0x04

#define DATA_SERIAL                            0
#define DATA_SERIAL1                           1
#define DATA_SERIAL2                           2
#define DATA_SERIAL3                           3

uint8_t command_index = 0;
uint8_t megapi_mode = BLUETOOTH_MODE;
uint8_t index = 0;
uint8_t dataLen;
uint8_t prevc=0;
uint8_t BluetoothSource = DATA_SERIAL;
uint8_t serialRead;
uint8_t buffer[52];
uint8_t bufferBt1[52];
uint8_t bufferBt2[52];
double  lastTime = 0.0;
double  currentTime = 0.0;

float dt;

long blink_time = 0;

boolean isStart = false;
boolean isAvailable = false;
boolean leftflag;
boolean rightflag;
boolean start_flag = false;
boolean move_flag = false;
boolean blink_flag = false;

String mVersion = "0e.01.018";
//////////////////////////////////////////////////////////////////////////////////////

#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define POTENTIONMETER         4
#define JOYSTICK               5
#define SOUND_SENSOR           7
#define RGBLED                 8
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define PIRMOTION              15
#define LINEFOLLOWER           17
#define SHUTTER                20
#define LIMITSWITCH            21
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define STEPPER                40
#define TIMER                  50
#define COMMON_COMMONCMD       60
  //Secondary command
  #define SET_STARTER_MODE     0x10
  #define SET_AURIGA_MODE      0x11
  #define SET_MEGAPI_MODE      0x12
  #define GET_BATTERY_POWER    0x70
  #define GET_AURIGA_MODE      0x71
  #define GET_MEGAPI_MODE      0x72
#define ENCODER_BOARD 61
  //Read type
  #define ENCODER_BOARD_POS    0x01
  #define ENCODER_BOARD_SPEED  0x02

#define ENCODER_PID_MOTION     62
  //Secondary command
  #define ENCODER_BOARD_POS_MOTION_MOVE    0x01
  #define ENCODER_BOARD_SPEED_MOTION       0x02
  #define ENCODER_BOARD_PWM_MOTION         0x03
  #define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
  #define ENCODER_BOARD_CAR_POS_MOTION     0x05
  #define ENCODER_BOARD_POS_MOTION_MOVETO  0x06

#define STEPPER_NEW            76
  //Secondary command
  #define STEPPER_POS_MOTION_MOVE          0x01
  #define STEPPER_SPEED_MOTION             0x02
  #define STEPPER_SET_CUR_POS_ZERO         0x04
  #define STEPPER_POS_MOTION_MOVETO        0x06

#define COLORSENSOR            67
  //Secondary command
  #define GETRGB                           0x01
  #define GETBOOL                          0x02
  #define GETCOLOR                         0x03

#define GET 1
#define RUN 2
#define RESET 4
#define START 5

/**
 * \par Function
 *    encoder_move_finish_callback
 * \par Description
 *    This function called when encoder motor move finish.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void encoder_move_finish_callback(int slot,int extId)
{
  writeHead();
  writeSerial(extId);
  sendByte(slot);
  writeEnd();
}

/**
 * \par Function
 *    stepper_move_finish_callback
 * \par Description
 *    This function called when stepper motor move finish.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void stepper_move_finish_callback(int slot,int extId)
{
  writeHead();
  writeSerial(extId);
  sendByte(slot);
  writeEnd();
}
/**
 * \par Function
 *    isr_process_encoder1
 * \par Description
 *    This function use to process the interrupt of encoder1 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder1 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder1(void)
{
  if(digitalRead(encoders[0].getPortB()) == 0)
  {
    encoders[0].pulsePosMinus();
  }
  else
  {
    encoders[0].pulsePosPlus();;
  }
}

/**
 * \par Function
 *    isr_process_encoder2
 * \par Description
 *    This function use to process the interrupt of encoder2 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder2 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder2(void)
{
  if(digitalRead(encoders[1].getPortB()) == 0)
  {
    encoders[1].pulsePosMinus();
  }
  else
  {
    encoders[1].pulsePosPlus();
  }
}

/**
 * \par Function
 *    isr_process_encoder3
 * \par Description
 *    This function use to process the interrupt of encoder3 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder3 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder3(void)
{
  if(digitalRead(encoders[2].getPortB()) == 0)
  {
    encoders[2].pulsePosMinus();
  }
  else
  {
    encoders[2].pulsePosPlus();
  }
}

/**
 * \par Function
 *    isr_process_encoder4
 * \par Description
 *    This function use to process the interrupt of encoder4 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder4 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder4(void)
{
  if(digitalRead(encoders[3].getPortB()) == 0)
  {
    encoders[3].pulsePosMinus();
  }
  else
  {
    encoders[3].pulsePosPlus();
  }
}

/**
 * \par Function
 *    WriteMegapiModeToEEPROM
 * \par Description
 *    This function use to write the MegaPi Mode configuration parameter to EEPROM.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void WriteMegapiModeToEEPROM(void)
{
  EEPROM.write(MEGAPI_MODE_PARTITION_CHECK, EEPROM_IF_HAVEPID_CHECK1);
  EEPROM.write(MEGAPI_MODE_PARTITION_CHECK + 1, EEPROM_IF_HAVEPID_CHECK2);
  EEPROM.write(MEGAPI_MODE_START_ADDR, EEPROM_CHECK_START);
  EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
  EEPROM.write(MEGAPI_MODE_END_ADDR, EEPROM_CHECK_END);
}

/**
 * \par Function
 *    readEEPROM
 * \par Description
 *    This function use to read the configuration parameters from EEPROM.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readEEPROM(void)
{
  if((EEPROM.read(MEGAPI_MODE_PARTITION_CHECK) == EEPROM_IF_HAVEPID_CHECK1) && (EEPROM.read(MEGAPI_MODE_PARTITION_CHECK + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    if((EEPROM.read(MEGAPI_MODE_START_ADDR)  == EEPROM_CHECK_START) && (EEPROM.read(MEGAPI_MODE_END_ADDR)  == EEPROM_CHECK_END))
    {
      EEPROM.get(MEGAPI_MODE_CONFIGURE, megapi_mode);
#ifdef DEBUG_INFO
      Serial.print( "Read megapi_mode from EEPROM:");
      Serial.println(megapi_mode);
#endif
    }
    else
    {
      Serial.println( "Data area damage on megapi mode!" );
    }
  }
  else
  {
#ifdef DEBUG_INFO
    Serial.println( "First written auriga mode!" );
#endif
    WriteMegapiModeToEEPROM();
  }
}

/**
 * \par Function
 *    Forward
 * \par Description
 *    This function use to control the car kit go forward.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Forward(void)
{
  encoders[0].setMotorPwm(moveSpeed);
  encoders[1].setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    Backward
 * \par Description
 *    This function use to control the car kit go backward.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Backward(void)
{
  encoders[0].setMotorPwm(-moveSpeed);
  encoders[1].setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    BackwardAndTurnLeft
 * \par Description
 *    This function use to control the car kit go backward and turn left.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void BackwardAndTurnLeft(void)
{
  encoders[0].setMotorPwm(-moveSpeed/4);
  encoders[1].setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    BackwardAndTurnRight
 * \par Description
 *    This function use to control the car kit go backward and turn right.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void BackwardAndTurnRight(void)
{
  encoders[0].setMotorPwm(-moveSpeed);
  encoders[1].setMotorPwm(moveSpeed/4);
}

/**
 * \par Function
 *    TurnLeft
 * \par Description
 *    This function use to control the car kit go backward and turn left.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnLeft(void)
{
  encoders[0].setMotorPwm(moveSpeed);
  encoders[1].setMotorPwm(-moveSpeed/2);
}

/**
 * \par Function
 *    TurnRight
 * \par Description
 *    This function use to control the car kit go backward and turn right.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnRight(void)
{
  encoders[0].setMotorPwm(moveSpeed/2);
  encoders[1].setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    TurnLeft1
 * \par Description
 *    This function use to control the car kit go backward and turn left(fast).
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnLeft1(void)
{
  encoders[0].setMotorPwm(moveSpeed);
  encoders[1].setMotorPwm(moveSpeed);
}

/**
 * \par Function
 *    TurnRight1
 * \par Description
 *    This function use to control the car kit go backward and turn right(fast).
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void TurnRight1(void)
{
  encoders[0].setMotorPwm(-moveSpeed);
  encoders[1].setMotorPwm(-moveSpeed);
}

/**
 * \par Function
 *    Stop
 * \par Description
 *    This function use to stop the car kit.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void Stop(void)
{
  encoders[0].setMotorPwm(0);
  encoders[1].setMotorPwm(0);
}

/**
 * \par Function
 *    ChangeSpeed
 * \par Description
 *    This function use to change the speed of car kit.
 * \param[in]
 *    spd - the speed of car kit(-255 ~ 255)
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void ChangeSpeed(int16_t spd)
{
  moveSpeed = spd;
}

/**
 * \par Function
 *    readBuffer
 * \par Description
 *    This function use to read the serial data from its buffer..
 * \param[in]
 *    index - The first address in the array
 * \par Output
 *    None
 * \return
 *    The data need to be read.
 * \par Others
 *    None
 */
uint8_t readBuffer(int16_t index)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    return buffer[index];
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    return bufferBt1[index];
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    return bufferBt2[index];
  }
}

/**
 * \par Function
 *    writeBuffer
 * \par Description
 *    This function use to write the serial data to its buffer..
 * \param[in]
 *    index - The data's first address in the array
  * \param[in]
 *    c - The data need to be write.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeBuffer(int16_t index,uint8_t c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    buffer[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    bufferBt1[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    bufferBt2[index]=c;
  }
}

/**
 * \par Function
 *    writeHead
 * \par Description
 *    This function use to write the head of transmission frame.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeHead(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
}

/**
 * \par Function
 *    writeEnd
 * \par Description
 *    This function use to write the terminator of transmission frame.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeEnd(void)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.println();
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.println();
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.println();
  }
}

/**
 * \par Function
 *    writeSerial
 * \par Description
 *    This function use to write the data to serial.
 * \param[in]
 *    c - The data need to be write.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeSerial(uint8_t c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.write(c);
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.write(c);
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.write(c);
  }
}

/**
 * \par Function
 *    readSerial
 * \par Description
 *    This function use to read the data from serial, and fill the data
 *    to its buffer.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readSerial(void)
{
  isAvailable = false;
  if(Serial.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL;
    serialRead = Serial.read();
  }
  else if(Serial2.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL2;
    serialRead = Serial2.read();
  }
  else if(Serial3.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL3;
    serialRead = Serial3.read();
  }
}

/**
 * \par Function
 *    parseData
 * \par Description
 *    This function use to process the data from the serial port,
 *    call the different treatment according to its action.
 *    ff 55 len idx action device port  slot  data a
 *    0  1  2   3   4      5      6     7     8
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void parseData(void)
{
  isStart = false;
  uint8_t idx = readBuffer(3);
  uint8_t action = readBuffer(4);
  uint8_t device = readBuffer(5);
  command_index = (uint8_t)idx;
  switch(action)
  {
    case GET:
      {
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN:
      {
        runModule(device);
        callOK();
      }
      break;
    case RESET:
      {
        /* reset On-Board encoder driver */
        for(int i=0;i<4;i++)
        {
          encoders[i].setPulsePos(0);
          encoders[i].moveTo(0,10);
          encoders[i].setMotorPwm(0);
          encoders[i].setMotionMode(DIRECT_MODE);
          steppers[i].setCurrentPosition(0);
          steppers[i].moveTo(0);
          steppers[i].disableOutputs();
        }

        /* reset dc motor on driver port */
        dc.reset(PORT1A);
        dc.run(0);
        dc.reset(PORT1B);
        dc.run(0);
        dc.reset(PORT2A);
        dc.run(0);
        dc.reset(PORT2B);
        dc.run(0);
        dc.reset(PORT3A);
        dc.run(0);
        dc.reset(PORT3B);
        dc.run(0);
        dc.reset(PORT4A);
        dc.run(0);
        dc.reset(PORT4B);
        dc.run(0);

        /* reset stepper motor driver */
        
        callOK();
      }
      break;
     case START:
      {
        callOK();
      }
      break;
  }
}

/**
 * \par Function
 *    callOK
 * \par Description
 *    Response for executable commands.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void callOK(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

/**
 * \par Function
 *    sendByte
 * \par Description
 *    Send byte data
 * \param[in]
 *    c - the byte data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendByte(uint8_t c)
{
  writeSerial(1);
  writeSerial(c);
}

/**
 * \par Function
 *    sendString
 * \par Description
 *    Send string data
 * \param[in]
 *    s - the string data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendString(String s)
{
  int16_t l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int16_t i=0;i<l;i++)
  {
    writeSerial(s.charAt(i));
  }
}

/**
 * \par Function
 *    sendFloat
 * \par Description
 *    Sned float data
 * \param[in]
 *    value - the float data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendFloat(float value)
{ 
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
 * \par Function
 *    sendLong
 * \par Description
 *    Sned long data
 * \param[in]
 *    value - the long data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendLong(long value)
{ 
  writeSerial(6);
  val.longVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
 * \par Function
 *    sendShort
 * \par Description
 *    Sned short data
 * \param[in]
 *    value - the short data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendShort(int16_t value)
{
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

/**
 * \par Function
 *    sendDouble
 * \par Description
 *    Sned double data, same as float data on arduino.
 * \param[in]
 *    value - the double data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendDouble(double value)
{
  writeSerial(5);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}

/**
 * \par Function
 *    readShort
 * \par Description
 *    read the short data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the short data.
 * \par Others
 *    None
 */
int16_t readShort(int16_t idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}

/**
 * \par Function
 *    readFloat
 * \par Description
 *    read the float data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the float data.
 * \par Others
 *    None
 */
float readFloat(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}

/**
 * \par Function
 *    readLong
 * \par Description
 *    read the long data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the long data.
 * \par Others
 *    None
 */
long readLong(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.longVal;
}

char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};

/**
 * \par Function
 *    readString
 * \par Description
 *    read the string data.
 * \param[in]
 *    idx - The string's first address in the array.
 * \param[in]
 *    len - The length of the string data.
 * \par Output
 *    None
 * \return
 *    the address of string data.
 * \par Others
 *    None
 */
char* readString(int16_t idx,int16_t len)
{
  for(int16_t i=0;i<len;i++)
  {
    _receiveStr[i]=readBuffer(idx+i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}

/**
 * \par Function
 *    readUint8
 * \par Description
 *    read the uint8 data.
 * \param[in]
 *    idx - The Uint8 data's first address in the array.
 * \param[in]
 *    len - The length of the uint8 data.
 * \par Output
 *    None
 * \return
 *    the address of uint8 data.
 * \par Others
 *    None
 */
uint8_t* readUint8(int16_t idx,int16_t len)
{
  for(int16_t i=0;i<len;i++)
  {
    if(i > 15)
    {
      break;
    }
    _receiveUint8[i] = readBuffer(idx+i);
  }
  return _receiveUint8;
}

/**
 * \par Function
 *    initStepper
 * \par Description
 *    Initialize acceleration, subdivision, and speed for stepper motor.
 * \param[in]
 *    index - The index of stepper.
 * \param[in]
 *    maxSpeed - The max speed of stepper.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void initStepper(uint8_t index,int16_t maxSpeed)
{
  // steppers[index].setpin(index+1);

  steppers[index].setMaxSpeed(maxSpeed);
  steppers[index].setAcceleration(20000);
  steppers[index].setMicroStep(16);
  steppers[index].setSpeed(maxSpeed);
  steppers[index].enableOutputs();
}
/**
 * \par Function
 *    runModule
 * \par Description
 *    Processing execute commands.
 * \param[in]
 *    device - The definition of all execute commands.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void runModule(uint8_t device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  uint8_t port = readBuffer(6);
  uint8_t pin = port;
  switch(device)
  {
    case MOTOR:
      {
        int16_t speed = readShort(7);
        dc.reset(port);
        dc.run(speed);
      }
      break;
    case ENCODER_BOARD:
      if(port == 0)
      {
        uint8_t slot = readBuffer(7);
        int16_t speed_value = readShort(8);
        speed_value = -speed_value;
        encoders[slot-1].setTarPWM(speed_value);
      }
      break;
    case JOYSTICK:
      {
        int16_t leftSpeed = readShort(6);
        encoders[0].setTarPWM(-leftSpeed);
        int16_t rightSpeed = readShort(8);
        encoders[1].setTarPWM(-rightSpeed);
      }
      break;
    case STEPPER_NEW:
      {
        uint8_t subcmd = port;
        uint8_t extID = readBuffer(3);
        uint8_t slot_num = readBuffer(7);
        int16_t maxSpeed = 0;
        if(STEPPER_POS_MOTION_MOVE == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          maxSpeed = abs(speed_temp);

          initStepper(slot_num - 1,maxSpeed);
          steppers[slot_num - 1].move(pos_temp,extID,stepper_move_finish_callback);
        }
        if(STEPPER_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);

          initStepper(slot_num - 1,speed_temp);
          steppers[slot_num - 1].setSpeed(speed_temp);
        }
        if(STEPPER_POS_MOTION_MOVETO == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          maxSpeed = abs(speed_temp);

          initStepper(slot_num - 1,maxSpeed);
          steppers[slot_num - 1].moveTo(pos_temp,extID,stepper_move_finish_callback);
        }
        else if(STEPPER_SET_CUR_POS_ZERO == subcmd)
        {
          steppers[slot_num - 1].setCurrentPosition(0);
        }
      }
      break;
    case STEPPER:
      {
        int16_t maxSpeed = readShort(7);
        long distance = readLong(9);
        steppers[port - 1] = MeStepperOnBoard(port);
        initStepper(port - 1,maxSpeed);
        steppers[port - 1].moveTo(distance);
      } 
      break;
    case RGBLED:
      {
        uint8_t slot = readBuffer(7);
        uint8_t idx = readBuffer(8);
        uint8_t pixels_len = readBuffer(2) - 6;
        if((port != 0) && ((led.getPort() != port) || (led.getSlot() != slot)))
        {
          led.reset(port,slot);
        }
        if(idx>0)
        {
          for(uint8_t i=0;i<pixels_len;i+=3)
          {
            led.setColorAt(idx+i/3-1,readBuffer(9+i),readBuffer(10+i),readBuffer(11+i)); 
          }
        }
        else
        {
          led.setColor(readBuffer(9),readBuffer(10),readBuffer(11)); 
        }
        led.show();
      }
      break;
    case COMMON_COMMONCMD:
      {
        uint8_t subcmd = port;
        uint8_t cmd_data = readBuffer(7);
        if(SET_MEGAPI_MODE == subcmd)
        {
          Stop();
          if((cmd_data == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE) || 
             (cmd_data == BLUETOOTH_MODE) ||
             (cmd_data == LINE_FOLLOW_MODE))
          {
            megapi_mode = cmd_data;
            if(EEPROM.read(MEGAPI_MODE_CONFIGURE) != megapi_mode)
            {
              EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
            }
          }
          else
          {
            megapi_mode = BLUETOOTH_MODE;
            if(EEPROM.read(MEGAPI_MODE_CONFIGURE) != megapi_mode)
            {
              EEPROM.write(MEGAPI_MODE_CONFIGURE, megapi_mode);
            }
          }
        }
      }
      break;
    case SERVO:
      {
        uint8_t slot = readBuffer(7);
        pin = slot==1?mePort[port].s1:mePort[port].s2;
        uint8_t v = readBuffer(8);
        Servo sv = servos[searchServoPin(pin)];
        if(v >= 0 && v <= 180)
        {
          if(!sv.attached())
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case SHUTTER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
        }
        uint8_t v = readBuffer(7);
        if(v < 2)
        {
          generalDevice.dWrite1(v);
        }
        else
        {
          generalDevice.dWrite2(v-2);
        }
      }
      break;
    case DIGITAL:
      {
        pinMode(pin,OUTPUT);
        uint8_t v = readBuffer(7);
        digitalWrite(pin,v);
     }
     break;
    case PWM:
      {
        pinMode(pin,OUTPUT);
        uint8_t v = readBuffer(7);
        analogWrite(pin,v);
      }
      break;
    case SERVO_PIN:
      {
        uint8_t v = readBuffer(7);
        if(v >= 0 && v <= 180)
        {
          Servo sv = servos[searchServoPin(pin)];
          if(!sv.attached())
          {
            sv.attach(pin);
          }
          sv.write(v);
        }
      }
      break;
    case TIMER:
      {
        lastTime = millis()/1000.0; 
      }
      break;
    case ENCODER_PID_MOTION:
      {
        uint8_t subcmd = port;
        uint8_t extID = readBuffer(3);
        uint8_t slot_num = readBuffer(7);
        if(ENCODER_BOARD_POS_MOTION_MOVE == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          encoders[slot_num-1].move(pos_temp,(float)speed_temp,extID,encoder_move_finish_callback);
        }
        if(ENCODER_BOARD_POS_MOTION_MOVETO == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          encoders[slot_num-1].moveTo(pos_temp,(float)speed_temp,extID,encoder_move_finish_callback);
        }
        else if(ENCODER_BOARD_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);  
          encoders[slot_num-1].runSpeed((float)speed_temp);
        }
        else if(ENCODER_BOARD_PWM_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);  
          encoders[slot_num-1].setTarPWM(speed_temp);     
        }
        else if(ENCODER_BOARD_SET_CUR_POS_ZERO == subcmd)
        {
          encoders[slot_num-1].setPulsePos(0);     
        }
        else if(ENCODER_BOARD_CAR_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          if(slot_num == 1)
          {
            encoders[0].move(pos_temp,(float)speed_temp);
            encoders[1].move(-pos_temp,(float)speed_temp);
          }
          else if(slot_num == 2)
          {
            encoders[0].move(-pos_temp,(float)speed_temp);
            encoders[1].move(pos_temp,(float)speed_temp);
          }
          else if(slot_num == 3)
          {
            encoders[0].move(pos_temp,(float)speed_temp);
            encoders[1].move(pos_temp,(float)speed_temp);
          }
          else if(slot_num == 4)
          {
            encoders[0].move(-pos_temp,(float)speed_temp);
            encoders[1].move(-pos_temp,(float)speed_temp);
          }
        }
      }
      break;
  }
}

/**
 * \par Function
 *    searchServoPin
 * \par Description
 *    Check if the pin has been allocated, if it is not allocated,
 *    then allocate it.
 * \param[in]
 *    pin - arduino gpio number
 * \par Output
 *    None
 * \return
 *    the servo number be assigned
 * \par Others
 *    None
 */
int16_t searchServoPin(int16_t pin)
{
  for(uint8_t i=0;i<12;i++)
  {
    if(servo_pins[i] == pin)
    {
      return i;
    }
    if(servo_pins[i] == 0)
    {
      servo_pins[i] = pin;
      return i;
    }
  }
  return 0;
}
/**
 * \par Function
 *    readSensor
 * \par Description
 *    This function is used to process query command.
 * \param[in]
 *    device - The definition of all query commands.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readSensor(uint8_t device)
{
  /**************************************************
      ff 55 len idx action device port slot data a
      0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value=0.0;
  uint8_t port,slot,pin;
  port = readBuffer(6);
  pin = port;
  writeHead();
  writeSerial(command_index);
  switch(device)
  {
    case ULTRASONIC_SENSOR:
      {
        if(us == NULL)
        {
          us = new MeUltrasonicSensor(port);
        }
        else if(us->getPort() != port)
        {
          delete us;
          us = new MeUltrasonicSensor(port);
        }
        value = (float)us->distanceCm();
        sendFloat(value);
      }
      break;
    case SOUND_SENSOR:
    case POTENTIONMETER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.aRead2();
        sendFloat(value);
      }
      break;
    case JOYSTICK:
      {
        slot = readBuffer(7);
        if(joystick.getPort() != port)
        {
          joystick.reset(port);
        }
        if(slot==0)
        {
          sendShort(joystick.read(1));
          sendShort(joystick.read(2));
        }
        else
        {
          value = joystick.read(slot);
          sendFloat(value);
        }
      }
      break;
    case PIRMOTION:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case LINEFOLLOWER:
      {
        if(generalDevice.getPort() != port)
        {
          generalDevice.reset(port);
          pinMode(generalDevice.pin1(),INPUT);
          pinMode(generalDevice.pin2(),INPUT);
        }
        value = generalDevice.dRead1()*2+generalDevice.dRead2();
        sendFloat(value);
      }
      break;
    case LIMITSWITCH:
      {
        slot = readBuffer(7);
        if(generalDevice.getPort() != port || generalDevice.getSlot() != slot)
        {
          generalDevice.reset(port,slot);
        }
        if(slot == 1)
        {
          pinMode(generalDevice.pin1(),INPUT_PULLUP);
          value = !generalDevice.dRead1();
        }
        else
        {
          pinMode(generalDevice.pin2(),INPUT_PULLUP);
          value = !generalDevice.dRead2();
        }
        sendFloat(value);  
      }
      break;
    case COLORSENSOR:
      {
        uint8_t colorsubcmd = 0;
        uint8_t colorindex = 0;
        uint8_t result = 0;
        uint32_t rgbcode = 0;
     
        colorsubcmd = readBuffer(7);
        colorindex  = readBuffer(8);

        if(colorsensor == NULL)
        {
          colorsensor = new MeColorSensor(port);
        }
        else if(colorsensor->getPort() != port)
        {
          delete colorsensor;
          colorsensor = new MeColorSensor(port);
        }
        
        if(colorsubcmd == GETRGB)
        {
          if(colorindex == 0x00)//r
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            result = (uint8_t)(rgbcode>>16);
          }
          else if(colorindex == 0x01)//g
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            result = (uint8_t)(rgbcode>>8);
          }
          else if(colorindex == 0x02)//b
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            result = (uint8_t)rgbcode;
          }
          else if(colorindex == 0x03)//rgb
          {
            colorsensor->ColorDataReadOnebyOne();
            rgbcode = colorsensor->ReturnColorCode();
            sendLong(rgbcode);
          }
        }
        else if(colorsubcmd == GETBOOL)
        {
          result = colorsensor->ColorIdentify();
          if(colorindex == 0x00)
          {
            if(result == WHITE)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x02)
          {
            if(result == RED)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x04)
          {
            if(result == YELLOW)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x05)
          {
            if(result == GREEN)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x07)
          {
            if(result == BLUE)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
          else if(colorindex == 0x09)
          {
            if(result == BLACK)
            {
              result =0x01;
            }
            else
            {
              result =0x00;
            }
          }
        }
        sendByte(result);
      }
      break;
    case VERSION:
      {
        sendString(mVersion);
      }
      break;
    case DIGITAL:
      {
        pinMode(pin,INPUT);
        sendFloat(digitalRead(pin));
      }
      break;
    case ANALOG:
      {
        pin = analogs[pin];
        pinMode(pin,INPUT);
        sendFloat(analogRead(pin));
      }
      break;
    case PULSEIN:
      {
        int16_t pw = readShort(7);
        pinMode(pin, INPUT);
        sendLong(pulseIn(pin,HIGH,pw));
      }
      break;
    case ULTRASONIC_ARDUINO:
      {
        uint8_t trig = readBuffer(6);
        uint8_t echo = readBuffer(7);
        long pw_data;
        float dis_data;
        pinMode(trig,OUTPUT);
        digitalWrite(trig,LOW);
        delayMicroseconds(2);
        digitalWrite(trig,HIGH);
        delayMicroseconds(10);
        digitalWrite(trig,LOW);
        pinMode(echo, INPUT);
        pw_data = pulseIn(echo,HIGH,30000);
        dis_data = pw_data/58.0;
        delay(5);
        writeHead();
        writeSerial(command_index);
        sendFloat(pw_data);
      }
      break;
    case TIMER:
      {
        sendFloat((float)currentTime);
      }
      break;
    case ENCODER_BOARD:
      {
        if(port == 0)
        {
          slot = readBuffer(7);
          uint8_t read_type = readBuffer(8);
          if(read_type == ENCODER_BOARD_POS)
          {
            sendLong(encoders[slot-1].getCurPos());
          }
          else if(read_type == ENCODER_BOARD_SPEED)
          {
            sendFloat(encoders[slot-1].getCurrentSpeed());
          }
        }
      }
      break;
    case COMMON_COMMONCMD:
      {
        uint8_t subcmd = port;
        if(GET_MEGAPI_MODE == subcmd)
        {
          sendByte(megapi_mode);
        }
      }
      break;
    default:
      {
        sendFloat(0);
      }
      break;
  }//switch
}

/**
 * \par Function
 *    ultrCarProcess
 * \par Description
 *    The main function for ultrasonic automatic obstacle avoidance
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void ultrCarProcess(void)
{
  if(us == NULL)
  {
    us = new MeUltrasonicSensor(PORT_7);
  }
  moveSpeed = 150;
  if(us != NULL)
  {
    distance = us->distanceCm();
  }
  else
  {
    return;
  }

  if((distance > 20) && (distance < 40))
  {
    randnum=random(300);
    if((randnum > 190) && (!rightflag))
    {
      leftflag=true;
      TurnLeft();
    }
    else
    {
      rightflag=true;
      TurnRight();  
    }
  }
  else if((distance < 20) && (distance > 0))
  {
    randnum=random(300);
    if(randnum > 190)
    {
      BackwardAndTurnLeft();
      for(int16_t i=0;i<300;i++)
      {
        if (read_serial())
        {
          break;
        }
        else
        {
          delay(2);
        }
      }
    }
    else
    {
      BackwardAndTurnRight();
      for(int i=0;i<300;i++)
      {
        if (read_serial())
        {
          break;
        }
        else
        {
          delay(2);
        }
      }
    }
  }
  else
  {
    leftflag=false;
    rightflag=false;
    Forward();
  }
}

/**
 * \par Function
 *    line_model
 * \par Description
 *    The main function for Patrol Line navigation mode
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void line_model(void)
{
  uint8_t val;
  val = line.readSensors();
  moveSpeed=120;
  switch (val)
  {
    case S1_IN_S2_IN:
      Forward();
      LineFollowFlag=10;
      break;

    case S1_IN_S2_OUT:
       Forward();
      if (LineFollowFlag>1) LineFollowFlag--;
      break;

    case S1_OUT_S2_IN:
      Forward();
      if (LineFollowFlag<20) LineFollowFlag++;
      break;

    case S1_OUT_S2_OUT:
      if(LineFollowFlag==10) Backward();
      if(LineFollowFlag<10) TurnLeft1();
      if(LineFollowFlag>10) TurnRight1();
      break;
  }
}

uint8_t buf[64];
uint8_t bufindex;

/**
 * \par Function
 *    read_serial
 * \par Description
 *    The function used to process serial data.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Is there a valid command 
 * \par Others
 *    None
 */
boolean read_serial(void)
{
  boolean result = false;
  readSerial();
  if(isAvailable)
  {
    uint8_t c = serialRead & 0xff;
    result = true;
    if ((c == 0x55) && (!isStart))
    {
      if(prevc == 0xff)
      {
        index=1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if(isStart)
      {
        if(index == 2)
        {
          dataLen = c; 
        }
        else if(index > 2)
        {
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
    index++;
    if(index > 51)
    {
      index=0; 
      isStart=false;
    }
    if(isStart && (dataLen == 0) && (index > 3))
    { 
      isStart = false;
      parseData(); 
      index=0;
    }
    return result;
  }
}
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  while(!Serial){}
  while(!Serial2){}
  while(!Serial3){}
  delay(5);
  for(int i=0;i<4;i++)
  {
    encoders[i].reset(i+1);
  }
  attachInterrupt(encoders[0].getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(encoders[1].getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(encoders[2].getIntNum(), isr_process_encoder3, RISING);
  attachInterrupt(encoders[3].getIntNum(), isr_process_encoder4, RISING);
  delay(5);
  pinMode(13,OUTPUT);

  //Set Pwm 970Hz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
  TCCR3A = _BV(WGM30);
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
  TCCR4A = _BV(WGM40);
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);

  for(int i=0;i<4;i++)
  {
    encoders[i].setPulse(8);
    encoders[i].setRatio(46.67);
    encoders[i].setPosPid(1.8,0,1.2);
    encoders[i].setSpeedPid(0.18,0,0);
    encoders[i].setMotionMode(DIRECT_MODE);
  }

  leftflag=false;
  rightflag=false;
  readEEPROM();
  Serial.print("Version: ");
  Serial.println(mVersion);
  blink_time = millis();
  BluetoothSource = DATA_SERIAL;
}

/**
 * \par Function
 *    loop
 * \par Description
 *    main function for arduino
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void loop()
{
  currentTime = millis()/1000.0-lastTime;

  if(millis() - blink_time > 1000)
  {
    blink_time = millis();
    blink_flag = !blink_flag;
    digitalWrite(13,blink_flag);
  }

  for(int i=0;i<4;i++)
  {
    steppers[i].update();
    encoders[i].loop();
  }

  readSerial();
  while(isAvailable)
  {
    unsigned char c = serialRead & 0xff;
    if ((c == 0x55) && (!isStart))
    {
      if(prevc == 0xff)
      {
        index=1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if(isStart)
      {
        if(index == 2)
        {
          dataLen = c; 
        }
        else if(index > 2)
        {
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
    index++;
    if(index > 51)
    {
      index=0; 
      isStart=false;
    }
    if(isStart && (dataLen == 0) && (index > 3))
    { 
      isStart = false;
      parseData(); 
      index=0;
    }
    readSerial();
  }

  if(megapi_mode == AUTOMATIC_OBSTACLE_AVOIDANCE_MODE)
  { 
    ultrCarProcess();
  }
  else if(megapi_mode == LINE_FOLLOW_MODE)
  {
    line_model();
  }
}
