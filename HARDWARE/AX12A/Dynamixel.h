//
//  Dynamixel.h
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/1.
//  Copyright © 2020 西北偏北. All rights reserved.
//

#ifndef Dynamixel_h
#define Dynamixel_h

#include <stdio.h>
#include "DynamixelConfig.h"
#include "usart.h"



bool setCWComplianceMargin(const byte  ID, const byte margin);
bool readCWComplianceMargin(const byte ID);

bool setCCWComplianceMargin(const byte  ID, const byte margin);
bool readCCWComplianceMargin(const byte ID);

bool setCWComplianceSlope(const byte  ID, const byte slope);
bool readCWComplianceSlope(const byte ID);

bool setCCWComplianceSlope(const byte  ID, const byte slope);
bool readCCWComplianceSlope(const byte ID);



// This is the only function to write on the serial port
bool writeRaw(byte *sentence, const byte nByteToBeWritten);
unsigned short readDxlData(void);               // the onvoidly function to read on seria port

void begin(const unsigned long speed);

bool ping(const byte  ID);    // ping the servo
bool action(const byte  ID);  // Excecute action written in REG_WRITE servo register. A status packet is receive, although the documentation don't mention it
bool reset(const byte  ID);   // Reset the servo to factory default setting
bool rebootMotor(const byte  ID);
//
//  EEPROM commands
//

bool readModelNumber(const byte ID);
bool readFirmware(const byte ID);

bool setId(const byte  ID, const byte newID);
bool readId(const byte ID);    // Is this really usefull ?

bool setBaudRate(const byte  ID, const byte baudRate);
bool readBaudRate(const byte ID);

bool setReturnDelayTime(const byte  ID, const byte returnDelayTime);
bool readReturnDelayTime(const byte ID);

bool setCWAngleLimit(const byte  ID, const unsigned short angle);
bool setCCWAngleLimit(const byte  ID, const unsigned short angle);

bool readCWAngleLimit(const byte ID);
bool readCCWAngleLimit(const byte ID);

bool setMaxTemperature(const byte  ID, const byte maxTemperature);
bool readMaxTemperature(const byte ID);

bool setMinVoltage(const byte  ID, const byte minVoltage);
bool readMinVoltage(const byte ID);

bool setMaxVoltage(const byte  ID, const byte maxVoltage);
bool readMaxVoltage(const byte ID);

bool setMaxTorque(const byte  ID, const unsigned short maxTorque);
bool readMaxTorque(const byte ID);

bool setStatusReturnLevel(const byte  ID, const byte Status);
bool readStatusReturnLevel(const byte ID);

bool setAlarmLed(const byte  ID, const byte Status);
bool readAlarmLed(const byte ID);

bool setAlarmShutdown(const byte  ID, const byte Status);
bool readAlarmShutdown(const byte ID);

//
//  RAM commands
//
bool setTorqueEnable(const byte  ID, const bool Status);
bool readTorqueEnable(const byte ID);

bool setLedEnable(const byte  ID, const bool Status);
bool readLedEnable(const byte  ID);

bool setGoalPosition(const byte ID, const short Position);
bool setGoalPositionAtSpeed(const byte ID, const short Position, const short Speed);
bool setMovingSpeed(const byte ID, const short Speed);
bool setTorqueLimit(const byte ID, const short torque);

bool readPresentPosition(const byte ID);
bool readPresentSpeed(const byte ID);
bool readPresentLoad(const byte ID);

bool readVoltage(const byte ID);
bool readTemperature(const byte ID);
bool isRegistered(const byte ID);
bool isMoving(const byte ID);

bool setEEPROMLock(const byte ID, const bool lock);
bool isEEPROMLock(const byte ID);

bool setPunch(const byte  ID, const unsigned short current);
bool readPunch(const byte ID);

// Low level public methods

bool sendDxlCommand(const byte ID, const byte dxlCommand);
bool sendDxlWrite(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten);
bool sendDxlRegData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten);
bool sendDxlRead(const byte ID, const byte dxlAddress, const byte nByteToBeRead);
bool dxlDataReady(void);
unsigned short readDxlError(void);                      // return the error or DXL_ERR_SUCCESS of the last operation
unsigned short readDxlResult(void);                     // return the value read of the last operation



void setDxlReturnDelayTime(const unsigned long returnDelayTime);
unsigned long readDxlReturnDelayTime(void);

#endif /* Dynamixel_h */
