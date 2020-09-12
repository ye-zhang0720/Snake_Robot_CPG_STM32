//
//  Dynamixel.c
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/1.
//  Copyright © 2020 西北偏北. All rights reserved.
//

#include "Dynamixel.h"


unsigned short  _error;                // Store the last error
byte _nByteToBeRead;                   // Used to check when Rx packet is complete
byte _currentId;                       // Used to check if the rx packet comes from the correct servo

unsigned long _currentTimeout;         // To be compared with millis();
unsigned long _returnDelayTime;        // Store the return time of Dynamixel servo (dfault 1000 µs)
unsigned long _currentReturnDelayTime; // this variable assume that dxlDataReady();is called periodically
unsigned short _dxlResult;             // Dxl read will be stored here


//
// Dxl Commands
//
bool ping(const byte  ID)
{ return (sendDxlCommand(ID, DXL_PING)); }

bool action(const byte  ID)
{ return (sendDxlCommand(ID, DXL_ACTION)); }

bool reset(const byte  ID)
{ return (sendDxlCommand(ID, DXL_RESET)); }

bool rebootMotor(const byte  ID)
{ return (sendDxlCommand(ID, DXL_REBOOT)); }


//
//  RAM commands
//
bool setCWComplianceMargin(const byte  ID, const byte margin)
{ return sendDxlWrite(ID, DXL_ADD_CW_COMPLIANCE_MARGIN , &margin, 1 );}
bool readCWComplianceMargin(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CW_COMPLIANCE_MARGIN , 1); }

bool setCCWComplianceMargin(const byte  ID, const byte margin)
{ return sendDxlWrite(ID, DXL_ADD_CCW_COMPLIANCE_MARGIN , &margin, 1 );}
bool readCCWComplianceMargin(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CCW_COMPLIANCE_MARGIN , 1); }

bool setCWComplianceSlope(const byte  ID, const byte slope)
{ return sendDxlWrite(ID, DXL_ADD_CW_COMPLIANCE_SLOPE , &slope, 1 );}
bool readCWComplianceSlope(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CW_COMPLIANCE_SLOPE , 1); }

bool setCCWComplianceSlope(const byte  ID, const byte slope)
{ return sendDxlWrite(ID, DXL_ADD_CCW_COMPLIANCE_SLOPE , &slope, 1 );}
bool readCCWComplianceSlope(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CCW_COMPLIANCE_SLOPE , 1); }


//
//  EEPROM commands
//

bool readModelNumber(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MODEL_NUMBER , 2); }

bool readFirmware(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_FIRMWARE , 1); }

bool setId(const byte  ID, const byte newID)
{ return sendDxlWrite(ID, DXL_ADD_ID, &newID, 1 );}
bool readId(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_ID , 1); }

bool setBaudRate(const byte  ID, const byte baudRate)
{  return sendDxlWrite(ID, DXL_ADD_BAUDRATE, &baudRate, 1 );}
bool readBaudRate(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_BAUDRATE , 1); }

bool setReturnDelayTime(const byte  ID, const byte returnDelayTime)
{  return sendDxlWrite(ID, DXL_ADD_RETURN_DELAY_TIME , &returnDelayTime, 1 );}
bool readReturnDelayTime(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_RETURN_DELAY_TIME , 1); }

bool setCWAngleLimit(const byte  ID, const unsigned short angle)
{  return sendDxlWrite(ID, DXL_ADD_CW_ANGLE_LIMIT ,(const byte*) &angle, 2 );}
bool setCCWAngleLimit(const byte  ID, const unsigned short angle)
{  return sendDxlWrite(ID, DXL_ADD_CCW_ANGLE_LIMIT , (const byte*) &angle, 2 );}

bool readCWAngleLimit(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CW_ANGLE_LIMIT , 2); }
bool readCCWAngleLimit(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_CCW_ANGLE_LIMIT , 2); }

bool setMaxTemperature(const byte  ID, const byte maxTemperature)
{ return sendDxlWrite(ID, DXL_ADD_MAX_TEMPERATURE , &maxTemperature, 1 );}
bool readMaxTemperature(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MAX_TEMPERATURE , 1); }

bool setMinVoltage(const byte  ID, const byte minVoltage)
{ return sendDxlWrite(ID, DXL_ADD_MIN_VOLTAGE , &minVoltage, 1 );}
bool readMinVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MIN_VOLTAGE , 1); }

bool setMaxVoltage(const byte  ID, const byte maxVoltage)
{ return sendDxlWrite(ID, DXL_ADD_MAX_VOLTAGE , &maxVoltage, 1 );}
bool readMaxVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MAX_VOLTAGE , 1); }

bool setMaxTorque(const byte  ID, const unsigned short maxTorque)
{ return sendDxlWrite(ID, DXL_ADD_MAX_TORQUE ,(const byte*) &maxTorque, 2 );}
bool readMaxTorque(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MAX_TORQUE , 2); }

bool setStatusReturnLevel(const byte  ID, const byte Status)
{ return sendDxlWrite(ID, DXL_ADD_STATUS_RETURN_LEVEL , &Status, 1 );}
bool readStatusReturnLevel(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_STATUS_RETURN_LEVEL , 1); }

bool setAlarmLed(const byte  ID, const byte Status)
{ return sendDxlWrite(ID, DXL_ADD_ALARM_LED , &Status, 1 );}
bool readAlarmLed(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_ALARM_LED , 1); }

bool setAlarmShutdown(const byte  ID, const byte Status)
{ return sendDxlWrite(ID, DXL_ADD_ALARM_SHUTDOWN , &Status, 1 );}
bool readAlarmShutdown(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_ALARM_SHUTDOWN , 1); }


//
//  RAM commands
//
bool setTorqueEnable(const byte  ID, const bool Status)
{ return sendDxlWrite(ID, DXL_ADD_TORQUE_ENABLE, (const byte*) &Status, 1 );}
bool readTorqueEnable(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_TORQUE_ENABLE , 1); }

bool setLedEnable(const byte  ID, const bool Status)
{ return sendDxlWrite(ID, DXL_ADD_LED, (const byte*) &Status, 1 );}
bool readLedEnable(const byte  ID )
{ return sendDxlRead(ID, DXL_ADD_LED , 1); }

bool setGoalPosition(const byte ID, const short Position)
{ return sendDxlWrite(ID, DXL_ADD_GOAL_POSITION, (const byte*) &Position, 2 );}

bool setGoalPositionAtSpeed(const byte ID, const short Position, const short Speed)
{
    const short params[] = { Position, Speed };
    return sendDxlWrite(ID, DXL_ADD_GOAL_POSITION, (const byte*) params, 4 );
}

bool setMovingSpeed(const byte ID, const short Speed)
{ return sendDxlWrite(ID, DXL_ADD_MOVING_SPEED, (const byte*) &Speed, 2 );}

bool setTorqueLimit(const byte ID, const short torque)
{ return sendDxlWrite(ID, DXL_ADD_TORQUE_LIMIT, (const byte*) &torque, 2 );}

bool readPresentPosition(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_POSITION , 2); }

bool readPresentSpeed(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_SPEED , 2); }

bool readPresentLoad(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_LOAD , 2); }

bool  readVoltage(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_VOLTAGE , 1); }

bool readTemperature(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PRESENT_TEMPERATURE, 1); }

bool isRegistered(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_REGISTERED , 1); }

bool isMoving(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_MOVING , 1); }

bool setEEPROMLock(const byte ID, const bool lock)
{ return sendDxlWrite(ID, DXL_ADD_LOCK, (const byte*) &lock, 1 );}
bool isEEPROMLock(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_LOCK , 1); }

bool setPunch(const byte  ID, const unsigned short current)
{ return sendDxlWrite(ID, DXL_ADD_PUNCH ,(const byte*) &current, 2 );}
bool readPunch(const byte ID)
{ return sendDxlRead(ID, DXL_ADD_PUNCH , 2); }


void setDxlReturnDelayTime(const unsigned long returnDelayTime)
{ _returnDelayTime = returnDelayTime; }
unsigned long readDxlReturnDelayTime()
{ return _returnDelayTime; }



//
// Low Level Public Methods
//

bool sendDxlCommand(const byte ID, const byte dxlCommand)
{
    byte sentence[6];
    byte Checksum = (~(ID + 0x02 + dxlCommand )) & 0xFF;
    
    sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
    sentence[2] = ID;                       // Servo ID
    sentence[3] = 0x02;                     // length
    sentence[4] = dxlCommand;               // dxl instruction
    sentence[5] = Checksum;                 // Checksum
    
    if(writeRaw(sentence, 6))
    {
        _nByteToBeRead = 6;
        _currentId = ID;
        setnByteToBeRead(_nByteToBeRead);
        return 1;
    }
    
    _nByteToBeRead = 0;
    _currentId = 0;
    return 0;
    
}


bool sendDxlRegData(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten )
{
    byte i;
    const byte txDataLength =  nByteToBeWritten + 3 ;    // length is "number of parameters N +2"
    byte sentence[7 + nByteToBeWritten];
    byte Checksum = 0;
    
    for (i = 0; i < nByteToBeWritten; i++)
        Checksum += params[i];
    
    Checksum = (~(ID + txDataLength  + DXL_REG_WRITE + dxlAddress + Checksum )) & 0xFF;
    
    sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
    sentence[2] = ID;                       // Servo ID
    sentence[3] = txDataLength;             // length
    sentence[4] = DXL_REG_WRITE;           // write instruction
    sentence[5] = dxlAddress;               // adress to start
    
    for (i = 0; i < nByteToBeWritten; i++)  // write params ( first one is address)
        sentence[i+6] = params[i];
    
    sentence[i+6] = Checksum;
    
    if(writeRaw(sentence, txDataLength + 4))
    {
        _nByteToBeRead = 6;
        _currentId = ID;
        setnByteToBeRead(_nByteToBeRead);
        return 1;
    }
    
    _nByteToBeRead = 0;
    _currentId = 0;
    return 0;
    
}

bool sendDxlWrite(const byte ID, const byte dxlAddress, const byte *params, const byte nByteToBeWritten)
{
    byte i;
    const byte txDataLength =  nByteToBeWritten + 3 ;    // length is "number of parameters N +2"
    byte sentence[7 + nByteToBeWritten];
    byte Checksum = 0;
    
    for (i = 0; i < nByteToBeWritten; i++)
        Checksum += params[i];
    
    Checksum = (~(ID + txDataLength  + DXL_WRITE_DATA + dxlAddress + Checksum )) & 0xFF;
    
    sentence[0] = sentence[1] = DXL_START;  // 2 start bytes
    sentence[2] = ID;                       // Servo ID
    sentence[3] = txDataLength;             // length
    sentence[4] = DXL_WRITE_DATA;           // write instruction
    sentence[5] = dxlAddress;               // adress to start
    
    for (i = 0; i < nByteToBeWritten; i++)  // write params ( first one is address)
        sentence[i+6] = params[i];
    
    sentence[i+6] = Checksum;
    
    if(writeRaw(sentence, txDataLength + 4))
    {
        _nByteToBeRead = 6;
        setnByteToBeRead(_nByteToBeRead);
        _currentId = ID;
        return 1;
    }
    
    _nByteToBeRead = 0;
    _currentId = 0;
    return 0;
    
}

bool sendDxlRead(const byte ID, const byte dxlAddress, const byte nByteToBeRead)
{
    
    byte Checksum = (~(ID +  4  + DXL_READ_DATA + dxlAddress + nByteToBeRead)) & 0xFF;
    byte sentence[] ={ DXL_START, DXL_START, ID, 0x04, DXL_READ_DATA, dxlAddress, nByteToBeRead, Checksum};
    
    if(writeRaw(sentence, 8))
    {
        _nByteToBeRead = 6 + nByteToBeRead;
        _currentId = ID;
        setnByteToBeRead(_nByteToBeRead);
        return 1;
    }
    
    _nByteToBeRead = 0;
    _currentId = 0;
    return 0;
    
}

bool dxlDataReady()
{
    if(_nByteToBeRead == 0)
        return 0;
    
    if(available()>=_nByteToBeRead)
    {
        if(_currentReturnDelayTime == 0)
            //_currentReturnDelayTime = _returnDelayTime + micros();
            
            return 1;
    }
    
    
    return 0;
}

unsigned short readDxlResult()
{
    if(available() == 0)
        return 0xffff;
    else
        return readDxlData();
}

unsigned short readDxlError()
{
    if(available() != 0)
       return readDxlData();
    
    return _error;
}



//
// Private Methods
//

// The only function to write on serial port
bool writeRaw(byte *sentence, const byte nByteToBeWritten)
{
//    if(!upDirectionPort())
//        return 0;
    
    _error = DXL_ERR_SUCCESS;  // purge error
    _dxlResult = 0;            // purge result
    
    if(USART1_Send_Data(sentence, nByteToBeWritten) == nByteToBeWritten)
    {
        _currentReturnDelayTime = 0;
//        if(!downDirectionPort())
//            return 0;
        return 1;
    }
    else
    {
        _currentTimeout = 0;
        return 0;
    }
}

// The only function to read on serial port
unsigned short readDxlData()
{

    byte check = 0;             // used to compute checksum
    byte header[5];             // 2 start byte + ID + length + Error
    byte nDataBytes = _nByteToBeRead - 6;
    byte result[nDataBytes];    // where the data will be saved
    byte Checksum = 0;
		
    unsigned char * _data;    //接收到的数据
    
    _data = USART1_read_Data();
    // Read the incoming bytes
    header[0] = _data[0];   // 2 Starts Bytes
    header[1] = _data[1];

    header[2] = _data[2];   // Servo ID
    header[3] = _data[3];   // Length
    header[4] = _data[4];   // Error

    for (byte i = 0; i < nDataBytes; i++)
    {
        result[i] = _data[5];
        check +=  result[i];
    }
		if(nDataBytes == 0)
			Checksum  = _data[5];   // Checksum

    Checksum  = _data[6];   // Checksum
    
    if (!( ( header[0] == DXL_START) & ( header[1]  == DXL_START) ))
        _error |=  DXL_ERR_RX_FAIL;     // No header found
    
    if (header[2]!=_currentId)
        _error |=DXL_ERR_ID;            // The response comes from another Servo !
    
    if (header[3]!= nDataBytes + 2)
    {
        nDataBytes = header[3] - 2;
        _error |=DXL_ERR_RX_LENGTH;             // Servo send an error we push it. To catch it error mask should be used
    }
    
    if (header[4]!=0)
        _error |=header[4];             // Servo send an error we push it. To catch it error mask should be used
    
    
    if (Checksum != ((~(header[2] + header[3]  + header[4] + check )) & 0xFF ))
        _error |=DXL_ERR_RX_CORRUPT;    // Checksum error
    
    if(nDataBytes == 0)
        _dxlResult = 0;
    else if(nDataBytes == 1)
        _dxlResult = (unsigned short) result[0];
    else if(nDataBytes == 2)
        _dxlResult = (result[1] << 8) | result[0];   // (msb << 8) | lsb
    
		
    _nByteToBeRead = 0;
    _currentTimeout = 0;
    _currentId = 0;
    
		//USART1_Send_Data(&_dxlResult, 1);
		
    return  _dxlResult ;               // Returns the error, if OK, DXL_ERR_SUCCESS
}
