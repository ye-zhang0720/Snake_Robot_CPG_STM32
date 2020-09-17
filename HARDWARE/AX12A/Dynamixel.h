//
//  Dynamixel.h
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/1.
//  Copyright © 2020 西北偏北. All rights reserved.
//
//PD8接口

#ifndef Dynamixel_h
#define Dynamixel_h

#include "sys.h"
#include <stdbool.h>
#include <stdio.h>

#define REC_BUFFER_LEN 32
#define SERVO_MAX_PARAMS (REC_BUFFER_LEN - 5)

#define REC_WAIT_START_US    75
#define REC_WAIT_PARAMS_US   (SERVO_MAX_PARAMS * 5)
#define REC_WAIT_MAX_RETRIES 200

#define SERVO_INSTRUCTION_ERROR   (1 << 6)		//指令错误      标志位
#define SERVO_OVERLOAD_ERROR      (1 << 5)		//过载错误      标志位
#define SERVO_CHECKSUM_ERROR      (1 << 4)		//校验码错误    标志位
#define SERVO_RANGE_ERROR         (1 << 3)		//运动边界错误  标志位
#define SERVO_OVERHEAT_ERROR      (1 << 2)		//过热错误      标志位
#define SERVO_ANGLE_LIMIT_ERROR   (1 << 1)		//角度限制错误  标志位
#define SERVO_INPUT_VOLTAGE_ERROR (1)			//输入电压错误  标志位

extern uint8_t servoErrorCode;

void ServoUSART_Init (u32 bound);
bool getAndCheckResponse (const uint8_t servoId);
//------------------------------------------------------------------------------
// All functions return true on success and false on failure

// ping a servo, returns true if we get back the expected values
bool pingServo (const uint8_t servoId);

// set the number of microseconds the servo waits before returning a response
// servo factory default value is 500, but we probably want it to be 0
// max value: 510
bool setServoReturnDelayMicros (const uint8_t servoId,
                                const uint16_t micros);

// set the errors that will cause the servo to blink its LED
bool setServoBlinkConditions (const uint8_t servoId,
                              const uint8_t errorFlags);

// set the errors that will cause the servo to shut off torque
bool setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t errorFlags);


// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue);

bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue);

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed (const uint8_t servoId,
                       const uint16_t speedValue);

bool getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue);

bool getServoCurrentVelocity (const uint8_t servoId,
                              int16_t *velocityValue);

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (const uint8_t servoId,
                    const float angle);

bool getServoAngle (const uint8_t servoId,
                    float *angle);




//------------------------------------------------------------------------------
// these shouldn't need to be called externally:

typedef struct ServoResponse
{
    uint8_t id;
    uint8_t length;
    uint8_t error;
    uint8_t params[SERVO_MAX_PARAMS];
    uint8_t checksum;
} ServoResponse;

typedef enum ServoCommand
{
    PING = 1,
    READ = 2,
    WRITE = 3
} ServoCommand;




void sendServoByte (uint16_t sendbyte);

void clearServoReceiveBuffer (void);

size_t  getServoBytesAvailable (void);
uint8_t getServoByte (void);

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params);
					   
bool getServoResponse (void);
void error(void);	

#endif /* Dynamixel_h */
