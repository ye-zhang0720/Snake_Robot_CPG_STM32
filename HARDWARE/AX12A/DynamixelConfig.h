//
//  DynamixelConfig.h
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/1.
//  Copyright © 2020 西北偏北. All rights reserved.
//

#ifndef DynamixelConfig_h
#define DynamixelConfig_h

#define byte unsigned char
#define bool _Bool



#define DXL_ERR_SUCCESS             (0x00)    // No header found during transmit

#define DXL_ERR_VOLTAGE             (0x01)    // When the applied voltage is out of the range of operating voltage set in the Control table,
#define DXL_ERR_ANGLE               (0x02)    // When Goal Position is written out of the range from CW Angle Limit to CCW Angle Limit
#define DXL_ERR_OVERHEATING         (0x04)    // When internal temperature of Dynamixel is out of the range of operating temperature set in the Control table
#define DXL_ERR_RANGE               (0x08)    // When a command is out of the range for use
#define DXL_ERR_TX_CHECKSUM         (0x10)    // When the Checksum of the transmitted Instruction Packet is incorrect
#define DXL_ERR_OVERLOAD            (0x20)    // When the current load cannot be controlled by the set Torque
#define DXL_ERR_INSTRUCTION         (0x40)    // In case of sending an undefined instruction or delivering the action command without the reg_write command

#define DXL_ERR_TX_FAIL             (0x100)   // No header found during transmit
#define DXL_ERR_RX_FAIL             (0x200)   // No header found during receive
#define DXL_ERR_TX_ERROR            (0x400)
#define DXL_ERR_RX_LENGTH           (0x800)   // The length of the response doesn't match
#define DXL_ERR_RX_TIMEOUT          (0x1000)  // Time out during receive operation
#define DXL_ERR_RX_CORRUPT          (0x2000)  // Checksum error
#define DXL_ERR_ID                  (0x4000)  // Wrong ID response after instruction packet

/******************************************************************************
* Communication protocol
******************************************************************************/

#define DXL_START                     (0xFF)
#define DXL_RETURN_DELAY_TIME         (800) // us, servo return delay time register : 2usec per data value

#define DXL_PING                      (0x01)
#define DXL_READ_DATA                 (0x02)
#define DXL_WRITE_DATA                (0x03)
#define DXL_REG_WRITE                 (0x04)
#define DXL_ACTION                    (0x05)
#define DXL_RESET                     (0x06)
#define DXL_REBOOT                    (0x08)
#define DXL_SYNC_READ                 (0x82)
#define DXL_SYNC_WRITE                (0x83)

/******************************************************************************
* Servo registers (ADDRESS)
******************************************************************************/

#define DXL_ADD_MODEL_NUMBER          (0x00)
#define DXL_ADD_FIRMWARE              (0x02)
#define DXL_ADD_ID                    (0x03)
#define DXL_ADD_BAUDRATE              (0x04)
#define DXL_ADD_RETURN_DELAY_TIME     (0x05)
#define DXL_ADD_CW_ANGLE_LIMIT        (0x06)
#define DXL_ADD_CCW_ANGLE_LIMIT       (0x08)
#define DXL_ADD_MAX_TEMPERATURE       (0x0B)
#define DXL_ADD_MIN_VOLTAGE           (0x0C)
#define DXL_ADD_MAX_VOLTAGE           (0x0D)
#define DXL_ADD_MAX_TORQUE            (0x0E)
#define DXL_ADD_STATUS_RETURN_LEVEL   (0x10)
#define DXL_ADD_ALARM_LED             (0x11)
#define DXL_ADD_ALARM_SHUTDOWN        (0x12)


#define DXL_ADD_TORQUE_ENABLE         (0x18)
#define DXL_ADD_LED                   (0x19)
#define DXL_ADD_GOAL_POSITION         (0x1E)
#define DXL_ADD_MOVING_SPEED          (0X20)
#define DXL_ADD_TORQUE_LIMIT          (0X22)
#define DXL_ADD_PRESENT_POSITION      (0X24)
#define DXL_ADD_PRESENT_SPEED         (0X26)
#define DXL_ADD_PRESENT_LOAD          (0X28)
#define DXL_ADD_PRESENT_VOLTAGE       (0X2A)
#define DXL_ADD_PRESENT_TEMPERATURE   (0x2B)
#define DXL_ADD_REGISTERED            (0X2C)
#define DXL_ADD_MOVING                (0X2E)
#define DXL_ADD_LOCK                  (0X2F)
#define DXL_ADD_PUNCH                 (0X30)

#define DXL_ADD_CW_COMPLIANCE_MARGIN  (0x1A)
#define DXL_ADD_CCW_COMPLIANCE_MARGIN (0x1B)
#define DXL_ADD_CW_COMPLIANCE_SLOPE   (0x1C)
#define DXL_ADD_CCW_COMPLIANCE_SLOPE  (0x1D)

#endif /* DynamixelConfig_h */
