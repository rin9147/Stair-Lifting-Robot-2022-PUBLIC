/***************************************************************
 * Arduino GYEMS RMD-L-70 Servo Motor Library - Version 1.0
 * by rin9147 forked from Rasheed Kittinanthapanya
 * RS485 communication between GYEMS servo and M5Stack(Arduino)
 ***************************************************************/

#ifndef GYEMS_H
#define GYEMS_H

#include "Arduino.h"

class GYEMS
{
public:
    GYEMS(byte ID, byte RX, byte TX, byte EN);
    void Int16ToByteData(int16_t Data, byte StoreByte[2]);
    void Int32ToByteData(int32_t Data, byte StoreByte[4]);
    void Int64ToByteData(int64_t Data, byte StoreByte[8]);
    void ReadReply13bit(byte commandByte, int16_t replyData[4]);
    void ReadReply14bit(byte commandByte, int64_t replyData[1]);
    void ReadMuitiLoopAngle(int64_t replyData[4], bool reply_flag);
    void MotorOff(void);
    void MotorStop(void);
    void MotorRun(void);
    void SetZero(void);
    void TorqueControl(int16_t Torque,int16_t replyData[4], bool reply_flag);
    void SpeedControl(int32_t DPS, int16_t replyData[4], bool reply_flag);
    void MultiPositionControlMode1(int64_t Deg, int64_t replyData[1], bool reply_flag);
    void MultiPositionControlMode2(int64_t Deg, uint32_t DPS, int64_t replyData[1], bool reply_flag);
    void SinglePositionControlMode1(uint16_t Deg, int8_t Direction, int16_t replyData[4], bool reply_flag);
    void SinglePositionControlMode2(uint16_t Deg, uint32_t DPS, uint8_t Direction, int16_t replyData[4], bool reply_flag);
    void IncrementalControlMode1(int32_t Deg, uint32_t DPS, int16_t replyData[4], bool reply_flag);
    void IncrementalControlMode2(int32_t Deg, uint32_t DPS, int16_t replyData[4], bool reply_flag);

private:
    byte Header = 0x3E;
    byte _ID;
    byte _EN;
};

#endif
