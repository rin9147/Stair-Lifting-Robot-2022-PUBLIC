/***************************************************************
 * Arduino GYEMS RMD-L-70 Servo Motor Library - Version 1.0
 * by rin9147 forked from Rasheed Kittinanthapanya
 * RS485 communication between GYEMS servo and M5Stack(Arduino)
 ***************************************************************/

#include "Arduino.h"
#include "GYEMS.h"

// GYMES
//------------------------------------------------------------------//
GYEMS::GYEMS(byte ID, byte RX, byte TX, byte EN)
{
  _ID = ID; // simply convert int to hex
  _EN = EN;
  pinMode(_EN, OUTPUT); // set enable pin for RS485 module as pin 2 of Arduino
  Serial1.begin(115200, SERIAL_8N1, RX, TX);
  digitalWrite(_EN, LOW);
}

// Convert int16_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int16ToByteData(int16_t Data, byte StoreByte[2])
{
  // unsigned int can store 16 bit int
  StoreByte[0] = byte((Data & 0xFF00) >> 8);
  StoreByte[1] = byte((Data & 0x00FF));
}

// Convert int32_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int32ToByteData(int32_t Data, byte StoreByte[4])
{
  // unsigned long can store 32 bit int
  StoreByte[0] = byte((Data & 0xFF000000) >> 24);
  StoreByte[1] = byte((Data & 0x00FF0000) >> 16);
  StoreByte[2] = byte((Data & 0x0000FF00) >> 8);
  StoreByte[3] = byte((Data & 0x000000FF));
}

// Convert int62_t to Byte
//------------------------------------------------------------------//
void GYEMS::Int64ToByteData(int64_t Data, byte StoreByte[8])
{
  // unsigned long long can store 64 bit int
  StoreByte[0] = byte((Data & 0xFF00000000000000) >> 56);
  StoreByte[1] = byte((Data & 0x00FF000000000000) >> 48);
  StoreByte[2] = byte((Data & 0x0000FF0000000000) >> 40);
  StoreByte[3] = byte((Data & 0x000000FF00000000) >> 32);
  StoreByte[4] = byte((Data & 0x00000000FF000000) >> 24);
  StoreByte[5] = byte((Data & 0x0000000000FF0000) >> 16);
  StoreByte[6] = byte((Data & 0x000000000000FF00) >> 8);
  StoreByte[7] = byte((Data & 0x00000000000000FF));
}

// read Driver respond(13bit)
//------------------------------------------------------------------//
void GYEMS::ReadReply13bit(byte commandByte, int16_t replyData[4])
{
  delayMicroseconds(800); // ReadReplay needs delayMicroseconds(800). DO NOT REMOVE AND CHANGE IT.
  byte FrameCheckSumReply = Header + commandByte + 0x07 + _ID;
  byte EncoderReply[13] = {0};
  int EncoderReplySize = sizeof(EncoderReply);
  while (Serial1.available() > 0)
  {
    Serial1.readBytes(EncoderReply, EncoderReplySize);
    if (FrameCheckSumReply == EncoderReply[0] + EncoderReply[1] + EncoderReply[2] + EncoderReply[3])
    {
      replyData[0] = (int8_t)EncoderReply[5];
      replyData[1] = (int16_t)(EncoderReply[7] << 8) | (EncoderReply[6]);
      replyData[2] = (int16_t)(EncoderReply[9] << 8) | (EncoderReply[8]);
      replyData[4] = (int16_t)(EncoderReply[11] << 8) | (EncoderReply[10]);
    }
  }
  delayMicroseconds(800); // ReadReplay needs delayMicroseconds(800). DO NOT REMOVE AND CHANGE IT.
}

// read Driver respond(14bit)
//------------------------------------------------------------------//
void GYEMS::ReadReply14bit(byte commandByte, int64_t replyData[1])
{
  delayMicroseconds(800); // ReadReplay needs delayMicroseconds(800). DO NOT REMOVE AND CHANGE IT.
  byte FrameCheckSumReply = Header + commandByte + 0x07 + _ID;
  byte EncoderReply[14] = {0};
  int EncoderReplySize = sizeof(EncoderReply);
  while (Serial1.available() > 0)
  {
    Serial1.readBytes(EncoderReply, EncoderReplySize);
    if (FrameCheckSumReply == EncoderReply[0] + EncoderReply[1] + EncoderReply[2] + EncoderReply[3])
    {
      replyData[0] =  (int64_t)EncoderReply[5]
                      | ((int64_t)EncoderReply[6] << 8)
                      | ((int64_t)EncoderReply[7] << 16)
                      | ((int64_t)EncoderReply[8] << 24)
                      | ((int64_t)EncoderReply[9] << 32)
                      | ((int64_t)EncoderReply[10] << 40)
                      | ((int64_t)EncoderReply[11] << 48)
                      | ((int64_t)EncoderReply[12] << 56);
    }
  }
  delayMicroseconds(800); // ReadReplay needs delayMicroseconds(800). DO NOT REMOVE AND CHANGE IT.
}

// Read multi-loop Angle command
//------------------------------------------------------------------//
void GYEMS::ReadMuitiLoopAngle(int64_t replyData[4], bool reply_flag)
{
  byte ReadMultiLoopAngleCommand = 0x92;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + ReadMultiLoopAngleCommand + _ID + DataLength;

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(ReadMultiLoopAngleCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  delayMicroseconds(800);

  if (reply_flag == true)
  {
    ReadReply14bit(ReadMultiLoopAngleCommand, replyData);
  }
}

// Motor stop shutdown command
//------------------------------------------------------------------//
void GYEMS::MotorOff(void)
{
  byte OffCommand = 0x80;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + OffCommand + _ID + DataLength;

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(OffCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  delayMicroseconds(800);
}

// Motor stop command
//------------------------------------------------------------------//
void GYEMS::MotorStop(void)
{

  byte StopCommand = 0x81;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + StopCommand + _ID + DataLength;

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(StopCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  delayMicroseconds(800);
}

// Motor operation command
//------------------------------------------------------------------//
void GYEMS::MotorRun(void)
{
  byte RunCommand = 0x88;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + RunCommand + _ID + DataLength;

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(RunCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  delayMicroseconds(800);
}

// Write the current opsition to ROM as the motor zero command (This command is not recommended for frequent use)
//------------------------------------------------------------------//
void GYEMS::SetZero(void)
{
  byte SetZeroCommand = 0x19;
  byte DataLength = 0x00;
  byte DataCheckByte = Header + SetZeroCommand + _ID + DataLength;

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SetZeroCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  delayMicroseconds(800);
}

// Torque closed loop control command
//------------------------------------------------------------------//
void GYEMS::TorqueControl(int16_t Torque, int16_t replyData[4], bool reply_flag)
{
  // Torque is a raw value, actual torque depends on the motor spec
  byte TorqueCommand = 0xA1;
  byte DataLength = 0x02;
  // byte DataLengthReplay = 0x07;
  byte FrameCheckSum = Header + TorqueCommand + DataLength + _ID;
  // byte FrameCheckSumReply = Header + TorqueCommand + DataLengthReplay + _ID;
  byte TorqueByte[2];
  Int16ToByteData(Torque, TorqueByte);
  byte DataCheckByte = TorqueByte[1] + TorqueByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(TorqueCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(TorqueByte[1]);
  Serial1.write(TorqueByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(TorqueCommand, replyData);
  }
}

// Speed closed loop control command
//------------------------------------------------------------------//
void GYEMS::SpeedControl(int32_t DPS, int16_t replyData[4], bool reply_flag)
{
  // DPS is degree per second
  int32_t SpeedLSB = DPS * 100;
  byte SpeedCommand = 0xA2;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + SpeedCommand + DataLength + _ID;
  byte SpeedByte[4];
  Int32ToByteData((int)SpeedLSB, SpeedByte);
  byte DataCheckByte = SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SpeedCommand);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(SpeedCommand, replyData);
  }
}

// Multi position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::MultiPositionControlMode1(int64_t Deg, int64_t replyData[4], bool reply_flag)
{
  int64_t DegLSB = Deg * 100;
  byte MultiPositionCommand1 = 0xA3;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + MultiPositionCommand1 + _ID + DataLength;
  byte PositionByte[8];
  Int64ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(MultiPositionCommand1);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[7]);
  Serial1.write(PositionByte[6]);
  Serial1.write(PositionByte[5]);
  Serial1.write(PositionByte[4]);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply14bit(MultiPositionCommand1, replyData);
  }
}

// Multi position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::MultiPositionControlMode2(int64_t Deg, uint32_t DPS, int64_t replyData[4], bool reply_flag)
{
  int64_t DegLSB = Deg * 100;
  uint32_t SpeedLSB = DPS * 100;
  byte MultiPositionCommand2 = 0xA4;
  byte DataLength = 0x0C;
  byte FrameCheckSum = Header + MultiPositionCommand2 + _ID + DataLength;
  byte PositionByte[8];
  byte SpeedByte[4];
  Int32ToByteData(SpeedLSB, SpeedByte);
  Int64ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(MultiPositionCommand2);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[7]);
  Serial1.write(PositionByte[6]);
  Serial1.write(PositionByte[5]);
  Serial1.write(PositionByte[4]);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply14bit(MultiPositionCommand2, replyData);
  }
}

// Single position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::SinglePositionControlMode1(uint16_t Deg, int8_t Direction, int16_t replyData[4], bool reply_flag)
{
  uint16_t DegLSB = Deg * 100;
  byte SinglePositionCommand1 = 0xA5;
  byte DataLength = 0x04;
  byte FrameCheckSum = Header + SinglePositionCommand1 + _ID + DataLength;
  byte PositionByte[4];
  Int32ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SinglePositionCommand1);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(Direction);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(SinglePositionCommand1, replyData);
  }
}

// Single position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::SinglePositionControlMode2(uint16_t Deg, uint32_t DPS, uint8_t Direction, int16_t replyData[4], bool reply_flag)
{
  uint16_t DegLSB = Deg * 100;
  uint32_t SpeedLSB = DPS * 100;
  byte SinglePositionCommand2 = 0xA6;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + SinglePositionCommand2 + _ID + DataLength;
  byte PositionByte[4];
  byte SpeedByte[4];
  Int32ToByteData(DegLSB, PositionByte);
  Int32ToByteData(SpeedLSB, SpeedByte);
  byte DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(SinglePositionCommand2);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(Direction);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(SinglePositionCommand2, replyData);
  }
}

// Multi position closed loop control command 1
//------------------------------------------------------------------//
void GYEMS::IncrementalControlMode1(int32_t Deg, uint32_t DPS, int16_t replyData[4], bool reply_flag)
{
  int32_t DegLSB = Deg * 100;
  uint32_t SpeedLSB = DPS * 100;
  byte IncrementalCommand1 = 0xA7;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + IncrementalCommand1 + _ID + DataLength;
  byte PositionByte[4];
  byte SpeedByte[4];

  Int32ToByteData(SpeedLSB, SpeedByte);
  Int32ToByteData(DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(IncrementalCommand1);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(IncrementalCommand1, replyData);
  }
}

// Multi position closed loop control command 2
//------------------------------------------------------------------//
void GYEMS::IncrementalControlMode2(int32_t Deg, uint32_t DPS, int16_t replyData[4], bool reply_flag)
{
  int32_t DegLSB = Deg * 100;
  uint32_t SpeedLSB = DPS * 100;
  byte IncrementalCommand2 = 0xA8;
  byte DataLength = 0x08;
  byte FrameCheckSum = Header + IncrementalCommand2 + _ID + DataLength;
  byte PositionByte[4];
  byte SpeedByte[4];

  Int32ToByteData((int32_t)SpeedLSB, SpeedByte);
  Int32ToByteData((int32_t)DegLSB, PositionByte);
  byte DataCheckByte = PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

  digitalWrite(_EN, HIGH);
  delayMicroseconds(50);
  Serial1.write(Header);
  Serial1.write(IncrementalCommand2);
  Serial1.write(_ID);
  Serial1.write(DataLength);
  Serial1.write(FrameCheckSum);
  Serial1.write(PositionByte[3]);
  Serial1.write(PositionByte[2]);
  Serial1.write(PositionByte[1]);
  Serial1.write(PositionByte[0]);
  Serial1.write(SpeedByte[3]);
  Serial1.write(SpeedByte[2]);
  Serial1.write(SpeedByte[1]);
  Serial1.write(SpeedByte[0]);
  Serial1.write(DataCheckByte);
  Serial1.flush();
  digitalWrite(_EN, LOW);

  if (reply_flag == true)
  {
    ReadReply13bit(IncrementalCommand2, replyData);
  }
}
