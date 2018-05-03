#include  "serializer.hpp"
#include  <iostream>

#define _USE_MATH_DEFINES
#include  <cmath>

RobotCommand::RobotCommand()
{
}

RobotCommand::RobotCommand(unsigned int id, float vel_x, float vel_y, float omega,
        float dribble_power,
        float kick_power, RobotCommand::KickType kick_type) :
        _id(id), _vel_x(vel_x), _vel_y(vel_y), _omega(omega),
        _dribble_power(dribble_power),
        _kick_power(kick_power), _kick_type(kick_type)
{}


/* Scramble Protocol
 * 0: 0101 0011 |HEADER
 * 1: 0000 xxxx |x:robot ID
 * 2: d00e f000 |d:dribble_flag, e:kick_flag, f:chip_enable
 * 3: gggg hhhh |g:dribble_power, h:kick_power
 * 4-7: vel_x(float)
 * 8-11: vel_y(float)
 * 12-15: omega(float)
 * 16: pppp pppp  | parity check (xor data 1 to 15)
 * 17: pppp pppp  | parity check (xor data 16 and 0xff)
 */

const char ScrambleSerializer::HEADER = 0x53;
const int ScrambleSerializer::NUM_DATA = 18;

bool  ScrambleSerializer::serialize(RobotCommand cmd, char* data)
{
    RobotCommand_Binary cmd_bin = scalingToBinary(cmd);

    data[0] = HEADER;

    data[1] = cmd_bin._id;

    data[2] = 0x00;
    // kick_flag
    if (cmd_bin._kick_power > 0) {
        data[2] |= 0x10;
    }
    // chip_enable
    if (cmd_bin._kick_type == RobotCommand_Binary::CHIP) {
        data[2] |= 0x08;
    }

    // TODO : Overflow err expression
    data[3] = 0x00;
    data[3] += cmd_bin._dribble_power;
    data[3] <<= 4;
    data[3] += cmd_bin._kick_power;

    *((float *)&(data[4])) = cmd._vel_x;
    *((float *)&(data[8])) = cmd._vel_y;
    *((float *)&(data[12])) = cmd._omega;

    

    // Make checksum
    data[16] = 0x00;
    for (size_t i=1; i<16; i++) {
        data[16] ^= data[i];
    }
    data[17] = data[16] ^ 0xFF;

    return  true;
}


// convert MKS unit to binary data in order to send as packet
ScrambleSerializer::RobotCommand_Binary ScrambleSerializer::scalingToBinary(RobotCommand robot_command)
{
  RobotCommand_Binary command_binary;

  // TODO:copy instances
  command_binary._id = robot_command._id;

  // Dribble power
  if (robot_command._dribble_power > 15) {
    command_binary._dribble_power = 15;
  } else if (robot_command._dribble_power < 0) {
    command_binary._dribble_power = 0;
  } else {
    command_binary._dribble_power = robot_command._dribble_power;
  }

  // Kick power
  if (robot_command._kick_power > 15) {
    command_binary._kick_power = 15;
  }else if (robot_command._kick_power < 0) {
    command_binary._kick_power = 0;
  } else {
    command_binary._kick_power = robot_command._kick_power;
  }

  // Kick Type
  if (robot_command._kick_type == RobotCommand::CHIP) {
    command_binary._kick_type = RobotCommand_Binary::CHIP;
  } else {
    command_binary._kick_type = RobotCommand_Binary::STRAIGHT;
  }

  return  command_binary;
};


float ScrambleSerializer::piTopi(float angle)
{
  while (angle >=  M_PI) angle -= 2*M_PI;
  while (angle < -M_PI) angle += 2*M_PI;
  return  angle;
}

float ScrambleSerializer::zeroTo2pi(float angle)
{
  while (angle >= 2*M_PI) angle -= 2*M_PI;
  while (angle < 0)      angle += 2*M_PI;
  return angle;
}
