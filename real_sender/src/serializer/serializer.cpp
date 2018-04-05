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
 * 0: 1111 1111 |HEADER_1 0xFF
 * 1: 1100 0011 |HEADER_2 0xC3
 * 2: 0000 xxxx |x:ID
 * 3: aaaa aaaa |a:vel_x(0~255)
 * 4: bbbb bbbb |b:vel_y(0~255)
 * 5: cccc cccc |c:omega(0~255)
 * 6: d01e f110 |d:dribble_flag, e:kick_flag, f:chip_enable
 * 7: gggg hhhh |g:dribble_power, h:kick_power
 * 8: **** **** |XOR([2] ~ [7])
 * 9: **** **** |XOR([8],0xFF)
 *
 */

const char ScrambleSerializer::HEADER_1 = 0xFF;
const char ScrambleSerializer::HEADER_2 = 0xC3;
const int ScrambleSerializer::NUM_DATA = 10;

bool  ScrambleSerializer::serialize(RobotCommand cmd, char* data)
{
    RobotCommand_Binary cmd_bin = scalingToBinary(cmd);

    data[0] = HEADER_1;
    data[1] = HEADER_2;

    data[2] = cmd_bin._id;

    data[3] = cmd_bin._vel_x;
    data[4] = cmd_bin._vel_y;
    data[5] = cmd_bin._omega;

    data[6] = 0x00;
    // dribble_flag
    if (cmd_bin._dribble_power > 0) {
        data[6] |= 0x80;
    }
    data[6] |= 0x20; // magic number for HEADER_2
    // kick_flag
    if (cmd_bin._kick_power > 0) {
        data[6] |= 0x10;
    }
    // chip_enable
    if (cmd_bin._kick_type == RobotCommand_Binary::CHIP) {
        data[6] |= 0x08;
    }
    data[6] |= 0x04; // magic number for HEADER_2
    // TODO : ChargerFlag, ErrFlag
    data[6] |= 0x02;

    // TODO : Overflow err expression
    data[7] = 0x00;
    data[7] += cmd_bin._dribble_power;
    data[7] <<= 4;
    data[7] += cmd_bin._kick_power;

    // Make checksum
    data[8] = 0x00;
    for (size_t i=2; i<8; i++) {
        data[8] ^= data[i];
    }
    data[9] = data[8] ^ 0xFF;

    return  true;
}


// convert MKS unit to binary data in order to send as packet
ScrambleSerializer::RobotCommand_Binary ScrambleSerializer::scalingToBinary(RobotCommand robot_command)
{
  RobotCommand_Binary command_binary;

  // TODO:copy instances
  command_binary._id = robot_command._id;

  // Velocity z
  if (robot_command._vel_x < 0.0) {
    command_binary._vel_x = 0;
  } else {
    command_binary._vel_x = robot_command._vel_x * 255 / 4;
    if (command_binary._vel_x > 255) {
        command_binary._vel_x = 255;
    }
  }

  // Angular velocity
  robot_command._omega = round(robot_command._omega * 128 / (2*M_PI)) + 127;
  if (robot_command._omega > 254) {
    command_binary._omega = 254;
  } else if (robot_command._omega < 0) {
    command_binary._omega = 0;
  } else {
    command_binary._omega = robot_command._omega;
  }

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
