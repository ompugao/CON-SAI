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
 * 2: aaaa aaaa |a:vel_x(0~255) neutral = 127
 * 3: bbbb bbbb |b:vel_y(0~255) neutral = 127
 * 4: cccc cccc |c:omega(0~255) neutral = 127
 * 5: d01e f110 |d:dribble_flag, e:kick_flag, f:chip_enable
 * 6: gggg hhhh |g:dribble_power, h:kick_power
 *
 */

const char ScrambleSerializer::HEADER = 0x53;
const int ScrambleSerializer::NUM_DATA = 7;
const int ScrambleSerializer::Vx_max = 5;
const int ScrambleSerializer::Vy_max = 5;

bool  ScrambleSerializer::serialize(RobotCommand cmd, char* data)
{
    RobotCommand_Binary cmd_bin = scalingToBinary(cmd);

    data[0] = HEADER;

    data[1] = cmd_bin._id;

    data[2] = cmd_bin._vel_x;
    data[3] = cmd_bin._vel_y;
    data[4] = cmd_bin._omega;

    data[5] = 0x00;
    // dribble_flag
    if (cmd_bin._dribble_power > 0) {
        data[5] |= 0x80;
    }
    data[5] |= 0x20; // magic number for HEADER_2
    // kick_flag
    if (cmd_bin._kick_power > 0) {
        data[5] |= 0x10;
    }
    // chip_enable
    if (cmd_bin._kick_type == RobotCommand_Binary::CHIP) {
        data[5] |= 0x08;
    }
    data[5] |= 0x04; // magic number for HEADER_2
    // TODO : ChargerFlag, ErrFlag
    data[5] |= 0x02;

    // TODO : Overflow err expression
    data[6] = 0x00;
    data[6] += cmd_bin._dribble_power;
    data[6] <<= 4;
    data[6] += cmd_bin._kick_power;

    // Make checksum roots
    /*data[8] = 0x00;
    for (size_t i=2; i<8; i++) {
        data[8] ^= data[i];
    }
    data[9] = data[8] ^ 0xFF;*/

    return  true;
}


// convert MKS unit to binary data in order to send as packet
ScrambleSerializer::RobotCommand_Binary ScrambleSerializer::scalingToBinary(RobotCommand robot_command)
{
  RobotCommand_Binary command_binary;

  // TODO:copy instances
  command_binary._id = robot_command._id;

  // Velocity vx
  robot_command._vel_x = round(robot_command._vel_x * 128 / Vx_max) + 127;
  if (robot_command._vel_x > 254) {
    command_binary._vel_x = 254;
  } else if(robot_command._vel_x < 0){
    command_binary._vel_x = 0;
  } else{
    command_binary._vel_x = robot_command._vel_x;    
  }

  // Velocity vy
  robot_command._vel_y = round(robot_command._vel_y * 128 / Vy_max) + 127;
  if (robot_command._vel_y > 254) {
    command_binary._vel_y = 254;
  } else if(robot_command._vel_y < 0){
    command_binary._vel_y = 0;
  } else{
    command_binary._vel_y = robot_command._vel_y;    
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
