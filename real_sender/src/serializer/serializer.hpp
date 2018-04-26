#ifndef  __SERIALIZER_HPP__
#define __SERIALIZER_HPP__

#include  <iostream>
#include  <vector>
#include  <string>

class RobotCommand
{
public:
  enum KickType{
    STRAIGHT, CHIP
  };

  unsigned int  _id;
  float _vel_x, _vel_y;  // 0 >= _vel_norm > 4.0, -M_PI/2 >= _vel_theta >= M_PI/2
  float _omega;                 // -2*M_PI >= omega >= 2*M_PI[rad/s]
  KickType _kick_type;
  float _dribble_power, _kick_power;  // 0 >= _dribble_power > 16, 0 >= _kick_power > 16

  RobotCommand();
  // 0 is straight, 1 is chip : kick_type
  RobotCommand(unsigned int id, float vel_x, float vel_y, float omega,
               float dribble_power,
               float kick_power, KickType kick_type);
};


class ScrambleSerializer {
public:
  bool  serialize(RobotCommand cmd, char* data);

private:
  static const char  HEADER;
  static const int  NUM_DATA;
  static const int  Vx_max;
  static const int Vy_max;


  class RobotCommand_Binary
  {
  public:
    static const unsigned int  STRAIGHT = 0, CHIP = 1;
    unsigned int  _id,
                  _vel_x, _vel_y, _omega,
                  _dribble_power, _kick_power, _kick_type;
  };

  RobotCommand_Binary scalingToBinary(RobotCommand robot_command);
  float piTopi(float angle);
  float zeroTo2pi(float angle);
};

#endif
