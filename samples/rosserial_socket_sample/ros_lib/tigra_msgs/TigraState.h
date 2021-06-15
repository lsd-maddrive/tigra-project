#ifndef _ROS_tigra_msgs_TigraState_h
#define _ROS_tigra_msgs_TigraState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tigra_msgs
{

  class TigraState : public ros::Msg
  {
    public:
      typedef float _rotation_speed_type;
      _rotation_speed_type rotation_speed;
      typedef float _angle_steering_type;
      _angle_steering_type angle_steering;

    TigraState():
      rotation_speed(0),
      angle_steering(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_rotation_speed;
      u_rotation_speed.real = this->rotation_speed;
      *(outbuffer + offset + 0) = (u_rotation_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotation_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotation_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotation_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotation_speed);
      union {
        float real;
        uint32_t base;
      } u_angle_steering;
      u_angle_steering.real = this->angle_steering;
      *(outbuffer + offset + 0) = (u_angle_steering.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_steering.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_steering.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_steering.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_steering);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_rotation_speed;
      u_rotation_speed.base = 0;
      u_rotation_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotation_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotation_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotation_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotation_speed = u_rotation_speed.real;
      offset += sizeof(this->rotation_speed);
      union {
        float real;
        uint32_t base;
      } u_angle_steering;
      u_angle_steering.base = 0;
      u_angle_steering.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_steering.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_steering.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_steering.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_steering = u_angle_steering.real;
      offset += sizeof(this->angle_steering);
     return offset;
    }

    virtual const char * getType() override { return "tigra_msgs/TigraState"; };
    virtual const char * getMD5() override { return "70164e1370b7f4a9aceb7b66a42aec65"; };

  };

}
#endif
