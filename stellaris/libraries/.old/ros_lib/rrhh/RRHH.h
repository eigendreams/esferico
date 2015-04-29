#ifndef _ROS_rrhh_RRHH_h
#define _ROS_rrhh_RRHH_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rrhh
{

  class RRHH : public ros::Msg
  {
    public:
      int16_t wheel1;
      int16_t wheel2;
      int16_t wheel3;
      int16_t wheel4;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_wheel1;
      u_wheel1.real = this->wheel1;
      *(outbuffer + offset + 0) = (u_wheel1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel1);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel2;
      u_wheel2.real = this->wheel2;
      *(outbuffer + offset + 0) = (u_wheel2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel2);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel3;
      u_wheel3.real = this->wheel3;
      *(outbuffer + offset + 0) = (u_wheel3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel3);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel4;
      u_wheel4.real = this->wheel4;
      *(outbuffer + offset + 0) = (u_wheel4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_wheel1;
      u_wheel1.base = 0;
      u_wheel1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel1 = u_wheel1.real;
      offset += sizeof(this->wheel1);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel2;
      u_wheel2.base = 0;
      u_wheel2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel2 = u_wheel2.real;
      offset += sizeof(this->wheel2);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel3;
      u_wheel3.base = 0;
      u_wheel3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel3 = u_wheel3.real;
      offset += sizeof(this->wheel3);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel4;
      u_wheel4.base = 0;
      u_wheel4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel4 = u_wheel4.real;
      offset += sizeof(this->wheel4);
     return offset;
    }

    const char * getType(){ return "rrhh/RRHH"; };
    const char * getMD5(){ return "779b6a21feca9e067fc9667b70c9d0ec"; };

  };

}
#endif