#ifndef _ROS_arm_interface_Arm_h
#define _ROS_arm_interface_Arm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_interface
{

  class Arm : public ros::Msg
  {
    public:
      int16_t base;
      int16_t brazo;
      int16_t antebrazo;
      int16_t munnieca;
      int16_t palma;
      int16_t dynamixel;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_base;
      u_base.real = this->base;
      *(outbuffer + offset + 0) = (u_base.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->base);
      union {
        int16_t real;
        uint16_t base;
      } u_brazo;
      u_brazo.real = this->brazo;
      *(outbuffer + offset + 0) = (u_brazo.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brazo.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->brazo);
      union {
        int16_t real;
        uint16_t base;
      } u_antebrazo;
      u_antebrazo.real = this->antebrazo;
      *(outbuffer + offset + 0) = (u_antebrazo.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_antebrazo.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->antebrazo);
      union {
        int16_t real;
        uint16_t base;
      } u_munnieca;
      u_munnieca.real = this->munnieca;
      *(outbuffer + offset + 0) = (u_munnieca.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_munnieca.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->munnieca);
      union {
        int16_t real;
        uint16_t base;
      } u_palma;
      u_palma.real = this->palma;
      *(outbuffer + offset + 0) = (u_palma.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_palma.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->palma);
      union {
        int16_t real;
        uint16_t base;
      } u_dynamixel;
      u_dynamixel.real = this->dynamixel;
      *(outbuffer + offset + 0) = (u_dynamixel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dynamixel.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dynamixel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_base;
      u_base.base = 0;
      u_base.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->base = u_base.real;
      offset += sizeof(this->base);
      union {
        int16_t real;
        uint16_t base;
      } u_brazo;
      u_brazo.base = 0;
      u_brazo.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brazo.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->brazo = u_brazo.real;
      offset += sizeof(this->brazo);
      union {
        int16_t real;
        uint16_t base;
      } u_antebrazo;
      u_antebrazo.base = 0;
      u_antebrazo.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_antebrazo.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->antebrazo = u_antebrazo.real;
      offset += sizeof(this->antebrazo);
      union {
        int16_t real;
        uint16_t base;
      } u_munnieca;
      u_munnieca.base = 0;
      u_munnieca.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_munnieca.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->munnieca = u_munnieca.real;
      offset += sizeof(this->munnieca);
      union {
        int16_t real;
        uint16_t base;
      } u_palma;
      u_palma.base = 0;
      u_palma.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_palma.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->palma = u_palma.real;
      offset += sizeof(this->palma);
      union {
        int16_t real;
        uint16_t base;
      } u_dynamixel;
      u_dynamixel.base = 0;
      u_dynamixel.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dynamixel.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dynamixel = u_dynamixel.real;
      offset += sizeof(this->dynamixel);
     return offset;
    }

    const char * getType(){ return "arm_interface/Arm"; };
    const char * getMD5(){ return "88878733614110778843defa752651ec"; };

  };

}
#endif