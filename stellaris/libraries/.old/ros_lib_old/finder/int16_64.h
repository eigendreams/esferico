#ifndef _ROS_finder_int16_64_h
#define _ROS_finder_int16_64_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace finder
{

  class int16_64 : public ros::Msg
  {
    public:
      int16_t data[64];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 64; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 64; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "finder/int16_64"; };
    const char * getMD5(){ return "3f9b54d05ebcaae21bf2081b0c8c9a85"; };

  };

}
#endif