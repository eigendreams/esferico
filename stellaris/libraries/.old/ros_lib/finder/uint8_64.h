#ifndef _ROS_finder_uint8_64_h
#define _ROS_finder_uint8_64_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace finder
{

  class uint8_64 : public ros::Msg
  {
    public:
      uint8_t data[64];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 64; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 64; i++){
      this->data[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "finder/uint8_64"; };
    const char * getMD5(){ return "1843d49ae0d694138db0e7712b15974e"; };

  };

}
#endif