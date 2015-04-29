#ifndef _ROS_hector_worldmodel_msgs_ObjectState_h
#define _ROS_hector_worldmodel_msgs_ObjectState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hector_worldmodel_msgs
{

  class ObjectState : public ros::Msg
  {
    public:
      int8_t state;
      enum { UNKNOWN =  0 };
      enum { PENDING =  1 };
      enum { ACTIVE =  2 };
      enum { INACTIVE =  3 };
      enum { CONFIRMED =  -1 };
      enum { DISCARDED =  -2 };
      enum { APPROACHING =  -3 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "hector_worldmodel_msgs/ObjectState"; };
    const char * getMD5(){ return "5bb1b6744a4e40af3e4b8b56b4e06597"; };

  };

}
#endif