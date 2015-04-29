#ifndef _ROS_SERVICE_VerifyObject_h
#define _ROS_SERVICE_VerifyObject_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hector_worldmodel_msgs/Object.h"

namespace hector_worldmodel_msgs
{

static const char VERIFYOBJECT[] = "hector_worldmodel_msgs/VerifyObject";

  class VerifyObjectRequest : public ros::Msg
  {
    public:
      hector_worldmodel_msgs::Object object;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->object.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->object.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return VERIFYOBJECT; };
    const char * getMD5(){ return "45a6d744054ce4dbd66f71ccfdc20273"; };

  };

  class VerifyObjectResponse : public ros::Msg
  {
    public:
      uint8_t response;
      enum { UNKNOWN =  0 };
      enum { DISCARD =  1 };
      enum { CONFIRM =  2 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->response >> (8 * 0)) & 0xFF;
      offset += sizeof(this->response);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->response =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->response);
     return offset;
    }

    const char * getType(){ return VERIFYOBJECT; };
    const char * getMD5(){ return "a4e2509e523147799a63deb6a40a0721"; };

  };

  class VerifyObject {
    public:
    typedef VerifyObjectRequest Request;
    typedef VerifyObjectResponse Response;
  };

}
#endif
