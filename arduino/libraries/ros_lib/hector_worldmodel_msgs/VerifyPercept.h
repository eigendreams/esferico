#ifndef _ROS_SERVICE_VerifyPercept_h
#define _ROS_SERVICE_VerifyPercept_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hector_worldmodel_msgs/PosePercept.h"

namespace hector_worldmodel_msgs
{

static const char VERIFYPERCEPT[] = "hector_worldmodel_msgs/VerifyPercept";

  class VerifyPerceptRequest : public ros::Msg
  {
    public:
      hector_worldmodel_msgs::PosePercept percept;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->percept.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->percept.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return VERIFYPERCEPT; };
    const char * getMD5(){ return "9bd63c0c584500367d789d059094e625"; };

  };

  class VerifyPerceptResponse : public ros::Msg
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

    const char * getType(){ return VERIFYPERCEPT; };
    const char * getMD5(){ return "a4e2509e523147799a63deb6a40a0721"; };

  };

  class VerifyPercept {
    public:
    typedef VerifyPerceptRequest Request;
    typedef VerifyPerceptResponse Response;
  };

}
#endif
