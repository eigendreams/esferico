#ifndef _ROS_SERVICE_GetObjectModel_h
#define _ROS_SERVICE_GetObjectModel_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hector_worldmodel_msgs/ObjectModel.h"

namespace hector_worldmodel_msgs
{

static const char GETOBJECTMODEL[] = "hector_worldmodel_msgs/GetObjectModel";

  class GetObjectModelRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return GETOBJECTMODEL; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetObjectModelResponse : public ros::Msg
  {
    public:
      hector_worldmodel_msgs::ObjectModel model;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->model.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->model.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETOBJECTMODEL; };
    const char * getMD5(){ return "361f26c46d94ccf48a94c138afa225e8"; };

  };

  class GetObjectModel {
    public:
    typedef GetObjectModelRequest Request;
    typedef GetObjectModelResponse Response;
  };

}
#endif
