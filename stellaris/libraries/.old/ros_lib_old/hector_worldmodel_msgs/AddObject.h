#ifndef _ROS_SERVICE_AddObject_h
#define _ROS_SERVICE_AddObject_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hector_worldmodel_msgs/Object.h"

namespace hector_worldmodel_msgs
{

static const char ADDOBJECT[] = "hector_worldmodel_msgs/AddObject";

  class AddObjectRequest : public ros::Msg
  {
    public:
      hector_worldmodel_msgs::Object object;
      bool map_to_next_obstacle;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->object.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_map_to_next_obstacle;
      u_map_to_next_obstacle.real = this->map_to_next_obstacle;
      *(outbuffer + offset + 0) = (u_map_to_next_obstacle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->map_to_next_obstacle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->object.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_map_to_next_obstacle;
      u_map_to_next_obstacle.base = 0;
      u_map_to_next_obstacle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->map_to_next_obstacle = u_map_to_next_obstacle.real;
      offset += sizeof(this->map_to_next_obstacle);
     return offset;
    }

    const char * getType(){ return ADDOBJECT; };
    const char * getMD5(){ return "e8c82f515de9d91badcd349a0de2b742"; };

  };

  class AddObjectResponse : public ros::Msg
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

    const char * getType(){ return ADDOBJECT; };
    const char * getMD5(){ return "45a6d744054ce4dbd66f71ccfdc20273"; };

  };

  class AddObject {
    public:
    typedef AddObjectRequest Request;
    typedef AddObjectResponse Response;
  };

}
#endif
