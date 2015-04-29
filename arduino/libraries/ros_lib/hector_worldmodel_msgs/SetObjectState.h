#ifndef _ROS_SERVICE_SetObjectState_h
#define _ROS_SERVICE_SetObjectState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hector_worldmodel_msgs/ObjectState.h"

namespace hector_worldmodel_msgs
{

static const char SETOBJECTSTATE[] = "hector_worldmodel_msgs/SetObjectState";

  class SetObjectStateRequest : public ros::Msg
  {
    public:
      const char* object_id;
      hector_worldmodel_msgs::ObjectState new_state;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_object_id = strlen(this->object_id);
      memcpy(outbuffer + offset, &length_object_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->object_id, length_object_id);
      offset += length_object_id;
      offset += this->new_state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_object_id;
      memcpy(&length_object_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_object_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_object_id-1]=0;
      this->object_id = (char *)(inbuffer + offset-1);
      offset += length_object_id;
      offset += this->new_state.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETOBJECTSTATE; };
    const char * getMD5(){ return "f1ca4bb9d91ddc88e48d16dceb6ab0e5"; };

  };

  class SetObjectStateResponse : public ros::Msg
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

    const char * getType(){ return SETOBJECTSTATE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetObjectState {
    public:
    typedef SetObjectStateRequest Request;
    typedef SetObjectStateResponse Response;
  };

}
#endif
