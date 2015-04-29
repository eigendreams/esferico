#ifndef _ROS_SERVICE_SetObjectName_h
#define _ROS_SERVICE_SetObjectName_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hector_worldmodel_msgs
{

static const char SETOBJECTNAME[] = "hector_worldmodel_msgs/SetObjectName";

  class SetObjectNameRequest : public ros::Msg
  {
    public:
      const char* object_id;
      const char* name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_object_id = strlen(this->object_id);
      memcpy(outbuffer + offset, &length_object_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->object_id, length_object_id);
      offset += length_object_id;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
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
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return SETOBJECTNAME; };
    const char * getMD5(){ return "c42e7737953fcb0f6da23160b2771456"; };

  };

  class SetObjectNameResponse : public ros::Msg
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

    const char * getType(){ return SETOBJECTNAME; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetObjectName {
    public:
    typedef SetObjectNameRequest Request;
    typedef SetObjectNameResponse Response;
  };

}
#endif
