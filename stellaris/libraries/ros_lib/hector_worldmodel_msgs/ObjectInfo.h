#ifndef _ROS_hector_worldmodel_msgs_ObjectInfo_h
#define _ROS_hector_worldmodel_msgs_ObjectInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hector_worldmodel_msgs
{

  class ObjectInfo : public ros::Msg
  {
    public:
      const char* class_id;
      const char* object_id;
      const char* name;
      float support;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_class_id = strlen(this->class_id);
      memcpy(outbuffer + offset, &length_class_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->class_id, length_class_id);
      offset += length_class_id;
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
      union {
        float real;
        uint32_t base;
      } u_support;
      u_support.real = this->support;
      *(outbuffer + offset + 0) = (u_support.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_support.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_support.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_support.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->support);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_class_id;
      memcpy(&length_class_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_class_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_class_id-1]=0;
      this->class_id = (char *)(inbuffer + offset-1);
      offset += length_class_id;
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
      union {
        float real;
        uint32_t base;
      } u_support;
      u_support.base = 0;
      u_support.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_support.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_support.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_support.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->support = u_support.real;
      offset += sizeof(this->support);
     return offset;
    }

    const char * getType(){ return "hector_worldmodel_msgs/ObjectInfo"; };
    const char * getMD5(){ return "d394469df540d3963132564bb1e1b6c6"; };

  };

}
#endif