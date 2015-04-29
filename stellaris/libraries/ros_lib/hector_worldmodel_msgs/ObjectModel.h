#ifndef _ROS_hector_worldmodel_msgs_ObjectModel_h
#define _ROS_hector_worldmodel_msgs_ObjectModel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "hector_worldmodel_msgs/Object.h"

namespace hector_worldmodel_msgs
{

  class ObjectModel : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t objects_length;
      hector_worldmodel_msgs::Object st_objects;
      hector_worldmodel_msgs::Object * objects;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = objects_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < objects_length; i++){
      offset += this->objects[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t objects_lengthT = *(inbuffer + offset++);
      if(objects_lengthT > objects_length)
        this->objects = (hector_worldmodel_msgs::Object*)realloc(this->objects, objects_lengthT * sizeof(hector_worldmodel_msgs::Object));
      offset += 3;
      objects_length = objects_lengthT;
      for( uint8_t i = 0; i < objects_length; i++){
      offset += this->st_objects.deserialize(inbuffer + offset);
        memcpy( &(this->objects[i]), &(this->st_objects), sizeof(hector_worldmodel_msgs::Object));
      }
     return offset;
    }

    const char * getType(){ return "hector_worldmodel_msgs/ObjectModel"; };
    const char * getMD5(){ return "0e38ac3cfe11f476b7d55135cfbc074a"; };

  };

}
#endif