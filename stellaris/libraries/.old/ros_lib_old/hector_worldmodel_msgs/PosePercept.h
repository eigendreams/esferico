#ifndef _ROS_hector_worldmodel_msgs_PosePercept_h
#define _ROS_hector_worldmodel_msgs_PosePercept_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "hector_worldmodel_msgs/PerceptInfo.h"

namespace hector_worldmodel_msgs
{

  class PosePercept : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::PoseWithCovariance pose;
      hector_worldmodel_msgs::PerceptInfo info;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->info.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "hector_worldmodel_msgs/PosePercept"; };
    const char * getMD5(){ return "3b0c987b9e39c7b12b9e71be97a0c021"; };

  };

}
#endif