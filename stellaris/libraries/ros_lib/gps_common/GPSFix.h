#ifndef _ROS_gps_common_GPSFix_h
#define _ROS_gps_common_GPSFix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "gps_common/GPSStatus.h"

namespace gps_common
{

  class GPSFix : public ros::Msg
  {
    public:
      std_msgs::Header header;
      gps_common::GPSStatus status;
      float latitude;
      float longitude;
      float altitude;
      float track;
      float speed;
      float climb;
      float pitch;
      float roll;
      float dip;
      float time;
      float gdop;
      float pdop;
      float hdop;
      float vdop;
      float tdop;
      float err;
      float err_horz;
      float err_vert;
      float err_track;
      float err_speed;
      float err_climb;
      float err_time;
      float err_pitch;
      float err_roll;
      float err_dip;
      float position_covariance[9];
      uint8_t position_covariance_type;
      enum { COVARIANCE_TYPE_UNKNOWN =  0 };
      enum { COVARIANCE_TYPE_APPROXIMATED =  1 };
      enum { COVARIANCE_TYPE_DIAGONAL_KNOWN =  2 };
      enum { COVARIANCE_TYPE_KNOWN =  3 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->track);
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->climb);
      offset += serializeAvrFloat64(outbuffer + offset, this->pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->dip);
      offset += serializeAvrFloat64(outbuffer + offset, this->time);
      offset += serializeAvrFloat64(outbuffer + offset, this->gdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->pdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->hdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->vdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->tdop);
      offset += serializeAvrFloat64(outbuffer + offset, this->err);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_horz);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_vert);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_track);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_climb);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_time);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_pitch);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_roll);
      offset += serializeAvrFloat64(outbuffer + offset, this->err_dip);
      for( uint8_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_covariance[i]);
      }
      *(outbuffer + offset + 0) = (this->position_covariance_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->track));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->climb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->dip));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->gdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->hdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->tdop));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_horz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_vert));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_track));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_climb));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_time));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_pitch));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_roll));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->err_dip));
      for( uint8_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position_covariance[i]));
      }
      this->position_covariance_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_covariance_type);
     return offset;
    }

    const char * getType(){ return "gps_common/GPSFix"; };
    const char * getMD5(){ return "3db3d0a7bc53054c67c528af84710b70"; };

  };

}
#endif