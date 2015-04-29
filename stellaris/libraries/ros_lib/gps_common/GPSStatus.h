#ifndef _ROS_gps_common_GPSStatus_h
#define _ROS_gps_common_GPSStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace gps_common
{

  class GPSStatus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint16_t satellites_used;
      uint8_t satellite_used_prn_length;
      int32_t st_satellite_used_prn;
      int32_t * satellite_used_prn;
      uint16_t satellites_visible;
      uint8_t satellite_visible_prn_length;
      int32_t st_satellite_visible_prn;
      int32_t * satellite_visible_prn;
      uint8_t satellite_visible_z_length;
      int32_t st_satellite_visible_z;
      int32_t * satellite_visible_z;
      uint8_t satellite_visible_azimuth_length;
      int32_t st_satellite_visible_azimuth;
      int32_t * satellite_visible_azimuth;
      uint8_t satellite_visible_snr_length;
      int32_t st_satellite_visible_snr;
      int32_t * satellite_visible_snr;
      int16_t status;
      uint16_t motion_source;
      uint16_t orientation_source;
      uint16_t position_source;
      enum { STATUS_NO_FIX = -1    };
      enum { STATUS_FIX = 0        };
      enum { STATUS_SBAS_FIX = 1   };
      enum { STATUS_GBAS_FIX = 2   };
      enum { STATUS_DGPS_FIX = 18  };
      enum { STATUS_WAAS_FIX = 33  };
      enum { SOURCE_NONE = 0  };
      enum { SOURCE_GPS = 1  };
      enum { SOURCE_POINTS = 2  };
      enum { SOURCE_DOPPLER = 4  };
      enum { SOURCE_ALTIMETER = 8  };
      enum { SOURCE_MAGNETIC = 16  };
      enum { SOURCE_GYRO = 32  };
      enum { SOURCE_ACCEL = 64  };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->satellites_used >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->satellites_used >> (8 * 1)) & 0xFF;
      offset += sizeof(this->satellites_used);
      *(outbuffer + offset++) = satellite_used_prn_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < satellite_used_prn_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_satellite_used_prni;
      u_satellite_used_prni.real = this->satellite_used_prn[i];
      *(outbuffer + offset + 0) = (u_satellite_used_prni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satellite_used_prni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satellite_used_prni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satellite_used_prni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satellite_used_prn[i]);
      }
      *(outbuffer + offset + 0) = (this->satellites_visible >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->satellites_visible >> (8 * 1)) & 0xFF;
      offset += sizeof(this->satellites_visible);
      *(outbuffer + offset++) = satellite_visible_prn_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < satellite_visible_prn_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_satellite_visible_prni;
      u_satellite_visible_prni.real = this->satellite_visible_prn[i];
      *(outbuffer + offset + 0) = (u_satellite_visible_prni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satellite_visible_prni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satellite_visible_prni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satellite_visible_prni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satellite_visible_prn[i]);
      }
      *(outbuffer + offset++) = satellite_visible_z_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < satellite_visible_z_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_satellite_visible_zi;
      u_satellite_visible_zi.real = this->satellite_visible_z[i];
      *(outbuffer + offset + 0) = (u_satellite_visible_zi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satellite_visible_zi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satellite_visible_zi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satellite_visible_zi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satellite_visible_z[i]);
      }
      *(outbuffer + offset++) = satellite_visible_azimuth_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < satellite_visible_azimuth_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_satellite_visible_azimuthi;
      u_satellite_visible_azimuthi.real = this->satellite_visible_azimuth[i];
      *(outbuffer + offset + 0) = (u_satellite_visible_azimuthi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satellite_visible_azimuthi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satellite_visible_azimuthi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satellite_visible_azimuthi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satellite_visible_azimuth[i]);
      }
      *(outbuffer + offset++) = satellite_visible_snr_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < satellite_visible_snr_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_satellite_visible_snri;
      u_satellite_visible_snri.real = this->satellite_visible_snr[i];
      *(outbuffer + offset + 0) = (u_satellite_visible_snri.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_satellite_visible_snri.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_satellite_visible_snri.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_satellite_visible_snri.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->satellite_visible_snr[i]);
      }
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->motion_source >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motion_source >> (8 * 1)) & 0xFF;
      offset += sizeof(this->motion_source);
      *(outbuffer + offset + 0) = (this->orientation_source >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->orientation_source >> (8 * 1)) & 0xFF;
      offset += sizeof(this->orientation_source);
      *(outbuffer + offset + 0) = (this->position_source >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_source >> (8 * 1)) & 0xFF;
      offset += sizeof(this->position_source);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->satellites_used =  ((uint16_t) (*(inbuffer + offset)));
      this->satellites_used |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->satellites_used);
      uint8_t satellite_used_prn_lengthT = *(inbuffer + offset++);
      if(satellite_used_prn_lengthT > satellite_used_prn_length)
        this->satellite_used_prn = (int32_t*)realloc(this->satellite_used_prn, satellite_used_prn_lengthT * sizeof(int32_t));
      offset += 3;
      satellite_used_prn_length = satellite_used_prn_lengthT;
      for( uint8_t i = 0; i < satellite_used_prn_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_satellite_used_prn;
      u_st_satellite_used_prn.base = 0;
      u_st_satellite_used_prn.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_satellite_used_prn.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_satellite_used_prn.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_satellite_used_prn.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_satellite_used_prn = u_st_satellite_used_prn.real;
      offset += sizeof(this->st_satellite_used_prn);
        memcpy( &(this->satellite_used_prn[i]), &(this->st_satellite_used_prn), sizeof(int32_t));
      }
      this->satellites_visible =  ((uint16_t) (*(inbuffer + offset)));
      this->satellites_visible |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->satellites_visible);
      uint8_t satellite_visible_prn_lengthT = *(inbuffer + offset++);
      if(satellite_visible_prn_lengthT > satellite_visible_prn_length)
        this->satellite_visible_prn = (int32_t*)realloc(this->satellite_visible_prn, satellite_visible_prn_lengthT * sizeof(int32_t));
      offset += 3;
      satellite_visible_prn_length = satellite_visible_prn_lengthT;
      for( uint8_t i = 0; i < satellite_visible_prn_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_satellite_visible_prn;
      u_st_satellite_visible_prn.base = 0;
      u_st_satellite_visible_prn.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_satellite_visible_prn.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_satellite_visible_prn.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_satellite_visible_prn.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_satellite_visible_prn = u_st_satellite_visible_prn.real;
      offset += sizeof(this->st_satellite_visible_prn);
        memcpy( &(this->satellite_visible_prn[i]), &(this->st_satellite_visible_prn), sizeof(int32_t));
      }
      uint8_t satellite_visible_z_lengthT = *(inbuffer + offset++);
      if(satellite_visible_z_lengthT > satellite_visible_z_length)
        this->satellite_visible_z = (int32_t*)realloc(this->satellite_visible_z, satellite_visible_z_lengthT * sizeof(int32_t));
      offset += 3;
      satellite_visible_z_length = satellite_visible_z_lengthT;
      for( uint8_t i = 0; i < satellite_visible_z_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_satellite_visible_z;
      u_st_satellite_visible_z.base = 0;
      u_st_satellite_visible_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_satellite_visible_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_satellite_visible_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_satellite_visible_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_satellite_visible_z = u_st_satellite_visible_z.real;
      offset += sizeof(this->st_satellite_visible_z);
        memcpy( &(this->satellite_visible_z[i]), &(this->st_satellite_visible_z), sizeof(int32_t));
      }
      uint8_t satellite_visible_azimuth_lengthT = *(inbuffer + offset++);
      if(satellite_visible_azimuth_lengthT > satellite_visible_azimuth_length)
        this->satellite_visible_azimuth = (int32_t*)realloc(this->satellite_visible_azimuth, satellite_visible_azimuth_lengthT * sizeof(int32_t));
      offset += 3;
      satellite_visible_azimuth_length = satellite_visible_azimuth_lengthT;
      for( uint8_t i = 0; i < satellite_visible_azimuth_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_satellite_visible_azimuth;
      u_st_satellite_visible_azimuth.base = 0;
      u_st_satellite_visible_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_satellite_visible_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_satellite_visible_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_satellite_visible_azimuth.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_satellite_visible_azimuth = u_st_satellite_visible_azimuth.real;
      offset += sizeof(this->st_satellite_visible_azimuth);
        memcpy( &(this->satellite_visible_azimuth[i]), &(this->st_satellite_visible_azimuth), sizeof(int32_t));
      }
      uint8_t satellite_visible_snr_lengthT = *(inbuffer + offset++);
      if(satellite_visible_snr_lengthT > satellite_visible_snr_length)
        this->satellite_visible_snr = (int32_t*)realloc(this->satellite_visible_snr, satellite_visible_snr_lengthT * sizeof(int32_t));
      offset += 3;
      satellite_visible_snr_length = satellite_visible_snr_lengthT;
      for( uint8_t i = 0; i < satellite_visible_snr_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_satellite_visible_snr;
      u_st_satellite_visible_snr.base = 0;
      u_st_satellite_visible_snr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_satellite_visible_snr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_satellite_visible_snr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_satellite_visible_snr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_satellite_visible_snr = u_st_satellite_visible_snr.real;
      offset += sizeof(this->st_satellite_visible_snr);
        memcpy( &(this->satellite_visible_snr[i]), &(this->st_satellite_visible_snr), sizeof(int32_t));
      }
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status = u_status.real;
      offset += sizeof(this->status);
      this->motion_source =  ((uint16_t) (*(inbuffer + offset)));
      this->motion_source |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->motion_source);
      this->orientation_source =  ((uint16_t) (*(inbuffer + offset)));
      this->orientation_source |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->orientation_source);
      this->position_source =  ((uint16_t) (*(inbuffer + offset)));
      this->position_source |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->position_source);
     return offset;
    }

    const char * getType(){ return "gps_common/GPSStatus"; };
    const char * getMD5(){ return "313baa8951fdd056c78bf61b1b07d249"; };

  };

}
#endif