#ifndef _ROS_finder_FourArmsInt16_h
#define _ROS_finder_FourArmsInt16_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace finder
{

  class FourArmsInt16 : public ros::Msg
  {
    public:
      int16_t mtfr;
      int16_t mtfl;
      int16_t mtbr;
      int16_t mtbl;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_mtfr;
      u_mtfr.real = this->mtfr;
      *(outbuffer + offset + 0) = (u_mtfr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mtfr.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mtfr);
      union {
        int16_t real;
        uint16_t base;
      } u_mtfl;
      u_mtfl.real = this->mtfl;
      *(outbuffer + offset + 0) = (u_mtfl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mtfl.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mtfl);
      union {
        int16_t real;
        uint16_t base;
      } u_mtbr;
      u_mtbr.real = this->mtbr;
      *(outbuffer + offset + 0) = (u_mtbr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mtbr.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mtbr);
      union {
        int16_t real;
        uint16_t base;
      } u_mtbl;
      u_mtbl.real = this->mtbl;
      *(outbuffer + offset + 0) = (u_mtbl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mtbl.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mtbl);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_mtfr;
      u_mtfr.base = 0;
      u_mtfr.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mtfr.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mtfr = u_mtfr.real;
      offset += sizeof(this->mtfr);
      union {
        int16_t real;
        uint16_t base;
      } u_mtfl;
      u_mtfl.base = 0;
      u_mtfl.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mtfl.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mtfl = u_mtfl.real;
      offset += sizeof(this->mtfl);
      union {
        int16_t real;
        uint16_t base;
      } u_mtbr;
      u_mtbr.base = 0;
      u_mtbr.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mtbr.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mtbr = u_mtbr.real;
      offset += sizeof(this->mtbr);
      union {
        int16_t real;
        uint16_t base;
      } u_mtbl;
      u_mtbl.base = 0;
      u_mtbl.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mtbl.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mtbl = u_mtbl.real;
      offset += sizeof(this->mtbl);
     return offset;
    }

    const char * getType(){ return "finder/FourArmsInt16"; };
    const char * getMD5(){ return "360615cafb5092f3977cf13a65073ffd"; };

  };

}
#endif