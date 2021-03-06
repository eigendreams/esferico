#ifndef _ROS_gazebo_msgs_ODEPhysics_h
#define _ROS_gazebo_msgs_ODEPhysics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

  class ODEPhysics : public ros::Msg
  {
    public:
      bool auto_disable_bodies;
      uint32_t sor_pgs_precon_iters;
      uint32_t sor_pgs_iters;
      float sor_pgs_w;
      float sor_pgs_rms_error_tol;
      float contact_surface_layer;
      float contact_max_correcting_vel;
      float cfm;
      float erp;
      uint32_t max_contacts;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_disable_bodies;
      u_auto_disable_bodies.real = this->auto_disable_bodies;
      *(outbuffer + offset + 0) = (u_auto_disable_bodies.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->auto_disable_bodies);
      *(outbuffer + offset + 0) = (this->sor_pgs_precon_iters >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sor_pgs_precon_iters >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sor_pgs_precon_iters >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sor_pgs_precon_iters >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sor_pgs_precon_iters);
      *(outbuffer + offset + 0) = (this->sor_pgs_iters >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sor_pgs_iters >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sor_pgs_iters >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sor_pgs_iters >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sor_pgs_iters);
      offset += serializeAvrFloat64(outbuffer + offset, this->sor_pgs_w);
      offset += serializeAvrFloat64(outbuffer + offset, this->sor_pgs_rms_error_tol);
      offset += serializeAvrFloat64(outbuffer + offset, this->contact_surface_layer);
      offset += serializeAvrFloat64(outbuffer + offset, this->contact_max_correcting_vel);
      offset += serializeAvrFloat64(outbuffer + offset, this->cfm);
      offset += serializeAvrFloat64(outbuffer + offset, this->erp);
      *(outbuffer + offset + 0) = (this->max_contacts >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->max_contacts >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->max_contacts >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->max_contacts >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_contacts);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_auto_disable_bodies;
      u_auto_disable_bodies.base = 0;
      u_auto_disable_bodies.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->auto_disable_bodies = u_auto_disable_bodies.real;
      offset += sizeof(this->auto_disable_bodies);
      this->sor_pgs_precon_iters =  ((uint32_t) (*(inbuffer + offset)));
      this->sor_pgs_precon_iters |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sor_pgs_precon_iters |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sor_pgs_precon_iters |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sor_pgs_precon_iters);
      this->sor_pgs_iters =  ((uint32_t) (*(inbuffer + offset)));
      this->sor_pgs_iters |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sor_pgs_iters |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sor_pgs_iters |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sor_pgs_iters);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sor_pgs_w));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->sor_pgs_rms_error_tol));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->contact_surface_layer));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->contact_max_correcting_vel));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->cfm));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->erp));
      this->max_contacts =  ((uint32_t) (*(inbuffer + offset)));
      this->max_contacts |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->max_contacts |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->max_contacts |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->max_contacts);
     return offset;
    }

    const char * getType(){ return "gazebo_msgs/ODEPhysics"; };
    const char * getMD5(){ return "667d56ddbd547918c32d1934503dc335"; };

  };

}
#endif