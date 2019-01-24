#ifndef _ROS_AGV5_target_h
#define _ROS_AGV5_target_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace AGV5
{

  class target : public ros::Msg
  {
    public:
      typedef float _linvel_type;
      _linvel_type linvel;
      typedef float _angvel_type;
      _angvel_type angvel;

    target():
      linvel(0),
      angvel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linvel;
      u_linvel.real = this->linvel;
      *(outbuffer + offset + 0) = (u_linvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linvel);
      union {
        float real;
        uint32_t base;
      } u_angvel;
      u_angvel.real = this->angvel;
      *(outbuffer + offset + 0) = (u_angvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angvel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linvel;
      u_linvel.base = 0;
      u_linvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linvel = u_linvel.real;
      offset += sizeof(this->linvel);
      union {
        float real;
        uint32_t base;
      } u_angvel;
      u_angvel.base = 0;
      u_angvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angvel = u_angvel.real;
      offset += sizeof(this->angvel);
     return offset;
    }

    const char * getType(){ return "AGV5/target"; };
    const char * getMD5(){ return "08ee453c16ec05ddf8a4d8bd799cc957"; };

  };

}
#endif
