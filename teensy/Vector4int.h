#ifndef _ROS_astrocent_Vector4int_h
#define _ROS_astrocent_Vector4int_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace astrocent
{

  class Vector4int : public ros::Msg
  {
    public:
      typedef int16_t _m1_type;
      _m1_type m1;
      typedef int16_t _m2_type;
      _m2_type m2;
      typedef int16_t _m3_type;
      _m3_type m3;
      typedef int16_t _m4_type;
      _m4_type m4;

    Vector4int():
      m1(0),
      m2(0),
      m3(0),
      m4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_m1;
      u_m1.real = this->m1;
      *(outbuffer + offset + 0) = (u_m1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->m1);
      union {
        int16_t real;
        uint16_t base;
      } u_m2;
      u_m2.real = this->m2;
      *(outbuffer + offset + 0) = (u_m2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->m2);
      union {
        int16_t real;
        uint16_t base;
      } u_m3;
      u_m3.real = this->m3;
      *(outbuffer + offset + 0) = (u_m3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->m3);
      union {
        int16_t real;
        uint16_t base;
      } u_m4;
      u_m4.real = this->m4;
      *(outbuffer + offset + 0) = (u_m4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->m4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_m1;
      u_m1.base = 0;
      u_m1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->m1 = u_m1.real;
      offset += sizeof(this->m1);
      union {
        int16_t real;
        uint16_t base;
      } u_m2;
      u_m2.base = 0;
      u_m2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->m2 = u_m2.real;
      offset += sizeof(this->m2);
      union {
        int16_t real;
        uint16_t base;
      } u_m3;
      u_m3.base = 0;
      u_m3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->m3 = u_m3.real;
      offset += sizeof(this->m3);
      union {
        int16_t real;
        uint16_t base;
      } u_m4;
      u_m4.base = 0;
      u_m4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->m4 = u_m4.real;
      offset += sizeof(this->m4);
     return offset;
    }

    virtual const char * getType() override { return "astrocent/Vector4int"; };
    virtual const char * getMD5() override { return "be6b534ce8e4e6302ef1ebbdde25f9a7"; };

  };

}
#endif
