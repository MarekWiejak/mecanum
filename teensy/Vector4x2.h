#ifndef _ROS_astrocent_Vector4x2_h
#define _ROS_astrocent_Vector4x2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "astrocent/Vector4float.h"
#include "astrocent/Vector4int.h"

namespace astrocent
{

  class Vector4x2 : public ros::Msg
  {
    public:
      typedef astrocent::Vector4float _rpm_type;
      _rpm_type rpm;
      typedef astrocent::Vector4int _pwm_type;
      _pwm_type pwm;

    Vector4x2():
      rpm(),
      pwm()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->rpm.serialize(outbuffer + offset);
      offset += this->pwm.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->rpm.deserialize(inbuffer + offset);
      offset += this->pwm.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "astrocent/Vector4x2"; };
    virtual const char * getMD5() override { return "6b77213f5f2041e3da7568bff28fefbc"; };

  };

}
#endif
