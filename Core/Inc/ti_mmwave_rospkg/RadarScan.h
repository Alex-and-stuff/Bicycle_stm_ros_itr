#ifndef _ROS_ti_mmwave_rospkg_RadarScan_h
#define _ROS_ti_mmwave_rospkg_RadarScan_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace ti_mmwave_rospkg
{

  class RadarScan : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _point_id_type;
      _point_id_type point_id;
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _z_type;
      _z_type z;
      typedef float _range_type;
      _range_type range;
      typedef float _velocity_type;
      _velocity_type velocity;
      typedef uint16_t _doppler_bin_type;
      _doppler_bin_type doppler_bin;
      typedef float _bearing_type;
      _bearing_type bearing;
      typedef float _intensity_type;
      _intensity_type intensity;

    RadarScan():
      header(),
      point_id(0),
      x(0),
      y(0),
      z(0),
      range(0),
      velocity(0),
      doppler_bin(0),
      bearing(0),
      intensity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->point_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->point_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->point_id);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_range;
      u_range.real = this->range;
      *(outbuffer + offset + 0) = (u_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->range);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
      *(outbuffer + offset + 0) = (this->doppler_bin >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->doppler_bin >> (8 * 1)) & 0xFF;
      offset += sizeof(this->doppler_bin);
      union {
        float real;
        uint32_t base;
      } u_bearing;
      u_bearing.real = this->bearing;
      *(outbuffer + offset + 0) = (u_bearing.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bearing.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bearing.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bearing.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bearing);
      union {
        float real;
        uint32_t base;
      } u_intensity;
      u_intensity.real = this->intensity;
      *(outbuffer + offset + 0) = (u_intensity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intensity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intensity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intensity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->intensity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->point_id =  ((uint16_t) (*(inbuffer + offset)));
      this->point_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->point_id);
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
      union {
        float real;
        uint32_t base;
      } u_range;
      u_range.base = 0;
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->range = u_range.real;
      offset += sizeof(this->range);
      union {
        float real;
        uint32_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      this->doppler_bin =  ((uint16_t) (*(inbuffer + offset)));
      this->doppler_bin |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->doppler_bin);
      union {
        float real;
        uint32_t base;
      } u_bearing;
      u_bearing.base = 0;
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bearing.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bearing = u_bearing.real;
      offset += sizeof(this->bearing);
      union {
        float real;
        uint32_t base;
      } u_intensity;
      u_intensity.base = 0;
      u_intensity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_intensity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_intensity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_intensity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->intensity = u_intensity.real;
      offset += sizeof(this->intensity);
     return offset;
    }

    const char * getType(){ return "ti_mmwave_rospkg/RadarScan"; };
    const char * getMD5(){ return "7a726cbc7d2934bb55d96dada9040f86"; };

  };

}
#endif
