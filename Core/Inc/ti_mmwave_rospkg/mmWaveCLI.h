#ifndef _ROS_SERVICE_mmWaveCLI_h
#define _ROS_SERVICE_mmWaveCLI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ti_mmwave_rospkg
{

static const char MMWAVECLI[] = "ti_mmwave_rospkg/mmWaveCLI";

  class mmWaveCLIRequest : public ros::Msg
  {
    public:
      typedef const char* _comm_type;
      _comm_type comm;

    mmWaveCLIRequest():
      comm("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_comm = strlen(this->comm);
      varToArr(outbuffer + offset, length_comm);
      offset += 4;
      memcpy(outbuffer + offset, this->comm, length_comm);
      offset += length_comm;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_comm;
      arrToVar(length_comm, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_comm; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_comm-1]=0;
      this->comm = (char *)(inbuffer + offset-1);
      offset += length_comm;
     return offset;
    }

    const char * getType(){ return MMWAVECLI; };
    const char * getMD5(){ return "705dab568ba6ff458350c8b88cb19648"; };

  };

  class mmWaveCLIResponse : public ros::Msg
  {
    public:
      typedef const char* _resp_type;
      _resp_type resp;

    mmWaveCLIResponse():
      resp("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_resp = strlen(this->resp);
      varToArr(outbuffer + offset, length_resp);
      offset += 4;
      memcpy(outbuffer + offset, this->resp, length_resp);
      offset += length_resp;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_resp;
      arrToVar(length_resp, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_resp; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_resp-1]=0;
      this->resp = (char *)(inbuffer + offset-1);
      offset += length_resp;
     return offset;
    }

    const char * getType(){ return MMWAVECLI; };
    const char * getMD5(){ return "b791c1a4a4f0cee32b54dd1a73706a59"; };

  };

  class mmWaveCLI {
    public:
    typedef mmWaveCLIRequest Request;
    typedef mmWaveCLIResponse Response;
  };

}
#endif
