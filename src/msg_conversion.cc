#include "msg_conversion.h"

namespace l2l_calib {
namespace conversion {

SimpleTime ToLocalTime( const ros::Time& time )
{
  SimpleTime local_time;
  local_time.secs = time.sec;
  local_time.nsecs = time.nsec;
  
  return local_time;
}

SimpleTime ToLocalTime( const PclTimeStamp& time )
{
  return SimpleTime::fromNSec( time * 1000ull );
}
  
}
}