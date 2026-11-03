#ifndef OV3640_HOST_AC_FW_CMDRESPONSE_HPP
#define OV3640_HOST_AC_FW_CMDRESPONSE_HPP
#include "Fw/Types/BasicTypes.hpp"
namespace Fw {
struct CmdResponse {
  enum T : U32 { OK = 0, EXECUTION_ERROR = 1, VALIDATION_ERROR = 2 };
  T e;
  CmdResponse(T v = OK) : e(v) {}
  bool operator==(T o) const { return e == o; }
};
}
#endif
