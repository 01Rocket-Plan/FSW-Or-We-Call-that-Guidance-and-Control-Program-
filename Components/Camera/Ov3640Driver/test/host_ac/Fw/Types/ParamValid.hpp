#ifndef OV3640_HOST_AC_FW_PARAMVALID_HPP
#define OV3640_HOST_AC_FW_PARAMVALID_HPP
namespace Fw {
struct ParamValid {
  enum T { VALID = 0, INVALID = 1, DEFAULT = 2 };
  T e = VALID;
};
}
#endif
