// Host-test stand-in for FPP autocoded CamRole (enumerator names/values match CamTypes.fpp).
#ifndef Components_CamRoleEnumAc_HPP
#define Components_CamRoleEnumAc_HPP

#include <cstdint>

namespace Components {

  class CamRole {
    public:
      using SerialType = std::int32_t;
      enum T {
        US_CAM = 0,
        DS_CAM = 1,
      };
      using t = enum T;

      enum {
        SERIALIZED_SIZE = sizeof(SerialType),
        NUM_CONSTANTS = 2,
      };

      CamRole() : e(US_CAM) {}
      CamRole(const enum T e1) : e(e1) {}
      CamRole(const SerialType e1) : e(static_cast<enum T>(e1)) {}
      CamRole(const CamRole& obj) : e(obj.e) {}

      CamRole& operator=(const CamRole& obj) { e = obj.e; return *this; }
      CamRole& operator=(enum T e1) { e = e1; return *this; }
      operator enum T() const { return e; }
      bool operator==(enum T e1) const { return e == e1; }
      bool operator!=(enum T e1) const { return e != e1; }

      enum T e;
  };

}

#endif
