// Host-test stand-in for FPP autocoded CamFault (enumerator names/values match CamTypes.fpp).
#ifndef Components_CamFaultEnumAc_HPP
#define Components_CamFaultEnumAc_HPP

#include <cstdint>

namespace Components {

  class CamFault {
    public:
      using SerialType = std::int32_t;
      enum T {
        NONE = 0,
        ID_MISMATCH = 1,
        TYPE_ROLE_MISMATCH = 2,
        BUS_TIMEOUT = 3,
        FRAME_TIMEOUT = 4,
        FRAME_TRUNCATED = 5,
      };
      using t = enum T;

      enum {
        SERIALIZED_SIZE = sizeof(SerialType),
        NUM_CONSTANTS = 6,
      };

      CamFault() : e(NONE) {}
      CamFault(const enum T e1) : e(e1) {}
      CamFault(const SerialType e1) : e(static_cast<enum T>(e1)) {}
      CamFault(const CamFault& obj) : e(obj.e) {}

      CamFault& operator=(const CamFault& obj) { e = obj.e; return *this; }
      CamFault& operator=(enum T e1) { e = e1; return *this; }
      operator enum T() const { return e; }
      bool operator==(enum T e1) const { return e == e1; }
      bool operator!=(enum T e1) const { return e != e1; }

      enum T e;
  };

}

#endif
