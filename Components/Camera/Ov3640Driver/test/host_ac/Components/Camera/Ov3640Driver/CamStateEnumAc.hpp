// Host-test stand-in for FPP autocoded CamState (enumerator names/values match CamTypes.fpp).
#ifndef Components_CamStateEnumAc_HPP
#define Components_CamStateEnumAc_HPP

#include <cstdint>

namespace Components {

  class CamState {
    public:
      using SerialType = std::int32_t;
      enum T {
        OFF = 0,
        INIT = 1,
        SELFTEST = 2,
        READY = 3,
        FAULT = 4,
      };
      using t = enum T;

      enum {
        SERIALIZED_SIZE = sizeof(SerialType),
        NUM_CONSTANTS = 5,
      };

      CamState() : e(OFF) {}
      CamState(const enum T e1) : e(e1) {}
      CamState(const SerialType e1) : e(static_cast<enum T>(e1)) {}
      CamState(const CamState& obj) : e(obj.e) {}

      CamState& operator=(const CamState& obj) { e = obj.e; return *this; }
      CamState& operator=(enum T e1) { e = e1; return *this; }
      operator enum T() const { return e; }
      bool operator==(enum T e1) const { return e == e1; }
      bool operator!=(enum T e1) const { return e != e1; }

      enum T e;
  };

}

#endif
