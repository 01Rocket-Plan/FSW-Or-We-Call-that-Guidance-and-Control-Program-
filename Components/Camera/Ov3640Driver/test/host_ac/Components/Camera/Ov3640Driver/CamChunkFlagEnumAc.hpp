// Host-test stand-in for FPP autocoded CamChunkFlag (enumerator names/values match CamTypes.fpp).
#ifndef Components_CamChunkFlagEnumAc_HPP
#define Components_CamChunkFlagEnumAc_HPP

#include <cstdint>

namespace Components {

  class CamChunkFlag {
    public:
      using SerialType = std::int32_t;
      enum T {
        NONE = 0,
        END = 1,
        TRUNCATED = 2,
      };
      using t = enum T;

      enum {
        SERIALIZED_SIZE = sizeof(SerialType),
        NUM_CONSTANTS = 3,
      };

      CamChunkFlag() : e(NONE) {}
      CamChunkFlag(const enum T e1) : e(e1) {}
      CamChunkFlag(const SerialType e1) : e(static_cast<enum T>(e1)) {}
      CamChunkFlag(const CamChunkFlag& obj) : e(obj.e) {}

      CamChunkFlag& operator=(const CamChunkFlag& obj) { e = obj.e; return *this; }
      CamChunkFlag& operator=(enum T e1) { e = e1; return *this; }
      operator enum T() const { return e; }
      bool operator==(enum T e1) const { return e == e1; }
      bool operator!=(enum T e1) const { return e != e1; }

      enum T e;
  };

}

#endif
