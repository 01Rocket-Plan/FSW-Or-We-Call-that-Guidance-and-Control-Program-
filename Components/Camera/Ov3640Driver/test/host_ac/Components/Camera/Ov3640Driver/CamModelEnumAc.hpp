// Host-test stand-in for FPP autocoded CamModel (enumerator names/values match CamTypes.fpp).
#ifndef Components_CamModelEnumAc_HPP
#define Components_CamModelEnumAc_HPP

#include <cstdint>

namespace Components {

  class CamModel {
    public:
      using SerialType = std::int32_t;
      enum T {
        OV3640 = 0,
        ESP32_CAM = 1,
      };
      using t = enum T;

      enum {
        SERIALIZED_SIZE = sizeof(SerialType),
        NUM_CONSTANTS = 2,
      };

      CamModel() : e(OV3640) {}
      CamModel(const enum T e1) : e(e1) {}
      CamModel(const SerialType e1) : e(static_cast<enum T>(e1)) {}
      CamModel(const CamModel& obj) : e(obj.e) {}

      CamModel& operator=(const CamModel& obj) { e = obj.e; return *this; }
      CamModel& operator=(enum T e1) { e = e1; return *this; }
      operator enum T() const { return e; }
      bool operator==(enum T e1) const { return e == e1; }
      bool operator!=(enum T e1) const { return e != e1; }

      enum T e;
  };

}

#endif
