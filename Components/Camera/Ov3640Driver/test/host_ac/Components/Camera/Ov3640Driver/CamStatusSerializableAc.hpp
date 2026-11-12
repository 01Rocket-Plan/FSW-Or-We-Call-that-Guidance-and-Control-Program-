// Host-test stand-in for FPP autocoded CamStatus (getters/setters; same fields and defaults).
#ifndef Components_CamStatusSerializableAc_HPP
#define Components_CamStatusSerializableAc_HPP

#include "Components/Camera/Ov3640Driver/CamFaultEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamModelEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamRoleEnumAc.hpp"
#include "Components/Camera/Ov3640Driver/CamStateEnumAc.hpp"
#include <cstdint>

namespace Components {

  class CamStatus {
    public:
      CamStatus()
          : m_camState(CamState::OFF),
            m_fault(CamFault::NONE),
            m_frameCount(0),
            m_model(CamModel::OV3640),
            m_role(CamRole::US_CAM) {}

      CamStatus(
          CamState::T camState,
          CamFault::T fault,
          std::uint32_t frameCount,
          CamModel::T model,
          CamRole::T role)
          : m_camState(camState),
            m_fault(fault),
            m_frameCount(frameCount),
            m_model(model),
            m_role(role) {}

      CamStatus(const CamStatus&) = default;
      CamStatus& operator=(const CamStatus&) = default;

      bool operator==(const CamStatus& obj) const {
        return m_camState.e == obj.m_camState.e && m_fault.e == obj.m_fault.e &&
               m_frameCount == obj.m_frameCount && m_model.e == obj.m_model.e &&
               m_role.e == obj.m_role.e;
      }
      bool operator!=(const CamStatus& obj) const { return !(*this == obj); }

      CamState::T get_camState() const { return m_camState.e; }
      CamFault::T get_fault() const { return m_fault.e; }
      std::uint32_t get_frameCount() const { return m_frameCount; }
      CamModel::T get_model() const { return m_model.e; }
      CamRole::T get_role() const { return m_role.e; }

      void set(CamState::T camState, CamFault::T fault, std::uint32_t frameCount,
               CamModel::T model, CamRole::T role) {
        set_camState(camState);
        set_fault(fault);
        set_frameCount(frameCount);
        set_model(model);
        set_role(role);
      }
      void set_camState(CamState::T camState) { m_camState = camState; }
      void set_fault(CamFault::T fault) { m_fault = fault; }
      void set_frameCount(std::uint32_t frameCount) { m_frameCount = frameCount; }
      void set_model(CamModel::T model) { m_model = model; }
      void set_role(CamRole::T role) { m_role = role; }

    private:
      CamState m_camState;
      CamFault m_fault;
      std::uint32_t m_frameCount;
      CamModel m_model;
      CamRole m_role;
  };

}

#endif
