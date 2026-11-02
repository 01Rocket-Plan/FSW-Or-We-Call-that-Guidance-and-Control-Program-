#include "NvFlags.hpp"

namespace Components {

NvFlags::NvFlags(const char* const compName, ::NvFlags::INvStore& store)
    : NvFlagsComponentBase(compName), m_core(store) {}

NvFlags::~NvFlags() {}

void NvFlags::configure() {
    (void)m_core.init();
    publishState();
}

void NvFlags::publishState() {
    this->tlmWrite_FlagsMask(m_core.sepReleasedGet());
    this->tlmWrite_Phase(m_core.phase());
    this->tlmWrite_WriteCount(m_core.writeCount());
    this->tlmWrite_SepCommanded(m_core.sepCommanded());
    this->tlmWrite_DsChuteCommanded(m_core.dsChuteCommanded());
}

Fw::Success NvFlags::sepReleasedSet_handler(FwIndexType portNum, U8 mask) {
    (void)portNum;
    Fw::Time t = this->getTime();
    U32 ts = t.getSeconds();
    auto st = m_core.sepReleasedSet(mask, ts);
    if (st != ::NvFlags::CoreStatus::Success) {
        this->log_WARNING_HI_NvWriteFail(mask);
        return Fw::Success::FAILURE;
    }
    this->log_ACTIVITY_HI_NvSepLatched(m_core.sepReleasedGet());
    publishState();
    return Fw::Success::SUCCESS;
}

U8 NvFlags::sepReleasedGet_handler(FwIndexType portNum) {
    (void)portNum;
    return m_core.sepReleasedGet();
}

Fw::Success NvFlags::sepCommanded_handler(FwIndexType portNum) {
    (void)portNum;
    Fw::Time t = this->getTime();
    auto st = m_core.setSepCommanded(t.getSeconds());
    if (st != ::NvFlags::CoreStatus::Success) {
        // Already set, or write failed. Distinguish via latch state.
        if (!m_core.sepCommanded()) {
            this->log_WARNING_HI_NvWriteFail(0);
            return Fw::Success::FAILURE;
        }
        return Fw::Success::FAILURE;
    }
    publishState();
    return Fw::Success::SUCCESS;
}

Fw::Success NvFlags::dsChuteCommanded_handler(FwIndexType portNum) {
    (void)portNum;
    Fw::Time t = this->getTime();
    auto st = m_core.setDsChuteCommanded(t.getSeconds());
    if (st != ::NvFlags::CoreStatus::Success) {
        if (!m_core.dsChuteCommanded()) {
            this->log_WARNING_HI_NvWriteFail(0);
            return Fw::Success::FAILURE;
        }
        return Fw::Success::FAILURE;
    }
    publishState();
    return Fw::Success::SUCCESS;
}

U32 NvFlags::phaseGet_handler(FwIndexType portNum) {
    (void)portNum;
    return m_core.phase();
}

Fw::Success NvFlags::phaseSet_handler(FwIndexType portNum, U32 phaseVal) {
    (void)portNum;
    Fw::Time t = this->getTime();
    auto st = m_core.setPhase(phaseVal, t.getSeconds());
    if (st != ::NvFlags::CoreStatus::Success) {
        this->log_WARNING_HI_NvWriteFail(0);
        return Fw::Success::FAILURE;
    }
    publishState();
    return Fw::Success::SUCCESS;
}

}  // namespace Components
