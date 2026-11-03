// Dictionary ID self-test for SdLogger (紀錄組子Bot1).
#include <gtest/gtest.h>
#include "Components/Recorder/SdLogger/SdLoggerComponentAc.hpp"

namespace {

//! Access protected EVENTID_/CHANNELID_ enums without constructing the component.
class SdLoggerIdProbe : public Components::SdLoggerComponentBase {
  public:
    enum {
        kEventQueueFullDrop = EVENTID_QUEUEFULLDROP,
        kEventSdError = EVENTID_SDERROR,
        kEventEnterReady = EVENTID_ENTERREADY,
        kEventEnterFault = EVENTID_ENTERFAULT,
        kEventSeqGap = EVENTID_SEQGAP,                     // abs 0x2004
        kEventTimeNonMonotonic = EVENTID_TIMENONMONOTONIC, // abs 0x2005
        kTlmWriteCount = CHANNELID_WRITECOUNT,
        kTlmDropCount = CHANNELID_DROPCOUNT,
        kTlmRecordQueueHwm = CHANNELID_RECORDQUEUEHWM,
        kTlmSdWritePoolHwm = CHANNELID_SDWRITEPOOLHWM,
        kTlmSdErrorCount = CHANNELID_SDERRORCOUNT,
        kTlmSdStatus = CHANNELID_SDSTATUS,
        kTlmLastSeq = CHANNELID_LASTSEQ,
        kTlmLastT_s = CHANNELID_LASTT_S,
    };

    void schedIn_handler(FwIndexType, U32) override {}
};

}  // namespace

TEST(SdLoggerDictId, EventLocalOpcodes) {
    EXPECT_EQ(SdLoggerIdProbe::kEventQueueFullDrop, 0x0);
    EXPECT_EQ(SdLoggerIdProbe::kEventSdError, 0x1);
    EXPECT_EQ(SdLoggerIdProbe::kEventEnterReady, 0x2);
    EXPECT_EQ(SdLoggerIdProbe::kEventEnterFault, 0x3);
    EXPECT_EQ(SdLoggerIdProbe::kEventSeqGap, 0x4);
    EXPECT_EQ(SdLoggerIdProbe::kEventTimeNonMonotonic, 0x5);
}

TEST(SdLoggerDictId, TelemetryLocalOpcodes) {
    EXPECT_EQ(SdLoggerIdProbe::kTlmWriteCount, 0x0);
    EXPECT_EQ(SdLoggerIdProbe::kTlmDropCount, 0x1);
    EXPECT_EQ(SdLoggerIdProbe::kTlmRecordQueueHwm, 0x2);
    EXPECT_EQ(SdLoggerIdProbe::kTlmSdWritePoolHwm, 0x3);
    EXPECT_EQ(SdLoggerIdProbe::kTlmSdErrorCount, 0x4);
    EXPECT_EQ(SdLoggerIdProbe::kTlmSdStatus, 0x5);
    EXPECT_EQ(SdLoggerIdProbe::kTlmLastSeq, 0x6);
    EXPECT_EQ(SdLoggerIdProbe::kTlmLastT_s, 0x7);
}
