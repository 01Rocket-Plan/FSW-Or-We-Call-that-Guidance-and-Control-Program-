// Dictionary ID self-test for SdLogger (Bot1 EVR 0x2000–0x2005; Bot2 TLM 0x2008–0x2013; image EVR 0x2006–0x2009).
#include <gtest/gtest.h>
#include "Components/Recorder/SdLogger/SdLoggerComponentAc.hpp"

namespace {

class SdLoggerIdProbe : public Components::SdLoggerComponentBase {
  public:
    enum {
        kEventQueueFullDrop = EVENTID_QUEUEFULLDROP,
        kEventSdError = EVENTID_SDERROR,
        kEventEnterReady = EVENTID_ENTERREADY,
        kEventEnterFault = EVENTID_ENTERFAULT,
        kEventSeqGap = EVENTID_SEQGAP,                     // abs 0x2004
        kEventTimeNonMonotonic = EVENTID_TIMENONMONOTONIC, // abs 0x2005
        kEventImageChunkDrop = EVENTID_IMAGECHUNKDROP,     // abs 0x2006
        kEventImageFrameGap = EVENTID_IMAGEFRAMEGAP,       // abs 0x2007
        kEventImageChunkGap = EVENTID_IMAGECHUNKGAP,       // abs 0x2008
        kEventImageTruncatedSeen = EVENTID_IMAGETRUNCATEDSEEN, // abs 0x2009
        kTlmWriteCount = CHANNELID_WRITECOUNT,
        kTlmDropCount = CHANNELID_DROPCOUNT,
        kTlmRecordQueueHwm = CHANNELID_RECORDQUEUEHWM,
        kTlmSdWritePoolHwm = CHANNELID_SDWRITEPOOLHWM,
        kTlmSdErrorCount = CHANNELID_SDERRORCOUNT,
        kTlmSdStatus = CHANNELID_SDSTATUS,
        kTlmLastSeq = CHANNELID_LASTSEQ,
        kTlmLastT_s = CHANNELID_LASTT_S,
        kTlmHsState = CHANNELID_HSSTATE,           // abs 0x2008
        kTlmLoggerSeq = CHANNELID_LOGGERSEQ,       // abs 0x2009
        kTlmProducerSeq = CHANNELID_PRODUCERSEQ,   // abs 0x200A
        kTlmCardModel = CHANNELID_CARDMODEL,       // abs 0x200B
        kTlmCardRole = CHANNELID_CARDROLE,         // abs 0x200C
        kTlmImageChunkCount = CHANNELID_IMAGECHUNKCOUNT,     // abs 0x200D
        kTlmImageDropCount = CHANNELID_IMAGEDROPCOUNT,       // abs 0x200E
        kTlmImageFrameCount = CHANNELID_IMAGEFRAMECOUNT,     // abs 0x200F
        kTlmImageBytesWritten = CHANNELID_IMAGEBYTESWRITTEN, // abs 0x2010
        kTlmImageQueueHwm = CHANNELID_IMAGEQUEUEHWM,         // abs 0x2011
        kTlmLastImageFrame = CHANNELID_LASTIMAGEFRAME,       // abs 0x2012
        kTlmLastImageChunk = CHANNELID_LASTIMAGECHUNK,       // abs 0x2013
    };

    void schedIn_handler(FwIndexType, U32) override {}
    void imuIn_handler(FwIndexType, const RocketTypes::ImuSample&) override {}
    void baroIn_handler(FwIndexType, const RocketTypes::BaroSample&) override {}
    void envIn_handler(FwIndexType, const RocketTypes::EnvSample&) override {}
    void gnssIn_handler(FwIndexType, const RocketTypes::GnssFix&) override {}
    void stateIn_handler(FwIndexType, const RocketTypes::StateOut&) override {}
    void finCmdIn_handler(FwIndexType, const RocketTypes::FinCmd&) override {}
    void servoCmdIn_handler(FwIndexType, const RocketTypes::ServoCmd&) override {}
    void sepCmdIn_handler(FwIndexType, const RocketTypes::SepCmd&) override {}
    void mpu6050In_handler(FwIndexType, const RocketTypes::Mpu6050Sample&) override {}
    void humTempIn_handler(FwIndexType, const RocketTypes::HumTempSample&) override {}
    void usImageIn_handler(FwIndexType, U32, U32, U8, Fw::Buffer&) override {}
    void dsImageIn_handler(FwIndexType, U32, U32, U8, Fw::Buffer&) override {}
    void START_cmdHandler(FwOpcodeType, U32) override {}
    void STOP_cmdHandler(FwOpcodeType, U32) override {}
    void FLUSH_cmdHandler(FwOpcodeType, U32) override {}
};

}  // namespace

TEST(SdLoggerDictId, EventLocalOpcodes) {
    EXPECT_EQ(SdLoggerIdProbe::kEventQueueFullDrop, 0x0);
    EXPECT_EQ(SdLoggerIdProbe::kEventSdError, 0x1);
    EXPECT_EQ(SdLoggerIdProbe::kEventEnterReady, 0x2);
    EXPECT_EQ(SdLoggerIdProbe::kEventEnterFault, 0x3);
    EXPECT_EQ(SdLoggerIdProbe::kEventSeqGap, 0x4);
    EXPECT_EQ(SdLoggerIdProbe::kEventTimeNonMonotonic, 0x5);
    EXPECT_EQ(SdLoggerIdProbe::kEventImageChunkDrop, 0x6);
    EXPECT_EQ(SdLoggerIdProbe::kEventImageFrameGap, 0x7);
    EXPECT_EQ(SdLoggerIdProbe::kEventImageChunkGap, 0x8);
    EXPECT_EQ(SdLoggerIdProbe::kEventImageTruncatedSeen, 0x9);
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
    EXPECT_EQ(SdLoggerIdProbe::kTlmHsState, 0x8);
    EXPECT_EQ(SdLoggerIdProbe::kTlmLoggerSeq, 0x9);
    EXPECT_EQ(SdLoggerIdProbe::kTlmProducerSeq, 0xA);
    EXPECT_EQ(SdLoggerIdProbe::kTlmCardModel, 0xB);
    EXPECT_EQ(SdLoggerIdProbe::kTlmCardRole, 0xC);
    EXPECT_EQ(SdLoggerIdProbe::kTlmImageChunkCount, 0xD);
    EXPECT_EQ(SdLoggerIdProbe::kTlmImageDropCount, 0xE);
    EXPECT_EQ(SdLoggerIdProbe::kTlmImageFrameCount, 0xF);
    EXPECT_EQ(SdLoggerIdProbe::kTlmImageBytesWritten, 0x10);
    EXPECT_EQ(SdLoggerIdProbe::kTlmImageQueueHwm, 0x11);
    EXPECT_EQ(SdLoggerIdProbe::kTlmLastImageFrame, 0x12);
    EXPECT_EQ(SdLoggerIdProbe::kTlmLastImageChunk, 0x13);
}

TEST(SdLoggerDictId, ModelRoleEnumsPresent) {
    EXPECT_EQ(static_cast<FwEnumStoreType>(Components::SdCardModel::MICRO_SDHC_UHS_I_16GB), 0);
    EXPECT_EQ(static_cast<FwEnumStoreType>(Components::SdCardRole::DS_LOG), 0);
    EXPECT_EQ(static_cast<FwEnumStoreType>(Components::SdCardRole::US_LOG), 1);
}
