// SdLoggerSim — same core + SdCardSimDevice (Base ID 0x2080)
#ifndef RECORDER_SDLOGGERSIM_HPP
#define RECORDER_SDLOGGERSIM_HPP

#include "Components/Recorder/SdLogger/SdLoggerSimComponentAc.hpp"
#include "Components/Recorder/SdLogger/SdCardCore.hpp"
#include "Components/Recorder/SdLogger/SdCardSimDevice.hpp"
#include "Components/Recorder/SdLogger/SdLoggerCore.hpp"

namespace Components {

class SdLoggerSim final : public SdLoggerSimComponentBase, private ISdLoggerSink {
 public:
  explicit SdLoggerSim(const char* compName);
  ~SdLoggerSim() override = default;

 private:
  void schedIn_handler(FwIndexType portNum, U32 context) override;
  void imuIn_handler(FwIndexType portNum, const RocketTypes::ImuSample& s) override;
  void baroIn_handler(FwIndexType portNum, const RocketTypes::BaroSample& s) override;
  void envIn_handler(FwIndexType portNum, const RocketTypes::EnvSample& s) override;
  void gnssIn_handler(FwIndexType portNum, const RocketTypes::GnssFix& fix) override;
  void stateIn_handler(FwIndexType portNum, const RocketTypes::StateOut& s) override;
  void finCmdIn_handler(FwIndexType portNum, const RocketTypes::FinCmd& c) override;
  void servoCmdIn_handler(FwIndexType portNum, const RocketTypes::ServoCmd& c) override;
  void sepCmdIn_handler(FwIndexType portNum, const RocketTypes::SepCmd& c) override;
  void mpu6050In_handler(FwIndexType portNum, const RocketTypes::Mpu6050Sample& s) override;
  void humTempIn_handler(FwIndexType portNum, const RocketTypes::HumTempSample& s) override;
  void usImageIn_handler(FwIndexType portNum, U32 frameNumber, U32 chunkIndex, U8 flags, Fw::Buffer& data) override;
  void dsImageIn_handler(FwIndexType portNum, U32 frameNumber, U32 chunkIndex, U8 flags, Fw::Buffer& data) override;

  void START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void FLUSH_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) override;
  void SIM_INJECT_cmdHandler(FwOpcodeType opCode, U32 cmdSeq, const Components::SdSimFault& fault) override;

  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;

  void onQueueFullDrop(uint32_t dropped, uint32_t depth) override;
  void onSeqGap(uint32_t expected, uint32_t got) override;
  void onTimeNonMonotonic(double prev_s, double now_s) override;
  void onSdError(SdMediaStatus status, uint8_t code) override;
  void onEnterReady() override;
  void onEnterFault(uint8_t reason) override;
  void onImageChunkDrop(uint32_t dropped, uint32_t depth) override;
  void onImageFrameGap(uint32_t expected, uint32_t got) override;
  void onImageChunkGap(uint32_t frameNumber, uint32_t expected, uint32_t got) override;
  void onImageTruncatedSeen(uint32_t frameNumber, uint32_t chunkIndex) override;

  void loadParams();
  void writeTlm();
  void ingestPacked(const PackedRecord& rec);
  F64 now_s();

  SdCardSimDevice m_dev;
  SdCardCore m_card;
  SdLoggerCore m_core;
  U32 m_tlmDiv;
  SdCardModel m_model;
  SdCardRole m_role;
};

}  // namespace Components
#endif
