// Host-only stand-in for autocoded Ov3640DriverComponentAc.hpp
#ifndef OV3640_HOST_OV3640DRIVERCOMPONENTAC_HPP
#define OV3640_HOST_OV3640DRIVERCOMPONENTAC_HPP

#include "CamTypes.hpp"
#include "Drv/I2c.hpp"
#include "Fw/Buffer.hpp"
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/CmdResponse.hpp"
#include "Fw/Types/ParamValid.hpp"

namespace Components {

class Ov3640DriverComponentBase {
 public:
  explicit Ov3640DriverComponentBase(const char* /*name*/) {}
  virtual ~Ov3640DriverComponentBase() = default;

 protected:
  virtual void sched1Hz_handler(FwIndexType portNum, U32 context) = 0;
  virtual void sched100Hz_handler(FwIndexType portNum, U32 context) = 0;
  virtual void POWER_ON_cmdHandler(FwOpcodeType op, U32 seq) = 0;
  virtual void RESET_cmdHandler(FwOpcodeType op, U32 seq) = 0;
  virtual void STOP_cmdHandler(FwOpcodeType op, U32 seq) = 0;
  virtual void CAPTURE_START_cmdHandler(FwOpcodeType op, U32 seq) = 0;
  virtual void CAPTURE_STOP_cmdHandler(FwOpcodeType op, U32 seq) = 0;
  virtual void SIM_INJECT_cmdHandler(FwOpcodeType op, U32 seq, U8 fault) {
    (void)fault; cmdResponse_out(op, seq, Fw::CmdResponse::OK);
  }
  virtual void SIM_SET_FRAME_cmdHandler(FwOpcodeType op, U32 seq, U8 enable) {
    (void)enable; cmdResponse_out(op, seq, Fw::CmdResponse::OK);
  }
  virtual void parameterUpdated(FwPrmIdType /*id*/) {}
  virtual void parametersLoaded() {}

  U32 paramGet_BUS_TIMEOUT_MISSES(Fw::ParamValid& /*v*/) { return m_prmBus; }
  U32 paramGet_FRAME_TIMEOUT_TICKS(Fw::ParamValid& /*v*/) { return m_prmFrame; }
  U8 paramGet_ENTRIES_PER_TICK(Fw::ParamValid& /*v*/) { return m_prmEntries; }
  CamModel paramGet_MODEL(Fw::ParamValid& /*v*/) { return m_prmModel; }
  CamRole paramGet_ROLE(Fw::ParamValid& /*v*/) { return m_prmRole; }
  U32 paramGet_SCCB_ADDR(Fw::ParamValid& /*v*/) { return m_prmAddr; }
  U32 paramGet_FAULT_RECOVER_K(Fw::ParamValid& /*v*/) { return m_prmRecoverK; }
  U32 paramGet_FAULT_RECOVER_R(Fw::ParamValid& /*v*/) { return m_prmRecoverR; }
  U32 paramGet_FRAME_MAX_BYTES(Fw::ParamValid& /*v*/) { return m_prmFrameMax; }

  void tlmWrite_Status(const CamStatus& s) { m_lastTlm = s; ++m_tlmWrites; }
  void log_ACTIVITY_HI_StateChanged(CamState from, CamState to) {
    m_lastFrom = from; m_lastTo = to; ++m_stateEvents;
  }
  void log_WARNING_HI_FaultRaised(CamFault code, U32 detail) {
    m_lastFault = code; m_lastFaultDetail = detail; ++m_faultEvents;
  }
  void log_ACTIVITY_HI_RecoverAttempt(U32 attempt, U32 limit) {
    m_lastRecoverAttempt = attempt; m_lastRecoverLimit = limit; ++m_recoverEvents;
  }
  void log_WARNING_HI_FrameTruncated(U32 frameNumber, U32 bytesSent) {
    m_lastTruncFrame = frameNumber; m_lastTruncBytes = bytesSent; ++m_truncEvents;
  }
  void cmdResponse_out(FwOpcodeType op, U32 seq, Fw::CmdResponse resp) {
    m_lastOp = op; m_lastSeq = seq; m_lastResp = resp; ++m_cmdResponses;
  }

  bool isConnected_imageOut_OutputPort(FwIndexType) const { return m_imageConnected; }
  bool isConnected_sccbWrite_OutputPort(FwIndexType) const { return m_sccbConnected; }
  bool isConnected_sccbWriteRead_OutputPort(FwIndexType) const { return m_sccbConnected; }

  void imageOut_out(FwIndexType, U32 frameNumber, U32 chunkIndex, U8 flags, Fw::Buffer& data) {
    m_lastFrameNumber = frameNumber;
    m_lastChunkIndex = chunkIndex;
    m_lastFlags = flags;
    m_lastImageSize = data.getSize();
    ++m_imageOuts;
    if (flags & static_cast<U8>(CamChunkFlag::END)) ++m_endChunks;
    if (flags & static_cast<U8>(CamChunkFlag::TRUNCATED)) ++m_truncChunks;
  }

  Drv::I2cStatus sccbWrite_out(FwIndexType, U32 addr, Fw::Buffer& buf) {
    ++m_sccbWrites; m_lastSccbAddr = addr;
    if (m_sccbFail) return Drv::I2cStatus::I2C_WRITE_ERR;
    if (buf.getSize() >= 3 && buf.getData()) {
      m_lastWriteReg = static_cast<U16>((buf.getData()[0] << 8) | buf.getData()[1]);
      m_lastWriteVal = buf.getData()[2];
    }
    return Drv::I2cStatus::I2C_OK;
  }

  Drv::I2cStatus sccbWriteRead_out(FwIndexType, U32 addr, Fw::Buffer& wr, Fw::Buffer& rd) {
    ++m_sccbReads; m_lastSccbAddr = addr;
    if (m_sccbFail) return Drv::I2cStatus::I2C_READ_ERR;
    if (rd.getSize() >= 1 && rd.getData() && wr.getSize() >= 2 && wr.getData()) {
      const U16 reg = static_cast<U16>((wr.getData()[0] << 8) | wr.getData()[1]);
      if (reg == 0x300A) rd.getData()[0] = 0x36;
      else if (reg == 0x300B) rd.getData()[0] = 0x4C;
      else rd.getData()[0] = 0;
    }
    return Drv::I2cStatus::I2C_OK;
  }

 public:
  void hostSetImageConnected(bool v) { m_imageConnected = v; }
  void hostSetSccbConnected(bool v) { m_sccbConnected = v; }
  void hostSetSccbFail(bool v) { m_sccbFail = v; }
  void hostSetParams(U32 bus, U32 frame, U8 entries, CamModel model, CamRole role, U32 addr,
                      U32 recoverK = 5, U32 recoverR = 3, U32 frameMax = 65536) {
    m_prmBus = bus; m_prmFrame = frame; m_prmEntries = entries;
    m_prmModel = model; m_prmRole = role; m_prmAddr = addr;
    m_prmRecoverK = recoverK; m_prmRecoverR = recoverR; m_prmFrameMax = frameMax;
  }

  void hostSched1Hz(U32 ctx = 0) { sched1Hz_handler(0, ctx); }
  void hostSched100Hz(U32 ctx = 0) { sched100Hz_handler(0, ctx); }
  void hostCmdPowerOn(FwOpcodeType op = 1, U32 seq = 1) { POWER_ON_cmdHandler(op, seq); }
  void hostCmdReset(FwOpcodeType op = 2, U32 seq = 1) { RESET_cmdHandler(op, seq); }
  void hostCmdStop(FwOpcodeType op = 3, U32 seq = 1) { STOP_cmdHandler(op, seq); }
  void hostCmdCaptureStart(FwOpcodeType op = 4, U32 seq = 1) { CAPTURE_START_cmdHandler(op, seq); }
  void hostCmdCaptureStop(FwOpcodeType op = 5, U32 seq = 1) { CAPTURE_STOP_cmdHandler(op, seq); }
  void hostCmdSimInject(U8 fault, FwOpcodeType op = 6, U32 seq = 1) { SIM_INJECT_cmdHandler(op, seq, fault); }
  void hostCmdSimSetFrame(U8 en, FwOpcodeType op = 7, U32 seq = 1) { SIM_SET_FRAME_cmdHandler(op, seq, en); }
  void hostLoadParams() { parametersLoaded(); }

  CamStatus m_lastTlm{};
  U32 m_tlmWrites = 0;
  U32 m_stateEvents = 0;
  U32 m_faultEvents = 0;
  U32 m_recoverEvents = 0;
  U32 m_truncEvents = 0;
  U32 m_lastRecoverAttempt = 0;
  U32 m_lastRecoverLimit = 0;
  U32 m_lastTruncFrame = 0;
  U32 m_lastTruncBytes = 0;
  CamState m_lastFrom = CamState::OFF;
  CamState m_lastTo = CamState::OFF;
  CamFault m_lastFault = CamFault::NONE;
  U32 m_lastFaultDetail = 0;
  U32 m_cmdResponses = 0;
  FwOpcodeType m_lastOp = 0;
  U32 m_lastSeq = 0;
  Fw::CmdResponse m_lastResp{Fw::CmdResponse::OK};
  U32 m_imageOuts = 0;
  U32 m_endChunks = 0;
  U32 m_truncChunks = 0;
  U32 m_lastFrameNumber = 0;
  U32 m_lastChunkIndex = 0;
  U8 m_lastFlags = 0;
  U32 m_lastImageSize = 0;
  U32 m_sccbWrites = 0;
  U32 m_sccbReads = 0;
  U32 m_lastSccbAddr = 0;
  U16 m_lastWriteReg = 0;
  U8 m_lastWriteVal = 0;

 private:
  bool m_imageConnected = true;
  bool m_sccbConnected = true;
  bool m_sccbFail = false;
  U32 m_prmBus = 3;
  U32 m_prmFrame = 20;
  U8 m_prmEntries = 8;
  CamModel m_prmModel = CamModel::OV3640;
  CamRole m_prmRole = CamRole::US_CAM;
  U32 m_prmAddr = 0x3C;
  U32 m_prmRecoverK = 5;
  U32 m_prmRecoverR = 3;
  U32 m_prmFrameMax = 65536;
};

}  // namespace Components
#endif
