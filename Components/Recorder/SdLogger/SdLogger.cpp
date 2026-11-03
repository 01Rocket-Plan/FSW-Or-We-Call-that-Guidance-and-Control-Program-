#include "Components/Recorder/SdLogger/SdLogger.hpp"
#include "Components/Recorder/SdLogger/RecordChannels.hpp"

namespace Components {

static RocketTypes::DrvState toFpp(LoggerDrvState s) {
  return RocketTypes::DrvState(static_cast<RocketTypes::DrvState::T>(static_cast<U8>(s)));
}

static SdCardStatus toFppStatus(SdMediaStatus s) {
  switch (s) {
    case SdMediaStatus::OK: return SdCardStatus::OK;
    case SdMediaStatus::INIT: return SdCardStatus::INIT;
    case SdMediaStatus::BUSY: return SdCardStatus::BUSY;
    case SdMediaStatus::CARD_MISSING: return SdCardStatus::CARD_MISSING;
    case SdMediaStatus::IO_ERROR: return SdCardStatus::IO_ERROR;
    case SdMediaStatus::FULL: return SdCardStatus::FULL;
  }
  return SdCardStatus::IO_ERROR;
}

SdLogger::SdLogger(const char* compName)
    : SdLoggerComponentBase(compName),
      m_media(&m_defaultMedia),
      m_card(*m_media),
      m_core(m_card, *this),
      m_tlmDiv(0),
      m_model(SdCardModel::MICRO_SDHC_UHS_I_16GB),
      m_role(SdCardRole::DS_LOG) {}

void SdLogger::setMedia(ISdMedia* media) {
  if (media) m_media = media;
}

F64 SdLogger::now_s() {
  const Fw::Time now = this->getTime();
  return static_cast<F64>(now.getSeconds()) + static_cast<F64>(now.getUSeconds()) * 1e-6;
}

void SdLogger::loadParams() {
  Fw::ParamValid v;
  m_core.setQueueDepth(this->paramGet_RECORD_QUEUE_DEPTH(v));
  m_card.setPoolSize(this->paramGet_SD_WRITE_POOL_SIZE(v));
  m_model = this->paramGet_CARD_MODEL(v);
  m_role = this->paramGet_CARD_ROLE(v);
}

void SdLogger::parameterUpdated(FwPrmIdType) { loadParams(); }
void SdLogger::parametersLoaded() { loadParams(); }

void SdLogger::writeTlm() {
  this->tlmWrite_WriteCount(m_core.writeCount());
  this->tlmWrite_DropCount(m_core.dropCount());
  this->tlmWrite_RecordQueueHwm(m_core.queueHwm());
  this->tlmWrite_SdWritePoolHwm(m_core.writePoolHwm());
  this->tlmWrite_SdErrorCount(m_core.sdErrorCount());
  this->tlmWrite_SdStatus(toFppStatus(m_core.sdStatus()));
  this->tlmWrite_LastSeq(m_core.lastProducerSeq());
  this->tlmWrite_LastT_s(m_core.lastT_s());
  this->tlmWrite_HsState(toFpp(m_core.state()));
  this->tlmWrite_LoggerSeq(m_core.loggerSeq());
  this->tlmWrite_ProducerSeq(m_core.lastProducerSeq());
  this->tlmWrite_CardModel(m_model);
  this->tlmWrite_CardRole(m_role);
}

void SdLogger::ingestPacked(const PackedRecord& rec) { (void)m_core.ingest(rec); }

void SdLogger::schedIn_handler(FwIndexType, U32) {
  if (m_core.running()) m_core.flush();
  if (++m_tlmDiv >= 10) {
    m_tlmDiv = 0;
    writeTlm();
  }
}

void SdLogger::START_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  loadParams();
  const bool ok = m_core.start();
  this->cmdResponse_out(opCode, cmdSeq, ok ? Fw::CmdResponse::OK : Fw::CmdResponse::EXECUTION_ERROR);
}

void SdLogger::STOP_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.stop();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SdLogger::FLUSH_cmdHandler(FwOpcodeType opCode, U32 cmdSeq) {
  m_core.flush();
  this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
}

void SdLogger::imuIn_handler(FwIndexType, const RocketTypes::ImuSample& s) {
  const auto& a = s.get_acc_mps2();
  const auto& g = s.get_gyro_rads();
  float buf[7] = {a[0], a[1], a[2], g[0], g[1], g[2], s.get_gyroSat() ? 1.0f : 0.0f};
  PackedRecord rec;
  if (packRecord(RecordChannel::IMU, s.get_hdr().get_seq(), s.get_hdr().get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::baroIn_handler(FwIndexType, const RocketTypes::BaroSample& s) {
  float buf[3] = {s.get_press_Pa(), s.get_temp_C(), s.get_alt_m()};
  PackedRecord rec;
  if (packRecord(RecordChannel::BARO, s.get_hdr().get_seq(), s.get_hdr().get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::envIn_handler(FwIndexType, const RocketTypes::EnvSample& s) {
  float buf[1] = {s.get_humidity_pct()};
  PackedRecord rec;
  if (packRecord(RecordChannel::ENV, s.get_hdr().get_seq(), s.get_hdr().get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::gnssIn_handler(FwIndexType, const RocketTypes::GnssFix& fix) {
  GnssRecordPod pod{};
  pod.lat_rad = fix.get_lat_rad();
  pod.lon_rad = fix.get_lon_rad();
  pod.altMsl_m = fix.get_altMsl_m();
  pod.hdop = fix.get_hdop();
  pod.dataAge_s = fix.get_dataAge_s();
  pod.gpsWeek = fix.get_gpsWeek();
  pod.numSats = fix.get_numSats();
  pod.fixType = static_cast<uint8_t>(fix.get_fixType());
  pod.flags = static_cast<uint8_t>((fix.get_posValid() ? 1 : 0) | (fix.get_velHValid() ? 2 : 0) | (fix.get_velUValid() ? 4 : 0));
  PackedRecord rec;
  if (GnssRecordLayer::pack(fix.get_hdr().get_seq(), fix.get_hdr().get_t_s(), pod, rec)) ingestPacked(rec);
}

void SdLogger::stateIn_handler(FwIndexType, const RocketTypes::StateOut& s) {
  const auto& q = s.get_q_wxyz();
  const auto& r = s.get_rate_body_rads();
  const auto& v = s.get_vel_enu_mps();
  float buf[12] = {q[0], q[1], q[2], q[3], r[0], r[1], r[2], s.get_tilt_rad(), v[0], v[1], v[2], s.get_alt_m()};
  PackedRecord rec;
  if (packRecord(RecordChannel::STATE, s.get_seq(), s.get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::finCmdIn_handler(FwIndexType, const RocketTypes::FinCmd& c) {
  const auto& f = c.get_fin_rad();
  float buf[4] = {f[0], f[1], f[2], f[3]};
  PackedRecord rec;
  if (packRecord(RecordChannel::FIN_CMD, c.get_seq(), c.get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::servoCmdIn_handler(FwIndexType, const RocketTypes::ServoCmd& c) {
  const auto& p = c.get_pulse_us();
  uint16_t buf[4] = {p[0], p[1], p[2], p[3]};
  PackedRecord rec;
  if (packRecord(RecordChannel::SERVO_CMD, c.get_seq(), c.get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::sepCmdIn_handler(FwIndexType, const RocketTypes::SepCmd& c) {
  uint8_t buf[2] = {c.get_channelMask(), static_cast<uint8_t>(c.get_valid() ? 1 : 0)};
  PackedRecord rec;
  if (packRecord(RecordChannel::SEP_CMD, c.get_seq(), c.get_t_s(), buf, sizeof(buf), rec)) ingestPacked(rec);
}

void SdLogger::mpu6050In_handler(FwIndexType, const RocketTypes::Mpu6050Sample& s) {
  Mpu6050RecordPod pod{};
  const auto& a = s.get_accel_sensor_mps2();
  const auto& g = s.get_gyro_sensor_rads();
  for (int i = 0; i < 3; ++i) {
    pod.accel_sensor_mps2[i] = a[i];
    pod.gyro_sensor_rads[i] = g[i];
  }
  pod.accelSat = s.get_accelSat() ? 1 : 0;
  pod.gyroSat = s.get_gyroSat() ? 1 : 0;
  PackedRecord rec;
  if (Mpu6050RecordLayer::pack(s.get_hdr().get_seq(), s.get_hdr().get_t_s(), pod, rec)) ingestPacked(rec);
}

void SdLogger::humTempIn_handler(FwIndexType, const RocketTypes::HumTempSample& s) {
  Dht22RecordPod pod{s.get_temp_K(), s.get_humidity_frac()};
  PackedRecord rec;
  if (Dht22RecordLayer::pack(s.get_hdr().get_seq(), s.get_hdr().get_t_s(), pod, rec)) ingestPacked(rec);
}

void SdLogger::onQueueFullDrop(uint32_t dropped, uint32_t depth) {
  this->log_WARNING_HI_QueueFullDrop(dropped, depth);
}
void SdLogger::onSeqGap(uint32_t expected, uint32_t got) {
  this->log_WARNING_HI_SeqGap(expected, got);
}
void SdLogger::onTimeNonMonotonic(double prev_s, double now_s) {
  this->log_WARNING_HI_TimeNonMonotonic(prev_s, now_s);
}
void SdLogger::onSdError(SdMediaStatus status, uint8_t code) {
  this->log_WARNING_HI_SdError(toFppStatus(status), code);
}
void SdLogger::onEnterReady() { this->log_ACTIVITY_HI_EnterReady(); }
void SdLogger::onEnterFault(uint8_t reason) { this->log_WARNING_HI_EnterFault(reason); }

}  // namespace Components
