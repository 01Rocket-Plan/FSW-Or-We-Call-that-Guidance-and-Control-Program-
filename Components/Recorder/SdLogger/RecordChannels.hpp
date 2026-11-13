// Record channel IDs + record-only pack helpers (MPU6050 / DHT22 / GNSS / DS).
// No I2C/SPI — serialization only for SdLogger.
#ifndef RECORDER_RECORDCHANNELS_HPP
#define RECORDER_RECORDCHANNELS_HPP

#include <cstdint>
#include <cstring>

namespace Components {

enum class RecordChannel : uint16_t {
  IMU = 1,
  BARO = 2,
  ENV = 3,
  GNSS = 4,
  STATE = 5,
  FIN_CMD = 6,
  SERVO_CMD = 7,
  SEP_CMD = 8,
  MPU6050 = 9,
  HUM_TEMP = 10,
  IMAGE = 11,  // US cam / Esp32Cam 2KB chunks (Components.CamImageOut)
};

//! CamChunkFlag bits (Camera CamTypes) — END|TRUNCATED
constexpr uint8_t kCamChunkFlagEnd = 1;
constexpr uint8_t kCamChunkFlagTruncated = 2;

//! Image chunk queued for media write (separate from PackedRecord 256B samples)
struct ImageChunkRecord {
  uint32_t frameNumber = 0;
  uint32_t chunkIndex = 0;
  uint8_t flags = 0;
  uint16_t len = 0;
  double t_s = 0.0;
  uint8_t bytes[2048]{};
};

struct PackedRecord {
  uint16_t channelId = 0;
  uint32_t producerSeq = 0;
  double t_s = 0.0;
  uint16_t len = 0;
  uint8_t bytes[256]{};
};

inline bool packRecord(RecordChannel ch, uint32_t producerSeq, double t_s, const void* payload,
                       uint16_t payloadLen, PackedRecord& out) {
  if (payloadLen > sizeof(out.bytes)) return false;
  out.channelId = static_cast<uint16_t>(ch);
  out.producerSeq = producerSeq;
  out.t_s = t_s;
  out.len = payloadLen;
  if (payloadLen && payload) std::memcpy(out.bytes, payload, payloadLen);
  return true;
}

//! US MPU6050 record-only layer (host POD)
struct Mpu6050RecordPod {
  float accel_sensor_mps2[3];
  float gyro_sensor_rads[3];
  uint8_t accelSat;
  uint8_t gyroSat;
};

struct Mpu6050RecordLayer {
  static bool pack(uint32_t seq, double t_s, const Mpu6050RecordPod& p, PackedRecord& out) {
    return packRecord(RecordChannel::MPU6050, seq, t_s, &p, sizeof(p), out);
  }
};

//! US DHT22 / HumTemp record-only layer
struct Dht22RecordPod {
  float temp_K;
  float humidity_frac;
};

struct Dht22RecordLayer {
  static bool pack(uint32_t seq, double t_s, const Dht22RecordPod& p, PackedRecord& out) {
    return packRecord(RecordChannel::HUM_TEMP, seq, t_s, &p, sizeof(p), out);
  }
};

//! GNSS record-only layer
struct GnssRecordPod {
  double lat_rad;
  double lon_rad;
  float altMsl_m;
  float hdop;
  float dataAge_s;
  uint16_t gpsWeek;
  uint8_t numSats;
  uint8_t fixType;
  uint8_t flags;
};

struct GnssRecordLayer {
  static bool pack(uint32_t seq, double t_s, const GnssRecordPod& p, PackedRecord& out) {
    return packRecord(RecordChannel::GNSS, seq, t_s, &p, sizeof(p), out);
  }
};

}  // namespace Components
#endif
