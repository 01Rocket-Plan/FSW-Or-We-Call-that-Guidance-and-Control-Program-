// Abstract SD media interface (host-testable; no F´, no sleep, no wall clock).
#ifndef RECORDER_ISDMEDIA_HPP
#define RECORDER_ISDMEDIA_HPP

#include <cstddef>
#include <cstdint>

namespace Components {

enum class SdMediaStatus : uint8_t {
  OK = 0,
  INIT = 1,
  BUSY = 2,
  CARD_MISSING = 3,
  IO_ERROR = 4,
  FULL = 5,
};

class ISdMedia {
 public:
  virtual ~ISdMedia() = default;
  virtual bool open() = 0;
  virtual bool append(const uint8_t* data, size_t len) = 0;
  virtual bool sync() = 0;
  virtual SdMediaStatus status() const = 0;
  virtual size_t bytesWritten() const = 0;
};

}  // namespace Components
#endif
