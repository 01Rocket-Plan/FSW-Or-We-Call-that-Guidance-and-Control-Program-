// FakeGnssCore：GnssCore 完成前的最小假核心，只做 NMEA 斷句＋校驗和，用來把 Rig 接起來。
// 等組長的 GnssCore 進來後，T1~T8 改用真核心；本檔僅測試用，不進飛行版本。
#ifndef GNSS_FAKEGNSSCORE_HPP
#define GNSS_FAKEGNSSCORE_HPP
#include "Components/Gnss/GnssDriver/GnssIfaces.hpp"
#include <cstdlib>
#include <string>
#include <vector>

namespace Gnss {
struct FakeGnssCore {
  explicit FakeGnssCore(IGnssUart& u) : uart(u) {}
  IGnssUart& uart;
  std::string line;
  std::vector<std::string> good;  // 校驗正確的完整句子（不含 \r\n）
  uint32_t badChecksum = 0, truncated = 0, bytes = 0, reads = 0;

  void tick() {
    uint8_t buf[256]; reads++;
    int32_t n = uart.readBytes(buf, sizeof buf);  // 每個 tick 只讀一次（介面約定）
    for (int32_t i = 0; i < n; i++) feed(static_cast<char>(buf[i]));
    bytes += n > 0 ? static_cast<uint32_t>(n) : 0u;
  }
  void feed(char c) {
    if (c == '$') { if (!line.empty()) truncated++; line = "$"; return; }
    if (line.empty()) return;
    if (c == '\n') { finish(); line.clear(); return; }
    if (c != '\r') line += c;
    if (line.size() > 90) { truncated++; line.clear(); }  // NMEA 上限 82 字元
  }
  void finish() {
    size_t star = line.find('*');
    if (star == std::string::npos || star + 3 != line.size()) { truncated++; return; }
    uint8_t c = 0; for (size_t i = 1; i < star; i++) c ^= static_cast<uint8_t>(line[i]);
    if (std::strtoul(line.substr(star + 1).c_str(), nullptr, 16) != c) { badChecksum++; return; }
    good.push_back(line);
  }
  size_t count(const char* type) const {
    size_t k = 0; for (auto& s : good) if (s.compare(1, 5, type) == 0) k++; return k;
  }
};
}  // namespace Gnss
#endif
