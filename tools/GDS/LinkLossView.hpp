// LinkLossView: ground gap / LOS display (observe-only).
// No sleep, no system clock; caller supplies time in seconds each tick.
// Fixed-size buffers; no uplink, keys, or flight-software side effects.
#ifndef TOOLS_GDS_LINKLOSSVIEW_HPP
#define TOOLS_GDS_LINKLOSSVIEW_HPP

#include <cstddef>
#include <cstdint>

namespace GroundLinkView {

enum class DownlinkSource : uint8_t { DS = 0, US = 1 };

enum class FrameKind : uint8_t { TELEMETRY = 0, EVENT = 1, FILE = 2, UNKNOWN = 3 };

struct RxFrame {
  DownlinkSource src = DownlinkSource::DS;
  uint16_t apid = 0;
  uint32_t seq = 0;
  uint32_t bootCount = 0;
  double t_s = 0.0;
  bool valid = true;
  FrameKind kind = FrameKind::UNKNOWN;
};

struct GapSpan {
  uint16_t apid = 0;
  uint32_t seqFirstLost = 0;
  uint32_t seqLastLost = 0;
  uint32_t lost = 0;
  uint32_t bootCount = 0;
  double tStart = 0.0;
  double tEnd = 0.0;
  DownlinkSource src = DownlinkSource::DS;
};

struct SourceStats {
  DownlinkSource src = DownlinkSource::DS;
  uint32_t gapCount = 0;
  uint32_t lostTotal = 0;
  uint32_t framesValid = 0;
  uint32_t framesInvalid = 0;
  double lastRxT_s = 0.0;
  bool lastRxTValid = false;
  bool los = false;
};

static constexpr uint8_t kSourceCount = 2;
static constexpr uint8_t kMaxApids = 16;
static constexpr uint8_t kMaxGapSpans = 32;
// Gaps larger than half the U32 space are treated as reorder/backwards, not loss.
static constexpr uint32_t kMaxForwardDelta = 0x80000000u;

class LinkLossView {
 public:
  LinkLossView();

  void setLossOfSignalSec(double s);
  double lossOfSignalSec() const { return m_losSec; }

  void onFrame(const RxFrame& f);
  void tick(double now_s);

  SourceStats sourceStats(DownlinkSource src) const;
  // Returns number of valid spans; *out points at internal ring (not chronological
  // when wrapped). Prefer findGap() in tests.
  uint32_t gapSpans(const GapSpan** out) const;
  bool findGap(uint16_t apid, uint32_t seqFirstLost, GapSpan* out) const;
  double apidRate(DownlinkSource src, uint16_t apid) const;

 private:
  struct ApIdState {
    bool used = false;
    uint16_t apid = 0;
    uint32_t lastSeq = 0;
    bool lastSeqValid = false;
    uint32_t bootCount = 0;
    uint32_t refFrames = 0;
    double refStartT_s = 0.0;
    bool refValid = false;
    double rate = 0.0;
  };

  ApIdState* findOrCreate(DownlinkSource src, uint16_t apid);
  void resetSourceGaps(DownlinkSource src);
  void pushGap(const GapSpan& g);

  double m_losSec = 3.0;
  ApIdState m_ap[kSourceCount][kMaxApids];
  SourceStats m_stats[kSourceCount];
  GapSpan m_gaps[kMaxGapSpans];
  uint32_t m_gapHead = 0;
  uint32_t m_gapFilled = 0;
};

}  // namespace GroundLinkView

#endif
