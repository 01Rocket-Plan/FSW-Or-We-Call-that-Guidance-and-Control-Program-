#include "LinkLossView.hpp"

#include <cstdio>
#include <cstdlib>

using namespace GroundLinkView;

#define CHECK(cond)                                                                         \
  do {                                                                                      \
    if (!(cond)) {                                                                          \
      std::fprintf(stderr, "CHECK failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__);        \
      std::abort();                                                                         \
    }                                                                                       \
  } while (0)

static RxFrame mk(DownlinkSource s, uint16_t apid, uint32_t seq, uint32_t boot, double t,
                  bool valid = true) {
  RxFrame f;
  f.src = s;
  f.apid = apid;
  f.seq = seq;
  f.bootCount = boot;
  f.t_s = t;
  f.valid = valid;
  return f;
}

int main() {
  {
    LinkLossView v;
    v.setLossOfSignalSec(3.0);
    const uint16_t apid = 0x123;
    for (uint32_t s = 1; s <= 4; ++s) {
      v.onFrame(mk(DownlinkSource::DS, apid, s, 7, 0.1 * static_cast<double>(s)));
    }
    v.onFrame(mk(DownlinkSource::DS, apid, 8, 7, 0.5));
    SourceStats st = v.sourceStats(DownlinkSource::DS);
    CHECK(st.gapCount == 1);
    CHECK(st.lostTotal == 3);
    GapSpan g{};
    CHECK(v.findGap(apid, 5, &g));
    CHECK(g.seqLastLost == 7);
    CHECK(g.lost == 3);
    CHECK(g.tStart < g.tEnd);
  }

  {
    LinkLossView v;
    v.setLossOfSignalSec(3.0);
    v.onFrame(mk(DownlinkSource::DS, 0x10, 1, 1, 0.0));
    v.tick(3.0);
    CHECK(!v.sourceStats(DownlinkSource::DS).los);
    v.tick(3.0001);
    CHECK(v.sourceStats(DownlinkSource::DS).los);
    v.onFrame(mk(DownlinkSource::DS, 0x10, 2, 1, 4.0));
    v.tick(4.0);
    CHECK(!v.sourceStats(DownlinkSource::DS).los);
  }

  {
    LinkLossView v;
    const uint16_t apid = 0x22;
    v.onFrame(mk(DownlinkSource::DS, apid, 100, 1, 0.1));
    v.onFrame(mk(DownlinkSource::DS, apid, 1, 2, 0.2));
    CHECK(v.sourceStats(DownlinkSource::DS).gapCount == 0);
  }

  {
    LinkLossView v;
    v.onFrame(mk(DownlinkSource::US, 0x01, 10, 3, 1.0));
    v.onFrame(mk(DownlinkSource::US, 0x01, 99, 3, 1.1, false));
    SourceStats st = v.sourceStats(DownlinkSource::US);
    CHECK(st.framesValid == 1);
    CHECK(st.framesInvalid == 1);
    CHECK(st.gapCount == 0);
  }

  {
    LinkLossView v;
    const uint16_t apid = 0x55;
    v.onFrame(mk(DownlinkSource::DS, apid, 0xFFFFFFFEu, 1, 1.0));
    v.onFrame(mk(DownlinkSource::DS, apid, 0x00000001u, 1, 1.1));
    SourceStats st = v.sourceStats(DownlinkSource::DS);
    CHECK(st.gapCount == 1);
    CHECK(st.lostTotal == 2);
    GapSpan g{};
    CHECK(v.findGap(apid, 0xFFFFFFFFu, &g));
    CHECK(g.seqLastLost == 0x00000000u);
    CHECK(g.lost == 2);
  }

  {
    LinkLossView v;
    const uint16_t apid = 0x66;
    v.onFrame(mk(DownlinkSource::DS, apid, 50, 1, 1.0));
    v.onFrame(mk(DownlinkSource::DS, apid, 40, 1, 1.1));
    CHECK(v.sourceStats(DownlinkSource::DS).gapCount == 0);
    v.onFrame(mk(DownlinkSource::DS, apid, 52, 1, 1.2));
    CHECK(v.sourceStats(DownlinkSource::DS).gapCount == 1);
    CHECK(v.sourceStats(DownlinkSource::DS).lostTotal == 1);
  }

  {
    LinkLossView v;
    const uint16_t apid = 0x33;
    for (uint32_t s = 1; s <= 5; ++s) {
      v.onFrame(mk(DownlinkSource::DS, apid, s, 1, 0.2 * static_cast<double>(s)));
    }
    v.tick(1.0);
    CHECK(v.apidRate(DownlinkSource::DS, apid) > 0.0);
  }

  std::printf("LinkLossHostTest: PASS\n");
  return 0;
}
