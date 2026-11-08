#include "LinkLossView.hpp"

namespace GroundLinkView {

static inline uint8_t idx(DownlinkSource s) { return static_cast<uint8_t>(s); }

LinkLossView::LinkLossView() {
  for (uint8_t i = 0; i < kSourceCount; ++i) {
    m_stats[i].src = static_cast<DownlinkSource>(i);
    m_stats[i].lastRxT_s = 0.0;
    m_stats[i].lastRxTValid = false;
    m_stats[i].los = false;
  }
}

void LinkLossView::setLossOfSignalSec(double s) {
  if (s > 0.0) {
    m_losSec = s;
  }
}

LinkLossView::ApIdState* LinkLossView::findOrCreate(DownlinkSource src, uint16_t apid) {
  const uint8_t si = idx(src);
  if (si >= kSourceCount) {
    return nullptr;
  }
  for (uint8_t i = 0; i < kMaxApids; ++i) {
    if (m_ap[si][i].used && m_ap[si][i].apid == apid) {
      return &m_ap[si][i];
    }
  }
  for (uint8_t i = 0; i < kMaxApids; ++i) {
    if (!m_ap[si][i].used) {
      ApIdState& a = m_ap[si][i];
      a = ApIdState{};
      a.used = true;
      a.apid = apid;
      return &a;
    }
  }
  return nullptr;
}

void LinkLossView::resetSourceGaps(DownlinkSource src) {
  const uint8_t si = idx(src);
  if (si >= kSourceCount) {
    return;
  }
  for (uint8_t i = 0; i < kMaxApids; ++i) {
    if (m_ap[si][i].used) {
      m_ap[si][i].lastSeqValid = false;
    }
  }
}

void LinkLossView::pushGap(const GapSpan& g) {
  m_gaps[m_gapHead] = g;
  m_gapHead = (m_gapHead + 1u) % kMaxGapSpans;
  if (m_gapFilled < kMaxGapSpans) {
    m_gapFilled++;
  }
}

void LinkLossView::onFrame(const RxFrame& f) {
  const uint8_t si = idx(f.src);
  if (si >= kSourceCount) {
    return;
  }
  SourceStats& st = m_stats[si];
  if (!f.valid) {
    st.framesInvalid++;
    return;
  }

  // Capture previous RX time BEFORE updating (DeepSeek draft bug: tStart used
  // already-updated lastRxT_s).
  const double prevRxT = st.lastRxT_s;
  const bool hadPrevRx = st.lastRxTValid;

  st.framesValid++;
  st.lastRxT_s = f.t_s;
  st.lastRxTValid = true;

  ApIdState* a = findOrCreate(f.src, f.apid);
  if (a == nullptr) {
    return;
  }

  const bool bootChanged = a->lastSeqValid && (a->bootCount != f.bootCount);
  if (!a->lastSeqValid || bootChanged) {
    if (bootChanged) {
      resetSourceGaps(f.src);
    }
    a->lastSeq = f.seq;
    a->lastSeqValid = true;
    a->bootCount = f.bootCount;
    a->refFrames++;
    if (!a->refValid) {
      a->refStartT_s = f.t_s;
      a->refValid = true;
    }
    return;
  }

  // Unsigned delta: natural U32 wrap (e.g. FFFFFFFE -> 00000001) counts as forward.
  const uint32_t delta = f.seq - a->lastSeq;
  if (delta == 0u) {
    // Duplicate: ignore for gap accounting; still refresh boot/rate.
  } else if (delta < kMaxForwardDelta) {
    if (delta > 1u) {
      const uint32_t lost = delta - 1u;
      GapSpan g;
      g.apid = f.apid;
      g.seqFirstLost = a->lastSeq + 1u;
      g.seqLastLost = f.seq - 1u;
      g.lost = lost;
      g.bootCount = f.bootCount;
      g.tStart = hadPrevRx ? prevRxT : f.t_s;
      g.tEnd = f.t_s;
      g.src = f.src;
      pushGap(g);
      // Saturating add to avoid wrap of counters themselves.
      if (st.gapCount < UINT32_MAX) {
        st.gapCount++;
      }
      if (st.lostTotal <= UINT32_MAX - lost) {
        st.lostTotal += lost;
      } else {
        st.lostTotal = UINT32_MAX;
      }
    }
    a->lastSeq = f.seq;
  } else {
    // Backward / large reorder: do not count as gap; keep lastSeq monotonic.
  }

  a->lastSeqValid = true;
  a->bootCount = f.bootCount;
  a->refFrames++;
  if (!a->refValid) {
    a->refStartT_s = f.t_s;
    a->refValid = true;
  }
}

void LinkLossView::tick(double now_s) {
  for (uint8_t i = 0; i < kSourceCount; ++i) {
    SourceStats& st = m_stats[i];
    if (!st.lastRxTValid) {
      // No frame yet: leave los=false (link never established).
      st.los = false;
    } else {
      st.los = (now_s - st.lastRxT_s) > m_losSec;
    }

    for (uint8_t j = 0; j < kMaxApids; ++j) {
      ApIdState& a = m_ap[i][j];
      if (!a.used || !a.refValid) {
        continue;
      }
      const double dt = now_s - a.refStartT_s;
      if (dt > 0.0) {
        a.rate = static_cast<double>(a.refFrames) / dt;
      }
      a.refFrames = 0;
      a.refStartT_s = now_s;
    }
  }
}

SourceStats LinkLossView::sourceStats(DownlinkSource src) const {
  const uint8_t si = idx(src);
  if (si >= kSourceCount) {
    return SourceStats{};
  }
  return m_stats[si];
}

uint32_t LinkLossView::gapSpans(const GapSpan** out) const {
  if (out) {
    *out = m_gaps;
  }
  return m_gapFilled;
}

bool LinkLossView::findGap(uint16_t apid, uint32_t seqFirstLost, GapSpan* out) const {
  if (m_gapFilled == 0) {
    return false;
  }
  const uint32_t n = m_gapFilled;
  // Walk from oldest to newest in the ring.
  const uint32_t start =
      (m_gapFilled < kMaxGapSpans) ? 0u : m_gapHead;
  for (uint32_t i = 0; i < n; ++i) {
    const GapSpan& g = m_gaps[(start + i) % kMaxGapSpans];
    if (g.apid == apid && g.seqFirstLost == seqFirstLost) {
      if (out) {
        *out = g;
      }
      return true;
    }
  }
  return false;
}

double LinkLossView::apidRate(DownlinkSource src, uint16_t apid) const {
  const uint8_t si = idx(src);
  if (si >= kSourceCount) {
    return 0.0;
  }
  for (uint8_t i = 0; i < kMaxApids; ++i) {
    if (m_ap[si][i].used && m_ap[si][i].apid == apid) {
      return m_ap[si][i].rate;
    }
  }
  return 0.0;
}

}  // namespace GroundLinkView
