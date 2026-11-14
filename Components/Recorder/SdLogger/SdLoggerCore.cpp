#include "Components/Recorder/SdLogger/SdLoggerCore.hpp"
#include <cstring>

namespace Components {

SdLoggerCore::SdLoggerCore(SdCardCore& card, ISdLoggerSink& sink) : m_card(card), m_sink(sink) {}

uint32_t SdLoggerCore::setQueueDepth(uint32_t d) {
  if (d == 0) d = 1;
  if (d > kMaxRecordQueueDepth) {
    m_sink.onQueueDepthClamped(0, d, kMaxRecordQueueDepth);
    d = kMaxRecordQueueDepth;
  }
  m_queueDepth = d;
  return d;
}

uint32_t SdLoggerCore::setImageQueueDepth(uint32_t d) {
  if (d == 0) d = 1;
  if (d > kMaxImageQueueDepth) {
    m_sink.onQueueDepthClamped(1, d, kMaxImageQueueDepth);
    d = kMaxImageQueueDepth;
  }
  m_imageQueueDepth = d;
  return d;
}

void SdLoggerCore::setState(LoggerDrvState s, uint8_t faultReason) {
  if (m_state == s) return;
  m_state = s;
  if (s == LoggerDrvState::READY) m_sink.onEnterReady();
  if (s == LoggerDrvState::FAULT) m_sink.onEnterFault(faultReason);
}

bool SdLoggerCore::start() {
  setState(LoggerDrvState::INIT);
  setState(LoggerDrvState::SELFTEST);
  if (!m_card.init()) {
    m_sink.onSdError(m_card.status(), 1);
    setState(LoggerDrvState::FAULT, 1);
    return false;
  }
  m_running = true;
  setState(LoggerDrvState::READY);
  return true;
}

bool SdLoggerCore::checkSeq(uint16_t channelId, uint32_t producerSeq) {
  const size_t idx = channelId < 32 ? channelId : 0;
  if (!m_haveSeq[idx]) {
    m_haveSeq[idx] = true;
    m_expectSeq[idx] = producerSeq + 1;
    return true;
  }
  const uint32_t expected = m_expectSeq[idx];
  if (producerSeq != expected) {
    m_sink.onSeqGap(expected, producerSeq);
    m_expectSeq[idx] = producerSeq + 1;
    return true;
  }
  m_expectSeq[idx] = producerSeq + 1;
  return true;
}

bool SdLoggerCore::checkTime(double t_s) {
  if (m_haveTime && t_s < m_lastT_s) {
    m_sink.onTimeNonMonotonic(m_lastT_s, t_s);
  }
  m_haveTime = true;
  m_lastT_s = t_s;
  return true;
}

void SdLoggerCore::checkImageGaps(uint32_t frameNumber, uint32_t chunkIndex, uint8_t flags) {
  if (!m_haveImage) {
    m_haveImage = true;
    m_curFrame = frameNumber;
    m_expectFrame = frameNumber;  // advanced only on END
    if (chunkIndex != 0) {
      m_sink.onImageChunkGap(frameNumber, 0, chunkIndex);
    }
    m_expectChunk = chunkIndex + 1;
  } else if (frameNumber != m_curFrame) {
    // If prior frame never saw END, natural next is curFrame+1
    const uint32_t expected =
        (m_expectFrame > m_curFrame) ? m_expectFrame : (m_curFrame + 1);
    if (frameNumber != expected) {
      m_sink.onImageFrameGap(expected, frameNumber);
    }
    m_curFrame = frameNumber;
    if (chunkIndex != 0) {
      m_sink.onImageChunkGap(frameNumber, 0, chunkIndex);
    }
    m_expectChunk = chunkIndex + 1;
  } else {
    if (chunkIndex != m_expectChunk) {
      m_sink.onImageChunkGap(frameNumber, m_expectChunk, chunkIndex);
    }
    m_expectChunk = chunkIndex + 1;
  }

  if (flags & kCamChunkFlagTruncated) {
    m_sink.onImageTruncatedSeen(frameNumber, chunkIndex);
  }
  if (flags & kCamChunkFlagEnd) {
    ++m_imageFrameCount;
    m_expectFrame = frameNumber + 1;
  }
}

bool SdLoggerCore::ingest(const PackedRecord& rec) {
  if (!m_running || m_state != LoggerDrvState::READY) return false;
  checkSeq(rec.channelId, rec.producerSeq);
  checkTime(rec.t_s);
  m_lastProducerSeq = rec.producerSeq;

  // 深度參數可能在佇列裡已有資料時被調小：size 會暫時大於 depth，照樣丟最新的，等 flush 消化。
  if (m_q.size() >= m_queueDepth || !m_q.push(rec)) {
    ++m_drops;
    m_sink.onQueueFullDrop(m_drops, m_q.size());
    return false;
  }
  if (m_q.size() > m_queueHwm) m_queueHwm = m_q.size();
  return true;
}

bool SdLoggerCore::ingestImageChunk(uint32_t frameNumber, uint32_t chunkIndex, uint8_t flags,
                                    const uint8_t* data, uint16_t len, double t_s) {
  if (!m_running || m_state != LoggerDrvState::READY) return false;
  if (len > sizeof(ImageChunkRecord::bytes)) return false;

  checkTime(t_s);
  checkImageGaps(frameNumber, chunkIndex, flags);
  m_lastImageFrame = frameNumber;
  m_lastImageChunk = chunkIndex;
  m_lastProducerSeq = frameNumber;

  ImageChunkRecord* rec = (m_iq.size() >= m_imageQueueDepth) ? nullptr : m_iq.backSlot();
  if (rec == nullptr) {
    ++m_imageDrops;
    m_sink.onImageChunkDrop(m_imageDrops, m_iq.size());
    return false;
  }
  // 直接寫進環形緩衝那一格（不在堆疊上另組一份 2 KB）。
  rec->frameNumber = frameNumber;
  rec->chunkIndex = chunkIndex;
  rec->flags = flags;
  rec->len = len;
  rec->t_s = t_s;
  // 這一格可能還留著上一輪的資料：有 data 就複製，沒有就清 0（和原本新建一筆、bytes 全 0 的行為一樣）。
  if (len && data) {
    std::memcpy(rec->bytes, data, len);
  } else if (len) {
    std::memset(rec->bytes, 0, len);
  }
  m_iq.commitBack();
  if (m_iq.size() > m_imageQueueHwm) m_imageQueueHwm = m_iq.size();
  return true;
}

void SdLoggerCore::flush() {
  while (!m_q.empty()) {
    const PackedRecord& rec = m_q.front();
    SdRecordHeader hdr;
    hdr.channelId = rec.channelId;
    hdr.loggerSeq = m_loggerSeq;
    hdr.producerSeq = rec.producerSeq;
    hdr.t_s = rec.t_s;
    if (!m_card.writeRecord(hdr, rec.bytes, rec.len)) {
      m_sink.onSdError(m_card.status(), 2);
      if (m_card.status() == SdMediaStatus::CARD_MISSING || m_card.status() == SdMediaStatus::IO_ERROR) {
        setState(LoggerDrvState::FAULT, 2);
      }
      break;
    }
    ++m_loggerSeq;
    m_q.pop();
  }

  while (!m_iq.empty()) {
    const ImageChunkRecord& rec = m_iq.front();
    // payload: chunkIndex + flags + pad3 + data
    uint8_t payload[8 + 2048];
    std::memcpy(payload + 0, &rec.chunkIndex, 4);
    payload[4] = rec.flags;
    payload[5] = payload[6] = payload[7] = 0;
    if (rec.len) std::memcpy(payload + 8, rec.bytes, rec.len);
    const uint16_t payloadLen = static_cast<uint16_t>(8 + rec.len);

    SdRecordHeader hdr;
    hdr.channelId = static_cast<uint16_t>(RecordChannel::IMAGE);
    hdr.loggerSeq = m_loggerSeq;
    hdr.producerSeq = rec.frameNumber;
    hdr.t_s = rec.t_s;
    if (!m_card.writeRecord(hdr, payload, payloadLen)) {
      m_sink.onSdError(m_card.status(), 3);
      if (m_card.status() == SdMediaStatus::CARD_MISSING || m_card.status() == SdMediaStatus::IO_ERROR) {
        setState(LoggerDrvState::FAULT, 3);
      }
      break;
    }
    ++m_loggerSeq;
    ++m_imageChunkCount;
    m_imageBytesWritten += rec.len;
    m_iq.pop();
  }

  (void)m_card.sync();
}

}  // namespace Components
