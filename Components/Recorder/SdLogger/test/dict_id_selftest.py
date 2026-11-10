#!/usr/bin/env python3
"""Lightweight self-test: SdLogger dictionary names + documented absolute IDs."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
fpp = (ROOT / "SdLogger.fpp").read_text(encoding="utf-8")
doc = (ROOT.parent / "TELEMETRY_EVENT_DICTIONARY.md").read_text(encoding="utf-8")

req_tlm = [
    "WriteCount", "DropCount", "RecordQueueHwm", "SdWritePoolHwm",
    "SdErrorCount", "SdStatus", "LastSeq", "LastT_s",
    "HsState", "LoggerSeq", "ProducerSeq", "CardModel", "CardRole",
    "ImageChunkCount", "ImageDropCount", "ImageFrameCount", "ImageBytesWritten",
    "ImageQueueHwm", "LastImageFrame", "LastImageChunk",
]
req_ev = [
    "QueueFullDrop", "SdError", "EnterReady", "EnterFault", "SeqGap", "TimeNonMonotonic",
    "ImageChunkDrop", "ImageFrameGap", "ImageChunkGap", "ImageTruncatedSeen",
]

errors = []
for n in req_tlm:
    if not re.search(rf"telemetry\s+{n}\b", fpp):
        errors.append(f"missing telemetry {n}")
for n in req_ev:
    if not re.search(rf"event\s+{n}\b", fpp):
        errors.append(f"missing event {n}")

for i, name in enumerate(req_tlm):
    abs_id = 0x2000 + i
    needle = f"**0x{abs_id:04x}**"
    needle_u = f"**0x{abs_id:04X}**"
    if needle not in doc and needle_u not in doc:
        errors.append(f"doc missing abs id {needle} for {name}")

for ev in (
    "QueueFullDrop", "SeqGap", "TimeNonMonotonic",
    "ImageChunkDrop", "ImageFrameGap", "ImageChunkGap", "ImageTruncatedSeen",
):
    chunk = fpp.split(ev, 1)[1][:280]
    if "warning high" not in chunk:
        errors.append(f"{ev} severity not warning high")

if "SeqGap(expected: U32, got: U32)" not in fpp:
    errors.append("SeqGap args must be expected:U32, got:U32")
if "TimeNonMonotonic(prev_s: F64, now_s: F64)" not in fpp:
    errors.append("TimeNonMonotonic args must be prev_s:F64, now_s:F64")
if "ImageChunkDrop(dropped: U32, queueDepth: U32)" not in fpp:
    errors.append("ImageChunkDrop args must be dropped:U32, queueDepth:U32")
if "ImageFrameGap(expected: U32, got: U32)" not in fpp:
    errors.append("ImageFrameGap args must be expected:U32, got:U32")
if "ImageChunkGap(frameNumber: U32, expected: U32, got: U32)" not in fpp:
    errors.append("ImageChunkGap args must be frameNumber:U32, expected:U32, got:U32")
if "ImageTruncatedSeen(frameNumber: U32, chunkIndex: U32)" not in fpp:
    errors.append("ImageTruncatedSeen args must be frameNumber:U32, chunkIndex:U32")

for abs_id, name in ((0x2004, "SeqGap"), (0x2005, "TimeNonMonotonic")):
    needle_id = f"**0x{abs_id:04x}**"
    if needle_id not in doc or f"`{name}`" not in doc:
        errors.append(f"doc missing {needle_id} / {name}")

if "SdCardStatus" not in fpp:
    errors.append("missing SdCardStatus enum")
if "MICRO_SDHC_UHS_I_16GB" not in fpp:
    errors.append("missing SdCardModel MICRO_SDHC_UHS_I_16GB")
if "SdCardRole" not in fpp or "DS_LOG" not in fpp or "US_LOG" not in fpp:
    errors.append("missing SdCardRole DS_LOG/US_LOG")
if "passive component SdLoggerSim" not in fpp:
    errors.append("missing SdLoggerSim component")
if "usImageIn" not in fpp or "dsImageIn" not in fpp:
    errors.append("missing usImageIn/dsImageIn ports")
if "IMAGE_QUEUE_DEPTH" not in fpp:
    errors.append("missing IMAGE_QUEUE_DEPTH param")
if fpp.count("CamImageOut") < 4:  # SdLogger＋SdLoggerSim 各 us／ds 兩個
    errors.append("missing camera image port type CamImageOut on SdLogger")

for abs_id, name in (
    (0x2006, "ImageChunkDrop"),
    (0x2007, "ImageFrameGap"),
    (0x2008, "ImageChunkGap"),
    (0x2009, "ImageTruncatedSeen"),
    (0x200D, "ImageChunkCount"),
    (0x2013, "LastImageChunk"),
):
    needle_l = f"**0x{abs_id:04x}**"
    needle_u = f"**0x{abs_id:04X}**"
    if (needle_l not in doc and needle_u not in doc) or f"`{name}`" not in doc:
        errors.append(f"doc missing {needle_u} / {name}")


# Bot2 Core+Sim tlm 0x2008–0x200C must appear in doc
for abs_id, name in (
    (0x2008, "HsState"),
    (0x2009, "LoggerSeq"),
    (0x200A, "ProducerSeq"),
    (0x200B, "CardModel"),
    (0x200C, "CardRole"),
):
    needle_l = f"**0x{abs_id:04x}**"
    needle_u = f"**0x{abs_id:04X}**"
    if (needle_l not in doc and needle_u not in doc) or f"`{name}`" not in doc:
        errors.append(f"doc missing Bot2 tlm {needle_u} / {name}")

# Image tlm block 0x200D–0x2013
for abs_id, name in (
    (0x200D, "ImageChunkCount"),
    (0x200E, "ImageDropCount"),
    (0x200F, "ImageFrameCount"),
    (0x2010, "ImageBytesWritten"),
    (0x2011, "ImageQueueHwm"),
    (0x2012, "LastImageFrame"),
    (0x2013, "LastImageChunk"),
):
    needle_l = f"**0x{abs_id:04x}**"
    needle_u = f"**0x{abs_id:04X}**"
    if (needle_l not in doc and needle_u not in doc) or f"`{name}`" not in doc:
        errors.append(f"doc missing image tlm {needle_u} / {name}")

if errors:
    print("FAIL")
    for e in errors:
        print(" -", e)
    sys.exit(1)
print("PASS: SdLogger dict self-test OK")
print("TLM:", ", ".join(f"0x{0x2000+i:04x}:{n}" for i, n in enumerate(req_tlm)))
print("EV :", ", ".join(f"0x{0x2000+i:04x}:{n}" for i, n in enumerate(req_ev)))
