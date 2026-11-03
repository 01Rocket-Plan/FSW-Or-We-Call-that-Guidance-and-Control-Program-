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
]
req_ev = ["QueueFullDrop", "SdError", "EnterReady", "EnterFault", "SeqGap", "TimeNonMonotonic"]

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

for ev in ("QueueFullDrop", "SeqGap", "TimeNonMonotonic"):
    chunk = fpp.split(ev, 1)[1][:240]
    if "warning high" not in chunk:
        errors.append(f"{ev} severity not warning high")

if "SeqGap(expected: U32, got: U32)" not in fpp:
    errors.append("SeqGap args must be expected:U32, got:U32")
if "TimeNonMonotonic(prev_s: F64, now_s: F64)" not in fpp:
    errors.append("TimeNonMonotonic args must be prev_s:F64, now_s:F64")

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

if errors:
    print("FAIL")
    for e in errors:
        print(" -", e)
    sys.exit(1)
print("PASS: SdLogger dict self-test OK")
print("TLM:", ", ".join(f"0x{0x2000+i:04x}:{n}" for i, n in enumerate(req_tlm)))
print("EV :", ", ".join(f"0x{0x2000+i:04x}:{n}" for i, n in enumerate(req_ev)))
