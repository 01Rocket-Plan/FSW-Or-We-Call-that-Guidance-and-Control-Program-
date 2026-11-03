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
]
req_ev = ["QueueFullDrop", "SdError", "EnterReady", "EnterFault", "SeqGap", "TimeNonMonotonic"]

errors = []
for n in req_tlm:
    if not re.search(rf"telemetry\s+{n}\b", fpp):
        errors.append(f"missing telemetry {n}")
for n in req_ev:
    if not re.search(rf"event\s+{n}\b", fpp):
        errors.append(f"missing event {n}")

# Documented absolute IDs for tlm 0x2000..0x2007
for i, name in enumerate(req_tlm):
    abs_id = 0x2000 + i
    pat = rf"\*\*{abs_id:#06x}\*\*.*`{name}`" if False else None
    if f"**{abs_id:#06x}**" not in doc.replace("0x2000", "0x2000") and f"**0x{abs_id:04X}**" not in doc:
        # accept either 0x2000 or 0x2000 style in table
        if f"**0x{abs_id:04x}**" not in doc and f"**0x{abs_id:04X}**" not in doc:
            # our doc uses **0x2000** lowercase hex
            needle = f"**0x{abs_id:04x}**"
            if needle not in doc:
                errors.append(f"doc missing abs id {needle} for {name}")

# severity on QueueFullDrop / SeqGap / TimeNonMonotonic
for ev in ("QueueFullDrop", "SeqGap", "TimeNonMonotonic"):
    chunk = fpp.split(ev, 1)[1][:240]
    if "warning high" not in chunk:
        errors.append(f"{ev} severity not warning high")

# SeqGap / TimeNonMonotonic arg shapes
if "SeqGap(expected: U32, got: U32)" not in fpp:
    errors.append("SeqGap args must be expected:U32, got:U32")
if "TimeNonMonotonic(prev_s: F64, now_s: F64)" not in fpp:
    errors.append("TimeNonMonotonic args must be prev_s:F64, now_s:F64")

# Documented abs IDs for new events
for abs_id, name in ((0x2004, "SeqGap"), (0x2005, "TimeNonMonotonic")):
    needle_id = f"**0x{abs_id:04x}**"
    if needle_id not in doc or f"`{name}`" not in doc:
        errors.append(f"doc missing {needle_id} / {name}")

if "SdCardStatus" not in fpp:
    errors.append("missing SdCardStatus enum")

if errors:
    print("FAIL")
    for e in errors:
        print(" -", e)
    sys.exit(1)
print("PASS: SdLogger dict self-test OK")
print("TLM:", ", ".join(f"0x{0x2000+i:04x}:{n}" for i,n in enumerate(req_tlm)))
print("EV :", ", ".join(f"0x{0x2000+i:04x}:{n}" for i,n in enumerate(req_ev)))
