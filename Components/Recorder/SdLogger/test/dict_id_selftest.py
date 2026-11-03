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
req_ev = ["QueueFullDrop", "SdError", "EnterReady", "EnterFault"]

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

# severity on QueueFullDrop
if "warning high" not in fpp.split("QueueFullDrop", 1)[1][:200]:
    errors.append("QueueFullDrop severity not warning high")

if "SdCardStatus" not in fpp:
    errors.append("missing SdCardStatus enum")

if errors:
    print("FAIL")
    for e in errors:
        print(" -", e)
    sys.exit(1)
print("PASS: SdLogger dict self-test OK")
print("TLM:", ", ".join(f"0x{0x2000+i:04x}:{n}" for i,n in enumerate(req_tlm)))
print("EV :", ", ".join(req_ev))
