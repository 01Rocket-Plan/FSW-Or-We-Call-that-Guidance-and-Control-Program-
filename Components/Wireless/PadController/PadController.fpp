module Components {

  @ 發射架主控（PadMcu，ESP32）；Base ID 0x8700
  @ 臍帶 UART 收箭體狀態（箭 → 架）並回送架上狀態（架 → 箭）、NRF24 收地面指令／送架上狀態（架 ↔ 地面）、
  @ 發射許可互鎖、待點火倒數、點火窗、推力確認後鬆夾持、離軌判斷。
  @ 邏輯全部在 PadControllerCore（host 可測）；本元件只是 F´ 外殼。
  @ 點火只走地面 → 架，火箭不在點火迴路裡（gate C15）；實體鑰匙在硬體串聯。
  passive component PadController {

    @ 1 kHz 速率群組（推力確認／鬆夾持時序以 1 ms 一拍計）；radioOut／umbilicalOut 由外殼降頻到 10 Hz
    sync input port schedIn: Svc.Sched

    @ 臍帶 UART 收到的位元組（可分段）
    sync input port umbilicalIn: Fw.BufferSend

    @ NRF24 收到的一包（一包＝一幀）
    sync input port radioIn: Fw.BufferSend

    @ 架上狀態幀 → NRF24 驅動（10 Hz）
    output port radioOut: Fw.BufferSend

    @ 架 → 箭 狀態幀（PadToRocket）→ 臍帶 UART 驅動（10 Hz）
    @ 方案 A：只是狀態回報；鬆夾持由架決定，箭只能用 READY 否決
    output port umbilicalOut: Fw.BufferSend

    @ 讀架上硬體
    output port ioGet: Components.PadIoGet

    @ 點火繼電器／夾持命令
    output port actOut: Components.PadActPort

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 待點火倒數（E1：約 30 s）
    param countdown_s: F64 default 30.0
    @ 點火窗：點火器通電後多久內要出現「第一個推力樣本」（點火延遲；C15：塔上 2 秒）。不是推力確認：推力確認從第一個推力樣本起算 thrustConfirm_s＝0.02 s
    param ignitionWindow_s: F64 default 2.0
    @ 推力訊號持續多久才算確認。2026-11-10 改 0.02 s（#42 C2；74F85 燃燒 0.873 s）。1 kHz：t0 … t0+20 ms 連 21 個樣本有推力 → t0+20 ms 確認
    param thrustConfirm_s: F64 default 0.02
    @ 推力確認後至少再等多久才發鬆夾持命令（1e-3 s＝一拍）；這段時間推力一消失 → HOLD_ABORT／THRUST_LOST、不鬆
    param releaseMargin_s: F64 default 0.001
    @ 鬆夾持命令到夾持真正打開的時間（假設值，HIL 實測）；只用來檢查 thrustConfirm_s＋releaseMargin_s＋clampActuatorLatency_s ≤ releaseDeadline_s
    param clampActuatorLatency_s: F64 default 0.02
    @ 鬆夾持後多久沒離軌 → NO_LIFTOFF。2026-11-10 改 1.0 s（74F85 燃燒 0.873 s）
    param liftoffTimeout_s: F64 default 1.0
    @ 鬆夾持期限：第一次看到推力後超過此時間還沒鬆 → 不鬆、繼續夾住（HOLD_ABORT／LATE_RELEASE）。定為 0.05 s
    param releaseDeadline_s: F64 default 0.05
    @ 離軌備援：荷重元讀值低於此值（N，假設值）
    param liftoffLoadMin_N: F64 default 0.5
    @ 離軌備援：荷重元低於 liftoffLoadMin_N 要持續多久才算離軌（假設值）
    param liftoffLoadHold_s: F64 default 0.05
    @ 下令鬆夾持後多久內要讀到夾持開到位，否則 NO_LIFTOFF／CLAMP_STUCK（假設值；≤ 0＝沒裝開到位開關、不檢查）
    param clampOpenTimeout_s: F64 default 0.1
    @ 多久沒收到簽名幀 → 地面鏈路紅
    param groundLinkTimeout_s: F64 default 1.0
    @ 多久沒收到箭體狀態 → 臍帶鏈路紅
    param umbilicalTimeout_s: F64 default 0.5
    @ 架傾角上限（E6：離垂線 5° ＝ 仰角 85°）
    param maxTilt_rad: F64 default 0.0872665
    @ 架側電源下限
    param padVmin_V: F64 default 11.0

    telemetry State: Components.PadState
    telemetry LastReason: Components.PadReason
    telemetry Interlocks: U16
    telemetry CountdownRemaining_s: F32
    telemetry RocketMode: U8
    telemetry RocketBit: U16
    telemetry UmbilicalCrcErrors: U32
    telemetry GroundRejects: U32

    event PadStateChange(fromState: Components.PadState, toState: Components.PadState, reason: Components.PadReason) \
      severity activity high \
      format "Pad {} -> {} reason={}"

    event PadCommand(op: Components.PadCmdOp, result: Components.PadCmdResult, reason: Components.PadReason) \
      severity activity low \
      format "Pad cmd {} result={} reason={}"

    event PadHangFire() \
      severity warning high \
      format "Pad HANG_FIRE: no thrust in ignition window; clamp held, igniter off, no retry"

    event PadNoLiftoff() \
      severity warning high \
      format "Pad NO_LIFTOFF: clamp released but rocket still on rail; keep clear"

    event PadHoldAbort(reason: Components.PadReason) \
      severity warning high \
      format "Pad HOLD_ABORT reason={}: clamp held, igniter off"

    event PadClampStuck() \
      severity warning high \
      format "Pad CLAMP_STUCK: release commanded but clamp-open switch not seen; igniter off, keep clear"

    event PadBadParams(thrustConfirm_s: F64, releaseMargin_s: F64, clampActuatorLatency_s: F64, releaseDeadline_s: F64) \
      severity warning high \
      format "Pad params rejected (confirm {} + margin {} + clamp latency {} must be <= deadline {}, all finite and >= 0); ARM refused"
  }

}
