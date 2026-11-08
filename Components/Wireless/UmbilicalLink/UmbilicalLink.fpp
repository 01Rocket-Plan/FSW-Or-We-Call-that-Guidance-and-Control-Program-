module Components {

  @ 架 → 箭 狀態（PadToRocket；旗標位元見 PadLinkCodec.hpp 的 PadToRocketFlag）
  @ 方案 A：只是狀態回報，不是放行指令；鬆夾持由架決定，箭用 READY 否決
  port PadToRocketPort(
    padState: Components.PadState
    countdown_s: F32
    flags: U8
    linkOk: bool
  )

  @ 箭體 ↔ 發射架 臍帶鏈路（DsMcu）；Base ID 0x8500
  @ 有線臍帶 UART，雙向：箭 → 架 送 RocketStatus（模式、BIT、ready、電池電壓），
  @ 架 → 箭 收 PadToRocket（架狀態、倒數、點火／推力確認／夾持旗標）。
  @ 點火只走地面 → 架，火箭不在點火迴路裡（gate C15）。
  @ 起飛後臍帶拔脫是預期行為：寫入失敗、收不到架狀態都只計數，不發 warning、不進錯誤狀態。
  @ 邏輯在 UmbilicalLinkCore（host 可測）。
  passive component UmbilicalLink {

    @ 10 Hz 以上的速率群組（內部節流到 period_s）
    sync input port schedIn: Svc.Sched

    @ 飛行模式（只讀；FlightMode 邏輯由 Edward 撰寫）
    sync input port modeIn: RocketTypes.ModePort

    @ BIT 旗標＋電池電壓
    sync input port bitIn: Components.RocketBitPort

    @ 位元組 → 臍帶 UART 驅動
    output port uartOut: Fw.BufferSend

    @ 臍帶 UART 驅動收到的位元組（可分段）
    sync input port uartIn: Fw.BufferSend

    @ 架狀態 → FlightMode（只供參考；要不要用由 FlightMode 決定）
    output port padStatusOut: Components.PadToRocketPort

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 送出週期
    param period_s: F64 default 0.1
    @ BIT 資料年齡上限，超過就報 ready=false
    param bitMaxAge_s: F64 default 0.5
    @ 多久沒收到架狀態 → PadLinkOk=false【待 Edward 定】
    param padLinkTimeout_s: F64 default 0.5

    telemetry SentFrames: U32
    telemetry WriteFailures: U32
    telemetry Ready: bool
    telemetry PadLinkOk: bool
    telemetry PadState: Components.PadState
    telemetry RxFrames: U32
    telemetry RxCrcErrors: U32
  }

}
