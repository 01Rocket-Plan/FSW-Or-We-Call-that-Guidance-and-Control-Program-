module Components {

  @ 箭體 → 發射架 臍帶狀態（DsMcu）；Base ID 0x8500
  @ 有線臍帶 UART，單向；箭上不從臍帶收東西（點火只走地面 → 架，gate C15）。
  @ 起飛後臍帶拔脫是預期行為：寫入失敗只計數，不發 warning、不進錯誤狀態。
  @ 邏輯在 UmbilicalTxCore（host 可測）。
  passive component UmbilicalTx {

    @ 10 Hz 以上的速率群組（內部節流到 period_s）
    sync input port schedIn: Svc.Sched

    @ 飛行模式（只讀；FlightMode 邏輯由 Edward 撰寫）
    sync input port modeIn: RocketTypes.ModePort

    @ BIT 旗標＋電池電壓
    sync input port bitIn: Components.RocketBitPort

    @ 位元組 → 臍帶 UART 驅動
    output port uartOut: Fw.BufferSend

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

    telemetry SentFrames: U32
    telemetry WriteFailures: U32
    telemetry Ready: bool
  }

}
