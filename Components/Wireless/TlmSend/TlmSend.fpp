module Components {

  @ tlmSend：遙測取樣與邏輯封包排程；Base ID 0x8000；rg100Hz
  @ P1 預設 10 Hz（rateDiv=10）。不做檔案紀錄、不含任務門檻。
  passive component TlmSend {

    sync input port run: Svc.Sched
    output port pktOut: Svc.ComDataWithContext

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 預設 APID 速率（tick 間隔）；P1＝10 @ rg100Hz
    param APID_RATE_DIV_P1: U32 default 10
    param APID_RATE_DIV_P2: U32 default 10
    param APID_RATE_DIV_P3: U32 default 100
    param APID_RATE_DIV_P4: U32 default 200

    sync command SET_ENABLED(apid: U16, enabled: U8)

    telemetry PktsSent: U32
    telemetry TimeNonMonoCount: U32

    event PktSent(apid: U16, seq: U32) \
      severity activity low \
      format "tlmSend apid={} seq={}"

    @ EVT_TIME_NONMONOTONIC：t_s 倒退仍照常送出
    event TimeNonMonotonic(apid: U16, prev_s: F64, now_s: F64) \
      severity warning low \
      format "tlmSend nonmonotonic apid={} prev={} now={}"
  }

}
