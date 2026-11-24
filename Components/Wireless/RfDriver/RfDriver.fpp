module Components {

  @ 射頻硬體型號 — 主飛行鏈路：LoRa Ra-02（433 MHz），晶片 Semtech SX1278
  @ 備註（非本 enum 取值／本 PR 不實作）：HC-12（SI4438 433 MHz）＝備援鏈；
  @ NRF24L01（2.4 GHz）＝地面站／發射台用，不上箭（Edward 設計文件）
  enum RfHwModel : U8 { LORA_RA_02 = 0 }

  @ 射頻角色：AIR_GROUND_LINK＝箭地無線鏈路（主飛行鏈）
  enum RfHwRole : U8 { AIR_GROUND_LINK = 0 }

  @ SX1278 RegModemConfig1.Bw 編碼（Semtech datasheet §6.4）；數值＝暫存器欄位值
  enum RfLoraBw : U8 {
    BW_7_8K = 0
    BW_10_4K = 1
    BW_15_6K = 2
    BW_20_8K = 3
    BW_31_25K = 4
    BW_41_7K = 5
    BW_62_5K = 6
    BW_125K = 7
    BW_250K = 8
    BW_500K = 9
  }

  @ RF 數傳真驅動；Base ID 0x8400（首席已核准）
  @ 硬體型號 LORA_RA_02（LoRa Ra-02，433 MHz，Semtech SX1278，主飛行鏈）；角色＝AIR_GROUND_LINK
  @ 握手 OFF→INIT→SELFTEST→READY／FAULT；連續 txTimeout → FAULT
  passive component RfDriver {

    sync input port dataIn: Svc.ComDataWithContext
    output port dataOut: Svc.ComDataWithContext
    @ F´ v4：通訊狀態埠是 Fw.SuccessCondition（舊 stub 的 Svc.ComStatus 在真 F´ 不存在）
    output port comStatusOut: Fw.SuccessCondition
    output port healthOut: RfHealth
    sync input port run: Svc.Sched

    time get port timeCaller
    param get port prmGetOut
    param set port prmSetOut
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 硬體型號 — LoRa Ra-02（433 MHz），晶片 Semtech SX1278，主飛行鏈
    param model: RfHwModel default RfHwModel.LORA_RA_02
    @ 角色 — 箭地無線鏈路
    param role: RfHwRole default RfHwRole.AIR_GROUND_LINK

    param channel: U32 default 0
    param power_dBm: I32 default 0
    @ TX 逾時：這一包空中時間之外再多等的秒數（issue #60）
    param txTimeout_s: F64 default 1.0
    param initDeadline_s: F64 default 2.0
    param selftestDeadline_s: F64 default 2.0
    param readyDeadline_s: F64 default 5.0
    param txTimeoutLimit: U32 default 3

    @ ---- LoRa SX1278 數據機（issue #60）；新 prm 加在最後，舊 prm ID 不變 ----
    @ SF7／125 kHz／4:5 已由 Edward 定案；前導碼／CRC／標頭模式仍待 Edward 定。
    @ 注意：頻段仍在評估（433 MHz 法規問題），定案後可能換頻段／硬體
    @ SpreadingFactor 6..12（SF6 必須隱式標頭）；定案 SF7（Edward）
    param loraSf: U8 default 7
    @ 頻寬；定案 125 kHz（Edward）
    param loraBw: RfLoraBw default RfLoraBw.BW_125K
    @ CodingRate 1..4 ＝ 4/5..4/8；定案 4/5（Edward）
    param loraCr: U8 default 1
    @ 前導碼符號數；待 Edward 定
    param loraPreambleLen: U16 default 8
    @ 酬載 CRC；待 Edward 定
    param loraCrcOn: bool default true
    @ 隱式標頭（false＝顯式）；待 Edward 定
    param loraImplicitHeader: bool default false
    @ LowDataRateOptimize：0 關、1 開、2 自動（Tsym > 16 ms 開，Semtech §4.1.1.6）
    param loraLdro: U8 default 2
    @ 一次 run tick 的 µs；預設 10000（DsMcu 目前接在 rg100Hz）；正式 rate group 待定
    param tickPeriod_us: U32 default 10000

    telemetry State: RocketTypes.DrvState
    telemetry TxTimeoutCount: U32
    telemetry TxDroppedCount: U32
    telemetry SentFrames: U32
    telemetry Model: RfHwModel
    telemetry Role: RfHwRole

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "RfDriver {} -> {}"
    event RfFault(code: U32) severity warning high format "RfDriver FAULT code={}"
    event RfTxTimeout(len: U32) severity warning low format "RfDriver TX timeout len={}"
    event InitOk() severity activity low format "RfDriver INIT OK"
    event SelftestOk() severity activity low format "RfDriver SELFTEST OK"
    event SelftestFail(tries: U32) severity warning high format "RfDriver SELFTEST fail tries={}"
  }

}
