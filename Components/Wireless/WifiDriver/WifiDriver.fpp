module Components {

  @ WiFi 硬體型號。主線初代 ESP32（ESP32 DevKit U1，內建 2.4 GHz WiFi）；
  @ 備用 STM32WB55CGU6 沒有 WiFi，用 UNSUPPORTED（init 失敗 → FAULT）。
  @ 數值固定：UNSUPPORTED＝0、ESP32_INTERNAL_WIFI＝1（不可改既有值）
  enum WifiHwModel : U8 { UNSUPPORTED = 0, ESP32_INTERNAL_WIFI = 1 }

  @ WiFi 角色，只是標示與遙測，不改變幀內容。
  @ AIR_GROUND_LINK＝箭地備用鏈路；PAD_GROUND_LINK＝發射架↔地面備用鏈路。
  enum WifiHwRole : U8 { AIR_GROUND_LINK = 0, PAD_GROUND_LINK = 1 }

  @ WiFi 工作模式。預設 STA（連到外部 AP）。SOFT_AP 供沒有路由器的桌上測試。
  enum WifiMode : U8 { STA = 0, SOFT_AP = 1 }

  @ WiFi 近距離備用傳輸驅動（LoRa RfDriver 的備用；主飛行鏈仍為 LoRa）；Base ID 0x8900
  @ 埠集合與 RfDriver／BleDriver 完全相同 → 拓樸可直接替換實例；上層 Framer／ComQueue／既有上行鑑權不變
  @ 狀態機沿用 RfDriverCore（OFF→INIT→SELFTEST→READY／FAULT），UDP 送收在 WifiLinkCore
  @ 硬體存取經 IWifiHal（平台用 ESP-IDF esp_wifi＋lwIP UDP 實作；未接 HAL 時 INIT 失敗 → FAULT）
  passive component WifiDriver {

    sync input port dataIn: Svc.ComDataWithContext
    output port dataOut: Svc.ComDataWithContext
    @ F´ v4：通訊狀態埠是 Fw.SuccessCondition
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

    @ 硬體型號 — 預設主線 ESP32 內建 WiFi
    param model: WifiHwModel default WifiHwModel.ESP32_INTERNAL_WIFI
    @ 角色 — 預設箭地備用鏈路
    param role: WifiHwRole default WifiHwRole.AIR_GROUND_LINK
    @ 工作模式 — 預設 STA
    param mode: WifiMode default WifiMode.STA

    @ 2.4 GHz 頻道。0＝交給堆疊／AP；SoftAP 模式強制使用 1–13（非法值會夾住）
    param channel: U8 default 0
    @ 發射功率 dBm；依 ESP32 WiFi 能力夾取到 8–20。HAL 可視板級限制再夾。
    param txPower_dBm: I32 default 20
    @ 本機 UDP 埠（預設 4242，可改）
    param localPort: U16 default 4242
    @ 對端 IPv4，以網路位元序 U32 存放（位元 31–24＝第一段；例如 127.0.0.1＝0x7F000001）
    param peerIp: U32 default 0
    @ 對端 UDP 埠（預設 4242，可改）
    param peerPort: U16 default 4242
    @ 為真時，從最近一次成功收到的來源更新對端（方便測試）
    param learnPeerFromRx: bool default false

    @ 以下與 RfDriver／BleDriver 同義（給 RfDriverCore）
    @ TX 逾時：預期送完之後再多等的秒數。WiFi 由 WifiLinkCore 回報送完、
    @ 不套 LoRa 空中時間，預期送完時間視為 0 → 送出後 txTimeout_s 內沒被 HAL 收下就算逾時
    param txTimeout_s: F64 default 1.0
    param initDeadline_s: F64 default 2.0
    param selftestDeadline_s: F64 default 2.0
    param readyDeadline_s: F64 default 5.0
    param txTimeoutLimit: U32 default 3
    @ FAULT 後隔多久自動重新 INIT（s）；≤0 不自動重來
    param faultRetry_s: F64 default 5.0

    telemetry State: RocketTypes.DrvState
    telemetry TxTimeoutCount: U32
    telemetry TxDroppedCount: U32
    telemetry SentFrames: U32
    telemetry Model: WifiHwModel
    telemetry Role: WifiHwRole
    telemetry WifiMode: WifiMode
    telemetry LinkUp: bool
    telemetry LinkDownCount: U32
    telemetry NoLinkDrops: U32
    telemetry BusyDrops: U32
    telemetry TxAbortedOnLinkLoss: U32
    telemetry RxFrames: U32
    telemetry RestartCount: U32
    telemetry RxOverflowDrops: U32
    telemetry Channel: U8
    telemetry Rssi_dBm: I8

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "WifiDriver {} -> {}"
    event RfFault(code: U32) severity warning high format "WifiDriver FAULT code={}"
    event RfTxTimeout(len: U32) severity warning low format "WifiDriver TX timeout len={}"
    event InitOk() severity activity low format "WifiDriver INIT OK"
    event SelftestOk() severity activity low format "WifiDriver SELFTEST OK"
    event SelftestFail(tries: U32) severity warning high format "WifiDriver SELFTEST fail tries={}"
    event WifiLinkUp() severity activity low format "WifiDriver link up" throttle 20
    event WifiLinkDown(downCount: U32) severity warning low format "WifiDriver link down count={}" throttle 20
    event WifiRestart(count: U32) severity activity high format "WifiDriver re-INIT after FAULT count={}"
    event WifiUnsupportedHw() severity warning high format "WifiDriver UNSUPPORTED hardware (no WiFi)"
  }

}
