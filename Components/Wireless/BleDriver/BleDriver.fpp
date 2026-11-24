module Components {

  @ BLE 硬體型號。主線初代 ESP32（ESP32 DevKit U1，內建 BLE 4.2）；備用 STM32WB55CGU6（內建 BLE 5.4）。
  @ 數值固定：STM32WB55CGU6_INTERNAL_BLE＝0、ESP32_INTERNAL_BLE＝1（不可改既有值）
  enum BleHwModel : U8 { STM32WB55CGU6_INTERNAL_BLE = 0, ESP32_INTERNAL_BLE = 1 }

  @ BLE 角色：PAD_CHECKOUT_LINK＝發射台／地面檢測近距離鏈路（預設）；
  @ AIR_GROUND_LINK＝箭地鏈路（備用，距離需求待評估，主飛行鏈仍為 LoRa RfDriver）
  enum BleHwRole : U8 { PAD_CHECKOUT_LINK = 0, AIR_GROUND_LINK = 1 }

  @ BLE 實體層。預設 LE_1M。LE_2M 僅 BLE 5+ 晶片可用（例如 STM32WB55）；
  @ 初代 ESP32 僅支援 LE_1M，設成 LE_2M 會自動回退 LE_1M。兩者都不支援 LE Coded PHY。
  enum BlePhy : U8 { LE_1M = 0, LE_2M = 1 }

  @ 藍牙備用傳輸（BLE）驅動（LoRa RfDriver 的備用；主飛行鏈仍為 LoRa）；Base ID 0x8800
  @ 埠集合與 RfDriver 完全相同 → 拓樸可直接替換 rfDriver 實例；上層 Framer／ComQueue／UplinkAuth 不變
  @ 狀態機沿用 RfDriverCore（OFF→INIT→SELFTEST→READY／FAULT），BLE 切片／重組／掉線在 BleLinkCore
  @ 硬體存取經 IBleHal（平台用 ESP-IDF NimBLE／Bluedroid 或 STM32_WPAN 實作；未接 HAL 時 INIT 失敗 → FAULT）
  passive component BleDriver {

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

    @ 硬體型號 — 預設主線 ESP32 內建 BLE
    param model: BleHwModel default BleHwModel.ESP32_INTERNAL_BLE
    @ 角色 — 預設發射台／地面檢測鏈路
    param role: BleHwRole default BleHwRole.PAD_CHECKOUT_LINK

    @ 發射功率 dBm；依硬體型號夾取（ESP32 −12..+9、STM32WB55 −40..+6）。HAL 可視板級限制再夾。
    param txPower_dBm: I32 default 0
    @ 實體層；預設 LE_1M。LE_2M 僅 BLE 5+ 可用；初代 ESP32 設成 LE_2M 時自動回退 LE_1M
    param phy: BlePhy default BlePhy.LE_1M
    @ 要求的 ATT_MTU（23..247）；247 → 每個 notification 244 B
    param attMtu: U16 default 247
    @ 沒連線時的廣播間隔 ms
    param advInterval_ms: U16 default 100
    @ 期望連線間隔 ms（中心端決定是否接受）
    param connInterval_ms: U16 default 15
    @ 連線監督逾時 ms
    param supervisionTimeout_ms: U16 default 2000

    @ 以下與 RfDriver 同義（給 RfDriverCore）
    @ TX 逾時：預期送完之後再多等的秒數（與 RfDriver 同義）。BLE 由 BleLinkCore 回報送完、
    @ 不套 LoRa 空中時間，預期送完時間視為 0 → 送出後 txTimeout_s 內最後一片沒被 HAL 收下就算逾時（逾時時清掉這一幀）
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
    telemetry Model: BleHwModel
    telemetry Role: BleHwRole
    telemetry LinkUp: bool
    telemetry AttMtu: U16
    telemetry LinkDownCount: U32
    telemetry NoLinkDrops: U32
    telemetry BusyDrops: U32
    telemetry TxAbortedOnLinkLoss: U32
    telemetry FragmentsSent: U32
    telemetry RxFrames: U32
    telemetry RxFragErrors: U32
    telemetry RestartCount: U32
    telemetry RxOverflowDrops: U32

    event RfState(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high format "BleDriver {} -> {}"
    event RfFault(code: U32) severity warning high format "BleDriver FAULT code={}"
    event RfTxTimeout(len: U32) severity warning low format "BleDriver TX timeout len={}"
    event InitOk() severity activity low format "BleDriver INIT OK"
    event SelftestOk() severity activity low format "BleDriver SELFTEST OK"
    event SelftestFail(tries: U32) severity warning high format "BleDriver SELFTEST fail tries={}"
    event BleLinkUp(attMtu: U16) severity activity low format "BleDriver link up MTU={}" throttle 20
    event BleLinkDown(downCount: U32) severity warning low format "BleDriver link down count={}" throttle 20
    event BleRestart(count: U32) severity activity high format "BleDriver re-INIT after FAULT count={}"
  }

}
