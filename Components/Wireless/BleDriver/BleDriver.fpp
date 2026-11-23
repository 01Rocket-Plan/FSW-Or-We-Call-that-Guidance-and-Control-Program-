module Components {

  @ BLE 硬體型號 — STM32WB55CGU6 內建 BLE（2.4 GHz；CPU2／Cortex-M0+ 跑 ST 無線堆疊，CPU1 跑本驅動）
  enum BleHwModel : U8 { STM32WB55CGU6_INTERNAL_BLE = 0 }

  @ BLE 角色：PAD_CHECKOUT_LINK＝發射台／地面檢測近距離鏈路（預設，見 BLE_FEASIBILITY）；
  @ AIR_GROUND_LINK＝箭地鏈路（備用，距離需求待評估，主飛行鏈仍為 LoRa RfDriver）
  enum BleHwRole : U8 { PAD_CHECKOUT_LINK = 0, AIR_GROUND_LINK = 1 }

  @ BLE 實體層。STM32WB55 只支援 LE 1M／LE 2M（不支援 LE Coded）
  enum BlePhy : U8 { LE_1M = 0, LE_2M = 1 }

  @ BLE 數傳驅動（LoRa RfDriver 的備用傳輸）；Base ID 0x8800（提案，待首席核准）
  @ 埠集合與 RfDriver 完全相同 → 拓樸可直接替換 rfDriver 實例；上層 Framer／ComQueue／UplinkAuth 不變
  @ 狀態機沿用 RfDriverCore（OFF→INIT→SELFTEST→READY／FAULT），BLE 切片／重組／掉線在 BleLinkCore
  @ 硬體存取經 IBleHal（MCU 平台用 STM32_WPAN 實作；未接 HAL 時 INIT 失敗 → FAULT）
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

    @ 硬體型號 — STM32WB55CGU6 內建 BLE
    param model: BleHwModel default BleHwModel.STM32WB55CGU6_INTERNAL_BLE
    @ 角色 — 預設發射台／地面檢測鏈路
    param role: BleHwRole default BleHwRole.PAD_CHECKOUT_LINK

    @ 發射功率 dBm；WB55 可設 −40..+6（AN5270 aci_hal_set_tx_power_level），超出會被夾住
    param txPower_dBm: I32 default 0
    @ 實體層；LE_1M 靈敏度較好（datasheet 只給 1M 的 −96 dBm），LE_2M 吞吐較高
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
