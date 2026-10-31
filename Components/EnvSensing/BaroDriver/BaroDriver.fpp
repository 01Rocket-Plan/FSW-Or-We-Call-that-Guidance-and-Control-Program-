module EnvSensing {

  @ BME680 模擬器故障注入
  enum BaroSimFault { NONE, NACK, STUCK, OUT_OF_RANGE, BAD_CHIP_ID, NO_DATA }

  @ DS BME680 氣壓／環境驅動（I2C）。接 rg100Hz：溫壓觸發與讀取交替約 50 Hz，
  @ 每 1 s 插一次溫壓＋氣體轉換（該段不輸出 Baro、seq 不加）。不 sleep、時間只取自 F´ 時間服務。
  passive component BaroDriver {

    @ rg100Hz
    sync input port schedIn: Svc.Sched

    @ I2C 寫入
    output port i2cWrite: Drv.I2c

    @ I2C 先寫後讀（repeated start）
    output port i2cWriteRead: Drv.I2cWriteRead

    @ 氣壓樣本 → estimator
    output port baroOut: RocketTypes.BaroData

    @ 濕度／氣體 → sdLogger（只送 Recorder）
    output port envOut: RocketTypes.EnvData

    time get port timeCaller
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    param get port prmGetOut
    param set port prmSetOut

    @ 握手狀態（只走遙測）
    telemetry HsState: RocketTypes.DrvState
    telemetry ErrorCount: U32
    telemetry RefPress_Pa: F32
    telemetry BaroSeq: U32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "BaroDriver {} -> {}"

    @ code：1 BUS_ERROR、2 BAD_CHIP_ID、3 STUCK、4 OUT_OF_RANGE、5 MEAS_TIMEOUT、6 SELFTEST_FAIL
    event Fault(code: U8, detail: U32) \
      severity warning high \
      format "BaroDriver fault code={} detail={}"

    @ 高度基準氣壓；0 = 自測期間取發射台平均
    param P0_PA: F32 default 0.0
    @ 卡值判定筆數（約 50 Hz）
    param STUCK_N: U16 default 50
    @ 連續錯誤門檻
    param ERR_MAX: U8 default 5
    @ 加熱器等待（datasheet 建議 20–30 ms）
    param GAS_WAIT_MS: U16 default 30
    @ 加熱器目標溫度
    param HEATER_TEMP_C: U16 default 320
  }

  @ BaroDriver 的 Sim 版：同一個驅動核心，接 BME680 暫存器層模擬器（base id = BaroDriver + 0x80）
  passive component BaroDriverSim {

    sync input port schedIn: Svc.Sched
    output port baroOut: RocketTypes.BaroData
    output port envOut: RocketTypes.EnvData

    time get port timeCaller
    telemetry port tlmOut
    event port logOut
    text event port logTextOut
    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut

    @ 設定模擬高度（標準大氣，海平面 101325 Pa）
    sync command SIM_SET_ALT(alt_m: F32)

    @ 設定模擬溫度、濕度與氣體電阻
    sync command SIM_SET_ENV(temp_C: F32, humidity_pct: F32, gas_ohm: F32)

    @ 注入故障
    sync command SIM_INJECT(fault: BaroSimFault)

    telemetry HsState: RocketTypes.DrvState
    telemetry ErrorCount: U32
    telemetry RefPress_Pa: F32
    telemetry BaroSeq: U32

    event StateChanged(fromState: RocketTypes.DrvState, toState: RocketTypes.DrvState) \
      severity activity high \
      format "BaroDriverSim {} -> {}"

    event Fault(code: U8, detail: U32) \
      severity warning high \
      format "BaroDriverSim fault code={} detail={}"
  }

}
