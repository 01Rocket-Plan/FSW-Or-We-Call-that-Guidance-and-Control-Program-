module UsCam {

  # Base ID：照 docs/conventions.md §5（相機 0x7000 段；SdLogger 用紀錄組 0x2000 段；Sim ＝ 本尊 +0x80）

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ---------------------------------------------------------------- 主動元件
  @ 100 Hz：相機擷取／初始化批次、SD 寫入
  instance rg100Hz: Svc.ActiveRateGroup base id 0x0200 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 45

  @ 1 Hz：相機狀態機／健康、遙測、指令
  instance rg1Hz: Svc.ActiveRateGroup base id 0x0300 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 40

  # ---------------------------------------------------------------- 被動元件（F´）
  instance rateGroupDriver: Svc.RateGroupDriver base id 0x0400
  instance chronoTime: Svc.ChronoTime base id 0x0500
  instance timer: Svc.LinuxTimer base id 0x0600
  instance systemResources: Svc.SystemResources base id 0x0700
  instance comDriver: Drv.TcpClient base id 0x0800

  # ---------------------------------------------------------------- 相機組
  @ 上級相機 OV3640（備援分支）（SIL：暫存器／幀模擬）。真機換成 Components.Ov3640Driver（0x7100）
  instance usCam: Components.Ov3640DriverSim base id 0x7180

  # ---------------------------------------------------------------- 紀錄組
  @ 相機板本機 SD（SIL：SdCardSimDevice）
  instance sdLogger: Components.SdLoggerSim base id 0x2080

}
