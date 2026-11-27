module UsMcu {

  # Base ID：照 docs/conventions.md §5（與 DsMcu 同表，兩個部署各自一份 ID 空間）

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ---------------------------------------------------------------- 主動元件
  @ 1 kHz：flightMode（US 精簡狀態）→ platform
  instance rg1kHz: Svc.ActiveRateGroup base id 0x0100 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 50

  @ 100 Hz：usMpu6050 → sdLogger.flush
  instance rg100Hz: Svc.ActiveRateGroup base id 0x0200 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 45

  @ 1 Hz：遙測、指令、健康
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

  # ---------------------------------------------------------------- MCU 組（US 不含控制律）
  instance platform: Mcu.Platform base id 0x1000
  instance flightMode: Mcu.FlightMode base id 0x1400

  # ---------------------------------------------------------------- 環境感測／紀錄（SIL 用 Sim；真機換本尊：UsMpu6050 0x4200、UsDht22 0x4300、SdLogger 0x2000）
  @ US MPU6050（僅記錄）。Base ID 0x4280 ＝ 0x4200 + 0x80
  instance usMpu6050: Components.UsMpu6050Sim base id 0x4280

  @ US DHT22（僅記錄）。Base ID 0x4380 ＝ 0x4300 + 0x80
  instance usDht22: Components.UsDht22Sim base id 0x4380

  @ 上級 SD 紀錄器（SIL：SdCardSimDevice）。Base ID 0x2080 ＝ 0x2000 + 0x80
  instance sdLogger: Components.SdLoggerSim base id 0x2080

  # ---------------------------------------------------------------- 其他組
  instance nvFlags: Components.NvFlags base id 0x6100 \
  {
    phase Fpp.ToCpp.Phases.instances """
    ::NvFlags::RamNvStore nvFlagsStore;
    Components::NvFlags nvFlags(FW_OPTIONAL_NAME("nvFlags"), nvFlagsStore);
    """
    phase Fpp.ToCpp.Phases.configComponents """
    UsMcu::nvFlags.configure();
    """
  }

}
