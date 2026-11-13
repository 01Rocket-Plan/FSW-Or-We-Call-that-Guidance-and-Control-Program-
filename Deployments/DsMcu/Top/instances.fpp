module DsMcu {

  # ----------------------------------------------------------------------
  # Base ID：照 docs/conventions.md §5
  #   0x0000–0x0FFF F´ 標準元件（本檔）；子拓樸 CdhCore 0x01000000、ComCcsds 0x02000000、
  #   FileHandling 0x05000000（F´ v4.4.1 預設）；其餘照各組段位。
  # ----------------------------------------------------------------------

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ---------------------------------------------------------------- 主動元件
  @ 1 kHz 鏈：imu → estimator → flightMode → controller → actuatorMgr → platform（Core 1）
  instance rg1kHz: Svc.ActiveRateGroup base id 0x0100 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 50

  @ 100 Hz：氣壓
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
  @ SIL：Linux timerfd 當 1 ms 基準時鐘（ESP32 部署換成 esp_timer，下一個 PR）
  instance timer: Svc.LinuxTimer base id 0x0600
  instance systemResources: Svc.SystemResources base id 0x0700
  instance comDriver: Drv.TcpClient base id 0x0800

  # ---------------------------------------------------------------- MCU 組
  instance platform: Mcu.Platform base id 0x1000
  @ SIL 佔位 IMU（EnvSensing ImuDriverSim 能用真 F´ 編譯後換掉）
  instance silImuFeed: Mcu.SilImuFeed base id 0x1080
  instance estimator: Mcu.Estimator base id 0x1100
  instance controller: Mcu.Controller base id 0x1200
  instance actuatorMgr: Mcu.ActuatorMgr base id 0x1300
  instance flightMode: Mcu.FlightMode base id 0x1400

  # ---------------------------------------------------------------- 其他組
  @ 環境感測組 BaroDriverSim（BaroDriver 0x4100 + 0x80）
  instance baroDriverSim: Components.BaroDriverSim base id 0x4180

  @ 電源組 NvFlags；SIL 用 RAM 後端（ESP32 換 NVS 後端）
  instance nvFlags: Components.NvFlags base id 0x6100 \
  {
    phase Fpp.ToCpp.Phases.instances """
    ::NvFlags::RamNvStore nvFlagsStore;
    Components::NvFlags nvFlags(FW_OPTIONAL_NAME("nvFlags"), nvFlagsStore);
    """
    phase Fpp.ToCpp.Phases.configComponents """
    DsMcu::nvFlags.configure();
    """
  }

}
