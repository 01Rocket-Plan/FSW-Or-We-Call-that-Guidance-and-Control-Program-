module PadMcu {

  # Base ID：照 docs/conventions.md §5（PadMcu 只用 0x0000 段與無線 0x8600–0x87FF 段）

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # ---------------------------------------------------------------- 主動元件
  @ 1 kHz：PadController（推力確認／鬆夾持以 1 ms 一拍計）
  instance rg1kHz: Svc.ActiveRateGroup base id 0x0100 \
    queue size Default.QUEUE_SIZE \
    stack size Default.STACK_SIZE \
    priority 50

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

  # ---------------------------------------------------------------- 無線組
  @ 發射架主控（SIL：沒接 ioGet／actOut／NRF24／臍帶 UART → 互鎖全紅、停在 SAFE）
  instance padController: Components.PadController base id 0x8700

}
