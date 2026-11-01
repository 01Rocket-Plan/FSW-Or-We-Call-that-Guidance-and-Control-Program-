@ TODO(Power/NvFlags v0.1): provisional until Power group publishes final NvFlags ports.
@ sepReleasedSet: set-only OR into NVS; bit0..3 = SEP_1..4; bit4..7 reserved; FAILURE on NVS write fail.
@ sepReleasedGet: return current released mask from RAM.
module SepNvPlaceholder {

  struct NvSepSet {
    mask: U8
  }

  @ OR bits into persistent released mask
  port SepReleasedSet(
    req: NvSepSet
  ) -> Fw.Success

  @ Read RAM copy of released mask
  port SepReleasedGet(
    ref mask: U8
  )

}
