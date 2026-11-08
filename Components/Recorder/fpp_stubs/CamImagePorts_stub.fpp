@ Stubs matching Camera Ov3640ImageOut / Esp32CamImageOut (2KB chunks).
@ Preferred for F´ build: Components/Recorder/SdLogger/CamImagePorts.fpp (same content).
@ When topology includes real Camera FPP, drop Recorder-local port defs and use Camera's.
module Components {

  constant SdLogger_IMAGE_CHUNK_MAX = 2048

  @ US_CAM image chunk → sdLogger (same signature as Camera Ov3640ImageOut)
  port Ov3640ImageOut(
    frameNumber: U32
    chunkIndex: U32
    flags: U8
    ref data: Fw.Buffer
  )

  @ DS_CAM image chunk → sdLogger (same signature as Camera Esp32CamImageOut)
  port Esp32CamImageOut(
    frameNumber: U32
    chunkIndex: U32
    flags: U8
    ref data: Fw.Buffer
  )

}
