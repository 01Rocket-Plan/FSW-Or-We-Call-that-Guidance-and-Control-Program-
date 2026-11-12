@ Stubs matching Camera Ov3640ImageOut / Esp32CamImageOut (2KB chunks).
@ For local fpp-check of SdLogger.fpp only; not part of the F´ CMake build.
@ Do not include together with Camera FPP — the ports are defined in
@ Components/Camera/Ov3640Driver/Ov3640Driver.fpp and Components/Camera/DsCam/Esp32Cam.fpp.

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
