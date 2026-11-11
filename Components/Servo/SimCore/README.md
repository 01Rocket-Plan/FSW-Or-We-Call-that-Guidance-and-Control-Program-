# Servo SimCore

Framework-free cores for `ServoDriverSim` / `SepServoDriverSim` (host-tested).

- `ServoSimCore`: tail 4 ch / gimbal 2 ch; validates `(SG90,TAIL,4)` or `(SG90,GIMBAL,2)`
- `SepSimCore`: SEP_1..3 fire-once (mask 0x07); validates `(MG996R,SEP)`
- Tests: `CXX="python3 -m ziglang c++" Components/Servo/run_sim_host_tests.sh`
- Device-level slew sims: `Components/Servo/ModelCores/` (Sg90SimDevice / Mg996rSimDevice; see that README)
