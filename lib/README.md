`lib/fprime` 是 F´ 子模組，**鎖定 v4.4.1**（commit `558b7ef`，MCU 組 2026-11-10）。

```bash
git submodule update --init lib/fprime
pip install fprime-tools==4.4.0 fprime-fpp==3.4.0   # 與 lib/fprime/requirements.txt 一致
```

- 為什麼是 v4.4.1：各組的 `run_fprime_compile_check.sh` 都是對 v4.4.1 標頭寫的；MCU 元件也是對 v4.4.1 真框架編譯 0 警告。
- `.gitmodules` 設 `shallow = true`，clone 只抓這一版。
- 要升級：先在 issue 提，所有組一起換。
