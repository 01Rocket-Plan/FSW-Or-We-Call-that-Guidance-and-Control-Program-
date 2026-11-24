// ======================================================================
// \title  RfDriverSim.hpp
// \brief  rfDriverSim F´ 外殼（passive；run 掛 Core 0 rg100Hz）。握手／TX／RX 全在 RfDriverCore，
//         位元組層模擬（掉包／BER／LoRa 空中時間）在 RfDriverSimDevice；本檔只搬資料。
//
// 為什麼用不透明儲存（pimpl）：RfDriverCore.hpp 的 Components::DrvState 跟同拓樸 BaroDriverSim 用的
// Bme680Core.hpp 同名，拓樸自動碼會把兩個元件標頭放進同一個編譯單元 → 重複定義。所以本標頭不 include
// 任何 Core 標頭，Core／Device 放在 .cpp 的 RfDriverSimImpl，存在元件內的固定大小緩衝（不配置記憶體）。
//
// 資料流：dataIn（framer 的空中幀）→ 1 格 TX 槽 → Core.popFrame → Device.writeFrame。
// 槽一收下幀就送 comStatusOut=FAILURE（忙碌），comQueue 停止吐幀；Core 送完回報就緒再送 SUCCESS。
// ======================================================================
#ifndef COMPONENTS_WIRELESS_RFDRIVERSIM_HPP
#define COMPONENTS_WIRELESS_RFDRIVERSIM_HPP

#include <cstddef>

#include "Components/Wireless/RfDriverSim/RfDriverSimComponentAc.hpp"

namespace Components {

class RfDriverSimImpl;

class RfDriverSim final : public RfDriverSimComponentBase {
  friend class RfDriverSimImpl;

 public:
  explicit RfDriverSim(const char* compName);
  ~RfDriverSim() override;

  RfDriverSim(const RfDriverSim&) = delete;
  RfDriverSim& operator=(const RfDriverSim&) = delete;

  //! 遙測寫出間隔（rg100Hz 下 100 拍＝1 Hz）
  static constexpr U32 kTlmEveryTicks = 100;
  //! RfDriverSimImpl 的儲存上限（.cpp 以 static_assert 檢查）
  static constexpr std::size_t kImplBytes = 4096;

 private:
  // ---- 埠
  void dataIn_handler(FwIndexType portNum, Fw::Buffer& data, const ComCfg::FrameContext& context) override;
  void run_handler(FwIndexType portNum, U32 context) override;

  // ---- 參數
  void parameterUpdated(FwPrmIdType id) override;
  void parametersLoaded() override;
  void loadParams();

  F64 nowSeconds();
  RfDriverSimImpl& impl();

  alignas(std::max_align_t) U8 m_implStorage[kImplBytes];
  bool m_paramsLoaded;
  bool m_started;
  U32 m_tlmDiv;
};

}  // namespace Components

#endif
