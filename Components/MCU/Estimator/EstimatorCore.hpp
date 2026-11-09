// ======================================================================
// \title  EstimatorCore.hpp
// \author MCU 組（EdwardWang+Claude）
// \brief  DS 估態核心：IMU（機體系）＋氣壓 → 姿態四元數、角速度、傾角、
//         垂直高度／速度（2 態線性 KF）。純 C++14，不含 F´、不含 ESP-IDF、
//         不 sleep、不讀系統時間、不動態配置。
//
// 座標（docs/conventions.md）：世界 ENU；機體 Z 沿長軸指向鼻錐；
// 四元數 Hamilton (w,x,y,z)，q 把機體向量轉到世界：v_w = q ⊗ v_b ⊗ q*。
// 墊上靜置比力（機體系）= (0, 0, +g)。
//
// 輸入的 IMU 已由 ImuDriver 用 prmDb 的 DCM 轉到機體系；這裡不再轉軸。
// ======================================================================
#ifndef COMPONENTS_MCU_ESTIMATOR_ESTIMATORCORE_HPP
#define COMPONENTS_MCU_ESTIMATOR_ESTIMATORCORE_HPP

#include <cstdint>

namespace Mcu {

//! 可由 prmDb 改的數字。預設值見 README「參數」表。
struct EstimatorParams {
  float g_mps2 = 9.80665f;          //!< 重力
  float padAlign_s = 0.5f;          //!< 墊上對準：累積多久的加速度／陀螺平均
  float accGate_mps2 = 0.5f;        //!< | |f| − g | 小於此值才算「靜止」，可拿加速度修水平
  float rateGate_rads = 0.05f;      //!< |ω − bias| 小於此值才算靜止（零偏濾波用）
  float kLevel_rads = 0.5f;         //!< 墊上水平修正增益（Mahony Kp，1/s）
  float gyroBiasTau_s = 2.0f;       //!< 墊上陀螺零偏低通時間常數
  float qAcc_m2ps4 = 0.25f;         //!< KF 程序雜訊：加速度白雜訊方差 (m/s²)²
  float rBaro_m2 = 0.25f;           //!< KF 氣壓高度量測方差 m²（σ=0.5 m）
  float baroGate_m = 10.0f;         //!< 氣壓新息超過此值丟棄
  float rZupt_m2ps2 = 1.0e-4f;      //!< 墊上零速更新方差 (m/s)²
  float altRefTau_s = 5.0f;         //!< 墊上氣壓基準低通時間常數
  float imuStale_s = 0.020f;        //!< IMU 停更新多久算不新鮮（對齊 SSS t_imu_stale_s）
  float baroMaxAge_s = 1.0f;        //!< 氣壓多久沒更新 velValid 變 false
  float dtMax_s = 0.005f;           //!< 單步積分上限（防時間跳躍）
};

struct ImuIn {
  double t_s = 0.0;
  uint32_t seq = 0;
  bool valid = false;
  float acc_mps2[3] = {0, 0, 0};   //!< 比力，機體系
  float gyro_rads[3] = {0, 0, 0};  //!< 角速度，機體系
  bool gyroSat = false;
};

struct BaroIn {
  double t_s = 0.0;
  uint32_t seq = 0;
  bool valid = false;
  float alt_m = 0.0f;  //!< 驅動算好的氣壓高度（任意基準；這裡只用差值）
};

//! 與 RocketTypes.StateOut v1.0 一一對應（wrapper 照抄）。
struct StateOutC {
  uint32_t seq = 0;
  double t_s = 0.0;
  float q_wxyz[4] = {1, 0, 0, 0};
  float rate_body_rads[3] = {0, 0, 0};
  float tilt_rad = 0.0f;
  float vel_enu_mps[3] = {0, 0, 0};
  bool velValid = false;
  float alt_m = 0.0f;           //!< 相對發射台（墊上氣壓基準）
  float baroAge_s = 1.0e6f;
  float accAxial_mps2 = 0.0f;   //!< 世界 U 向運動學加速度（已扣 g），對齊 SSS accel_u_mps2
  bool imuValid = false;        //!< IMU 新鮮 且 姿態已對準
  bool gyroSat = false;
};

//! 給遙測／事件用的內部狀態。
struct EstimatorDiag {
  bool aligned = false;
  bool onPad = true;
  float gyroBias_rads[3] = {0, 0, 0};
  float altRef_m = 0.0f;
  uint32_t baroRejectCount = 0;
  uint32_t imuStaleCount = 0;   //!< 新鮮→不新鮮 的次數
};

class EstimatorCore {
 public:
  EstimatorCore();

  void setParams(const EstimatorParams& p) { m_p = p; }
  const EstimatorParams& params() const { return m_p; }

  //! 最新樣本（seq 相同視為沒更新）。
  void pushImu(const ImuIn& s) { m_imu = s; }
  void pushBaro(const BaroIn& s) { m_baro = s; }
  //! onPad = 飛行模式在墊上（PAD_IDLE／ARMED）。只有墊上才用加速度修水平、估零偏、做零速更新。
  void setOnPad(bool onPad) { m_diag.onPad = onPad; }

  //! 1 kHz 呼叫一次。now_s 只來自 F´ 時間服務。
  void step(double now_s, StateOutC& out);

  const EstimatorDiag& diag() const { return m_diag; }

  // ---- 數學小工具（測試用，公開） ----
  static void quatToR(const float q[4], float R[3][3]);
  static void quatMul(const float a[4], const float b[4], float out[4]);
  static void quatNormalize(float q[4]);
  //! 最短弧：把機體單位向量 a 轉到世界 +U 的四元數（偏航取 0）。
  static void quatFromUp(const float a_body[3], float q[4]);

 private:
  void align(const float fMean[3]);
  void propagateAttitude(const float w[3], const float f[3], float dt);
  void kfPredict(float a_u, float dt);
  void kfUpdateAlt(float z);
  void kfUpdateVelZero();

  EstimatorParams m_p;
  EstimatorDiag m_diag;
  ImuIn m_imu;
  BaroIn m_baro;

  // 對準累積
  float m_fSum[3];
  float m_wSum[3];
  uint32_t m_nSum;
  double m_tAlignStart_s;

  // 姿態
  float m_q[4];

  // 垂直 KF：x = [h, v]，P 2×2
  float m_h, m_v, m_P[2][2];
  float m_vE, m_vN;  //!< 水平速度：純積分，無修正（GNSS 不進估態）
  bool m_altRefInit;

  // 追蹤
  bool m_haveImu;
  uint32_t m_lastImuSeq;
  double m_lastImuT_s;
  uint32_t m_lastBaroSeq;
  double m_lastBaroT_s;
  bool m_haveBaro;
  bool m_wasFresh;
  uint32_t m_seq;
  float m_aU;
};

}  // namespace Mcu

#endif
