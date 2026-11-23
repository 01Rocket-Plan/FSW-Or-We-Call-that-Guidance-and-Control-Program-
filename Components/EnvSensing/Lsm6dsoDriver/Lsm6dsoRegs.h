/* LSM6DSO 暫存器與數值（C／C++ 共用）。
 * 來源：ST 官方 lsm6dso-pid/lsm6dso_reg.h、lsm6dso_reg.c；自測極限：datasheet DS12140 Rev 3 Table 2
 * （ST 範例 STMems_Standard_C_drivers/lsm6dso_STdC/examples/lsm6dso_self_test.c 用同一組數字）。 */
#ifndef ENV_SENSING_LSM6DSO_REGS_H
#define ENV_SENSING_LSM6DSO_REGS_H

enum {
  LSM_REG_WHO_AM_I = 0x0F,   /* 讀回 0x6C */
  LSM_REG_CTRL1_XL = 0x10,   /* [7:4] ODR_XL、[3:2] FS_XL、[1] LPF2_XL_EN */
  LSM_REG_CTRL2_G  = 0x11,   /* [7:4] ODR_G、[3:1] FS_G（含 FS_125） */
  LSM_REG_CTRL3_C  = 0x12,   /* [7] BOOT [6] BDU [2] IF_INC [0] SW_RESET */
  LSM_REG_CTRL5_C  = 0x14,   /* [3:2] ST_G、[1:0] ST_XL */
  LSM_REG_OUTX_L_G = 0x22,   /* 0x22–0x27 陀螺 X/Y/Z（L,H），0x28–0x2D 加速度 X/Y/Z（L,H），little-endian */
  LSM_REG_OUTX_L_A = 0x28
};

enum {
  LSM_CTRL3_SW_RESET = 0x01,
  LSM_CTRL3_IF_INC   = 0x04,
  LSM_CTRL3_BDU      = 0x40,
  /* 飛行：XL ODR 1667 Hz(8)、FS ±16 g(1)；G ODR 1667 Hz(8)、FS ±2000 dps(6) */
  LSM_CTRL1_XL_1667HZ_16G = 0x84,
  LSM_CTRL2_G_1667HZ_2000DPS = 0x8C,
  /* 自測（照 ST 範例）：XL ODR 52 Hz(3)、FS ±4 g(2)；G ODR 208 Hz(5)、FS ±2000 dps(6) */
  LSM_CTRL1_XL_52HZ_4G = 0x38,
  LSM_CTRL2_G_208HZ_2000DPS = 0x5C,
  LSM_CTRL_ODR_OFF = 0x00,
  LSM_CTRL5_ST_XL_POS = 0x01,  /* ST_XL=01 正向 */
  LSM_CTRL5_ST_G_POS  = 0x04,  /* ST_G=01 正向 */
  LSM_CTRL5_ST_OFF    = 0x00
};

/* 靈敏度（ST lsm6dso_from_fsXX_to_mg/mdps） */
#define LSM_SENS_16G_MG_PER_LSB     0.488f
#define LSM_SENS_4G_MG_PER_LSB      0.122f
#define LSM_SENS_2000DPS_MDPS_PER_LSB 70.0f
/* 自測輸出變化量合格範圍（datasheet Table 2；XL 與 FS 無關；G 是 ±2000 dps 時） */
#define LSM_ST_XL_MIN_MG     50.0f
#define LSM_ST_XL_MAX_MG   1700.0f
#define LSM_ST_G_MIN_MDPS 150000.0f
#define LSM_ST_G_MAX_MDPS 700000.0f

#endif
