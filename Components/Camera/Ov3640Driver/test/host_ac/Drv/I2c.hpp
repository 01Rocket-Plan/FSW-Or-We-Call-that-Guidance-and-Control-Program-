#ifndef OV3640_HOST_AC_DRV_I2C_HPP
#define OV3640_HOST_AC_DRV_I2C_HPP
namespace Drv {
struct I2cStatus {
  enum T { I2C_OK = 0, I2C_ADDRESS_ERR, I2C_WRITE_ERR, I2C_READ_ERR, I2C_OPEN_ERR, I2C_OTHER_ERR };
  T e;
  I2cStatus(T v = I2C_OK) : e(v) {}
  bool operator==(T o) const { return e == o; }
};
}
#endif
