#ifndef OV3640_HOST_AC_FW_BUFFER_HPP
#define OV3640_HOST_AC_FW_BUFFER_HPP
#include "Fw/Types/BasicTypes.hpp"
namespace Fw {
class Buffer {
 public:
  Buffer() : m_data(nullptr), m_size(0), m_ctx(0) {}
  Buffer(U8* data, U32 size, U32 ctx = 0) : m_data(data), m_size(size), m_ctx(ctx) {}
  U8* getData() const { return m_data; }
  U32 getSize() const { return m_size; }
  void setSize(U32 s) { m_size = s; }
  U32 getContext() const { return m_ctx; }
  void setContext(U32 c) { m_ctx = c; }
 private:
  U8* m_data;
  U32 m_size;
  U32 m_ctx;
};
}
#endif
