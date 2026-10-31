# UsDht22（US DHT22，RMT）與 UsDht22Sim — 環境感測組（子Bot2）
set(SOURCE_FILES
  "${CMAKE_CURRENT_LIST_DIR}/UsDht22.fpp"
  "${CMAKE_CURRENT_LIST_DIR}/Dht22Core.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Dht22SimIo.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/Dht22RmtEsp32.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsDht22.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/UsDht22Sim.cpp"
)
register_fprime_module()
