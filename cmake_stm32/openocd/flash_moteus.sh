#!/bin/bash
BUILD_DIR="/work/cmake_stm32/build"

echo "======================================"
echo "Bắt đầu nạp firmware cho moteus n1..."
echo "Sử dụng mạch nạp: ST-LINK"
echo "Target: STM32G4x"
echo "======================================"

openocd \
  -f interface/stlink.cfg \
  -f target/stm32g4x.cfg \
  -c "adapter speed 4000" \
  -c "init" \
  -c "reset halt" \
  -c "program ${BUILD_DIR}/moteus.08000000.bin verify 0x08000000" \
  -c "program ${BUILD_DIR}/moteus.0800c000.bin verify 0x0800c000" \
  -c "program ${BUILD_DIR}/moteus.08010000.bin verify 0x08010000" \
  -c "resume 0x08010000" \
  -c "shutdown"

if [ $? -eq 0 ]; then
    echo -e "\n======================================"
    echo "FLASH VÀ VERIFY THÀNH CÔNG!"
    echo "======================================"
else
    echo -e "\n======================================"
    echo "LỖI: Quá trình nạp thất bại hoặc không tìm thấy ST-LINK."
    echo "======================================"
fi