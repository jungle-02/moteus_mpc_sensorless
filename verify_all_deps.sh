#!/bin/bash
# Luu tai: /work/verify_all_deps.sh

BASE_DIR="/work"
INCLUDES="-I$BASE_DIR -I$BASE_DIR/fw -I$BASE_DIR/third_party/mjlib -I$BASE_DIR/third_party/mbed-os \
-I$BASE_DIR/third_party/mbed-os/platform -I$BASE_DIR/third_party/mbed-os/drivers -I$BASE_DIR/third_party/mbed-os/hal \
-I$BASE_DIR/third_party/mbed-os/cmsis -I$BASE_DIR/third_party/mbed-os/cmsis/TARGET_CORTEX_M \
-I$BASE_DIR/third_party/mbed-os/rtos -I$BASE_DIR/third_party/mbed-os/rtos/TARGET_CORTEX \
-I$BASE_DIR/third_party/mbed-os/rtos/TARGET_CORTEX/rtx5/Include \
-I$BASE_DIR/third_party/mbed-os/rtos/TARGET_CORTEX/rtx5/RTX/Include \
-I$BASE_DIR/third_party/mbed-os/targets/TARGET_STM \
-I$BASE_DIR/third_party/mbed-os/targets/TARGET_STM/TARGET_STM32G4 \
-I$BASE_DIR/third_party/mbed-os/targets/TARGET_STM/TARGET_STM32G4/device \
-I$BASE_DIR/third_party/mbed-os/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE \
-I$BASE_DIR/third_party/mbed-os/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/device \
-I$BASE_DIR/third_party/mbed-os/targets/TARGET_STM/TARGET_STM32G4/TARGET_STM32G474xE/TARGET_NUCLEO_G474RE \
-I$BASE_DIR/third_party/boost -I$BASE_DIR/third_party/fmt/include -I$BASE_DIR/third_party/eigen"

DEFINES="-DTARGET_STM32G474xE -DTARGET_STM32G4 -D__ARM_ARCH_7EM__=1 -D__CORTEX_M4 -D__FPU_PRESENT=1 -DMBED_CONF_RTOS_PRESENT=0 -DINITIAL_SP=0x20020000"

echo "--- DANG QUET TOAN BO FIRMWARE SOURCE ---"
for f in fw/*.cc; do
    # Bo qua cac file test va cac cong cu PC
    if [[ "$f" == *"_test.cc" ]] || [[ "$f" == *"_main.cc" ]]; then continue; fi

    arm-none-eabi-g++ -E -xc++ $DEFINES $INCLUDES "$f" > /dev/null 2>err.txt
    if [ $? -eq 0 ]; then
        echo "✅ ${f#fw/}: Du"
    else
        REAL_ERR=$(grep "fatal error:" err.txt || grep "error:" err.txt | head -n 1)
        echo "❌ ${f#fw/}: THIEU -> $REAL_ERR"
    fi
done
rm -f err.txt