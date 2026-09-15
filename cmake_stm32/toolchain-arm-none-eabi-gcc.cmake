# File toolchain CMake cho ARM Cortex-M4 (bare-metal)

# ==================== SYSTEM ====================
set(CMAKE_SYSTEM_NAME Generic)  # bare-metal
set(CMAKE_SYSTEM_PROCESSOR arm) # ARM CPU
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY) # test compile khong link exe

# ==================== TOOLCHAIN ====================
find_program(CMAKE_C_COMPILER arm-none-eabi-gcc)   # C compiler
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++) # C++ compiler
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc) # ASM compiler
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)  # convert binary
find_program(CMAKE_SIZE_UTIL arm-none-eabi-size)   # check size

# ==================== FLAGS ====================
set(CORE_MACROS "-D__FPU_PRESENT=1 -D__FPU_USED=1 -D__CORTEX_M4=1 -DARM_MATH_CM4=1") # macro CPU/FPU

# flags chung: optimize + debug + CPU + tach section
set(COMMON_FLAGS "${CORE_MACROS} -O3 -Os -Wall -Wextra -Wno-builtin-macro-redefined -Wno-type-limits -Wno-unused-parameter -Wvla -Wno-array-bounds -Wno-maybe-uninitialized -Wno-stringop-overflow -fdata-sections -ffunction-sections -g -gdwarf-4 -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -mthumb")

# ==================== APPLY FLAGS ====================
set(C_ONLY_FLAGS "-Wno-implicit-function-declaration")
set(CXX_ONLY_FLAGS "-std=c++2a -fno-exceptions -fno-rtti -Wno-register -Wno-volatile")

set(CMAKE_C_FLAGS "${COMMON_FLAGS} ${C_ONLY_FLAGS}" CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${CXX_ONLY_FLAGS}" CACHE INTERNAL "")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp" CACHE INTERNAL "")

# ==================== LINKER ====================
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections --specs=nosys.specs -mfloat-abi=softfp -mfpu=fpv4-sp-d16" CACHE INTERNAL "")

