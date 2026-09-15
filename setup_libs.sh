#!/bin/bash
# Luu tai: /work/setup_libs.sh

echo "--- 1. Dang tai Submodules cho Boost ---"
cd /work/third_party/boost
git submodule update --init libs/config libs/core libs/assert libs/static_assert \
    libs/type_traits libs/preprocessor libs/mpl libs/utility libs/smart_ptr \
    libs/throw_exception libs/date_time libs/system libs/asio libs/iterator \
    libs/numeric libs/conversion libs/detail

echo "--- 2. Dang cau truc lai folder Header cho Boost ---"
rm -rf /work/third_party/boost/boost
mkdir -p /work/third_party/boost/boost
find libs -name "include" -type d | while read dir; do
    if [ -d "$dir/boost" ]; then
        cp -rsf $(pwd)/$dir/boost/* /work/third_party/boost/boost/
    fi
done

echo "--- 3. Tao file mbed_config.h Bare-metal ---"
cat <<EOF > /work/mbed_config.h
#ifndef MBED_CONFIG_H
#define MBED_CONFIG_H
#define MBED_CONF_RTOS_PRESENT 0
#define MBED_CONF_TARGET_LSE_AVAILABLE 0
#define MBED_APP_START 0x8010000
#define MBED_APP_SIZE 0x70000
#endif
EOF

echo "--- ✅ Xong! Giai quyet xong van de thieu header cua Boost va cau hinh Bare-metal ---"