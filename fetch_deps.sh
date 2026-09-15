#!/bin/bash
# Script fetch_deps.sh 
# Luu y: Chay script nay tai thu muc goc /work
set -e 

mkdir -p third_party
cd third_party

echo "--- 1. Clone cac Git Repositories (C++ Libs) ---"

# Ham clone va checkout an toan
clone_or_update() {
    local url=$1
    local dir=$2
    local ref=$3
    if [ ! -d "$dir" ]; then
        git clone "$url" "$dir"
    fi
    cd "$dir"
    git fetch origin
    git checkout "$ref"
    cd ..
}

# Clone cac thanh phan
clone_or_update "https://github.com/mjbots/mjlib.git" "mjlib" "491e614b9795bf1b7acbf158100cb2d36e99716a"
clone_or_update "https://github.com/mjbots/rules_mbed.git" "rules_mbed" "6a276d2ad4e2fe8a81438af4854d8d09d31b980b"
clone_or_update "https://github.com/serge1/ELFIO.git" "elfio" "580da2467b3d7da4c817d45a99a367e4b0d6d326"
clone_or_update "https://github.com/fmtlib/fmt.git" "fmt" "8.1.1"
clone_or_update "https://gitlab.com/libeigen/eigen.git" "eigen" "3.4.0"

# Xu ly Boost & Fixed Point
if [ ! -d "boost" ]; then
    git clone https://github.com/boostorg/boost.git boost
fi
(cd boost && git checkout boost-1.76.0)

if [ ! -d "fixed_point_lib" ]; then
    git clone https://github.com/johnmcfarlane/fixed_point.git fixed_point_lib
fi
(cd fixed_point_lib && (git checkout master || git checkout main))

# Tao cau truc long nhau cho Boost
mkdir -p boost/boost
ln -sfn ../../fixed_point_lib/include/fixed_point boost/boost/fixed_point

echo "--- 2. Tai va Va loi Mbed OS ---"

# Tai Mbed OS 5.13.4
if [ ! -d "mbed-os" ]; then
    if [ ! -f mbed_os.tar.gz ]; then
        curl -L https://github.com/ARMmbed/mbed-os/archive/mbed-os-5.13.4.tar.gz -o mbed_os.tar.gz
    fi
    mkdir -p mbed-os
    tar -xzf mbed_os.tar.gz -C mbed-os --strip-components=1
    rm -f mbed_os.tar.gz
fi

# Ap dung cac ban va
echo "Dang ap dung patches cho STM32G4..."
cd mbed-os
# Su dung --forward de khong bao loi neu patch da duoc ap dung roi
patch -p1 -N < ../rules_mbed/tools/workspace/mbed/mbed.patch || echo "Mbed patch da ton tai, bo qua."
patch -p1 -N < ../rules_mbed/tools/workspace/mbed/stm32g4.patch || echo "G4 patch da ton tai, bo qua."
cd ..

echo "--- 3. Thiet lap tuong thich cau truc ---"
ln -sfn mbed-os mbed-g4
ln -sfn mbed-os mbed-g4-bootloader

echo "--- ✅ Hoan tat: Da tai cac thu vien ben thu ba va ap dung ban va cho dong G4 ---"