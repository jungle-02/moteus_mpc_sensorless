#!/bin/bash
# Luu tai: /work/extract_bazel_info.sh
# Yeu cau: sudo apt install jq

FILE="compile_commands.txt"

echo "=== 1. TAT CA MACRO (DEFINES) ==="
jq -r '.[0].arguments[]' $FILE | grep '^-D' | sort -u > bazel_defines.txt
echo "Da xuat ra bazel_defines.txt"

echo -e "\n=== 2. CO BIEN DICH (FLAGS) ==="
jq -r '.[0].arguments[]' $FILE | grep -E '^-f|^-m|^-O|^-g|^-W|^-std' | sort -u > bazel_flags.txt
echo "Da xuat ra bazel_flags.txt"

echo -e "\n=== 3. CAC THU MUC INCLUDE ==="
jq -r '.[] | .arguments | join(" ")' $FILE \
| grep -oE -- '-(isystem|iquote)[[:space:]]+[^[:space:]]+' \
| sort -u > bazel_includes.txt
echo "Da xuat ra bazel_includes.txt"

echo -e "\n=== 4. TAT CA FILE DA BUILD ==="
jq -r '.[].file' $FILE | sort -u > bazel_sources.txt
echo "Da xuat ra bazel_sources.txt"

echo "Hoan tat trich xuat thong tin."