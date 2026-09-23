#!/usr/bin/env bash
# Recreate firmware/sdk from the CH32M030C9U template shipped with MounRiver Studio 2,
# then apply the small patches this project needs.
#
# Usage: scripts/setup_sdk.sh [path/to/CH32M030C9U.zip]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ZIP="${1:-C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/SDK/default/RISC-V/CH32M030/NoneOS/CH32M030C9U.zip}"
SDK="$ROOT/sdk"
TMP="$(mktemp -d)"

unzip -q "$ZIP" -d "$TMP"
rm -rf "$SDK"
mkdir -p "$SDK/User"
cp -r "$TMP/Core" "$TMP/Ld" "$TMP/Peripheral" "$TMP/Startup" "$SDK/"
cp "$TMP/User/system_ch32m030.c" "$TMP/User/system_ch32m030.h" "$TMP/User/ch32m030_conf.h" \
   "$TMP/User/ch32m030_it.c" "$TMP/User/ch32m030_it.h" "$SDK/User/"
rm -rf "$TMP"

# 48 MHz HSI (USB PD BMC timer constants are defined for 48 MHz)
sed -i 's|^#define SYSCLK_FREQ_72MHz_HSI  72000000|//#define SYSCLK_FREQ_72MHz_HSI  72000000|; s|^//#define SYSCLK_FREQ_48MHz_HSI  48000000|#define SYSCLK_FREQ_48MHz_HSI  48000000|' "$SDK/User/system_ch32m030.c"
# No debug.h (UART/printf) in this project
sed -i 's|#include "debug.h"|/* debug.h (UART/printf) is not used */|' "$SDK/User/ch32m030_it.h"
sed -i 's|#include "ch32m030_it.h"|#include "ch32m030.h"|' "$SDK/User/ch32m030_it.c"
sed -i 's|^#include "ch32m030.h"$|#include "ch32m030.h"\n\nextern volatile uint32_t ISINK_ADJ; /* was declared in debug.h, defined in ch32m030_pwr.c */|' "$SDK/User/system_ch32m030.c"

echo "SDK ready in $SDK"
