#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")/../.."
mkdir -p build/bsp-tests
# Linux x86-64: place .bss in STM32 SRAM range to test the real DMA memory guard.
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined -fno-pie -no-pie \
    -Wl,--section-start=.bss=0x20001000 -Itests/bsp/hal_stub -Ibsp/include \
    tests/bsp/test_bsp.cpp bsp/src/bsp.cpp bsp/src/bsp_can.cpp \
    bsp/src/bsp_usart.cpp bsp/src/bsp_pwm.cpp -o build/bsp-tests/test_bsp
build/bsp-tests/test_bsp
