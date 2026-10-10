#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")/../.."
mkdir -p build/modules-tests
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined -fno-omit-frame-pointer \
 -Itests/modules/hal_stub -Imodules/include -Ibsp/include -Ilibs/include \
 tests/modules/test_modules.cpp modules/src/BMI088.cpp modules/src/IST8310.cpp \
 modules/src/M2006.cpp modules/src/DJIMotorHandler.cpp modules/src/LED.cpp \
 bsp/src/bsp_i2c.cpp bsp/src/bsp_time.cpp libs/src/pid.cpp -o build/modules-tests/test_modules
build/modules-tests/test_modules
