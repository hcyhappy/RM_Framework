#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")/../.."
mkdir -p build/threads-tests
for output in 0 1; do
 g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=undefined -fno-omit-frame-pointer \
  -DRM_ENABLE_IST8310=1 -DRM_ENABLE_AHRS6=1 -DRM_ENABLE_MOTOR_OUTPUT="$output" \
  -Itests/threads/stubs -Itests/modules/hal_stub -Ithreads/include -Imodules/include -Ibsp/include -Ilibs/include \
  tests/threads/test_threads.cpp threads/src/*.cpp modules/src/BMI088.cpp modules/src/IST8310.cpp \
  modules/src/M2006.cpp modules/src/DJIMotorHandler.cpp modules/src/LED.cpp modules/src/ahrs6.cpp \
  bsp/src/bsp_i2c.cpp bsp/src/bsp_time.cpp libs/src/pid.cpp -o "build/threads-tests/test-$output"
 for scenario in startup initfail alive imu control abort offline ahrs; do "build/threads-tests/test-$output" "$scenario"; done
done
