#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")/../.."
mkdir -p build/libs-tests
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-omit-frame-pointer -Ilibs/include tests/libs/test_libs.cpp libs/src/pid.cpp libs/src/math.cpp libs/src/crc.cpp -o build/libs-tests/test_libs
# No dynamic allocation under test; LSan cannot enumerate processes in this sandbox.
ASAN_OPTIONS=detect_leaks=0 build/libs-tests/test_libs
