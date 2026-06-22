#!/bin/bash
###########################################################
# Name: setuo.sh

# To be run after opening the container, configures the cmake build directory, can also be used to delete and re-create the build directory if needed.

# Usage: sudo ./setup.sh CMAKE_FLAGS

# Date: 2026/06/14
###########################################################

set -euo pipefail
cd ..
rm -rf build
mkdir build
cd build

mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=Release ../..

cd ..
mkdir debug
cd debug
cmake -DCMAKE_BUILD_TYPE=Debug ../..