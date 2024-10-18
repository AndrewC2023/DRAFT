#!/bin/bash
###########################################################
# Name: installFromSource.sh

# Installs library via "make install"

# Usage: sudo ./installFromSource.sh TAR_URL CMAKE_FLAGS

# Date: 2020/10/27
###########################################################

set -euxo pipefail

# check dependencies
which pkg-config && which cmake

declare -ir THREADS=${THREADS:-4}
declare -r TAR_URL=${1}
declare -r CMAKE_FLAGS=${2:-""}
declare -r TMP_DIRECTORY="REQ_LIBS_TMP"

if [ -d ${TMP_DIRECTORY} ]; then
  rm -rf ${TMP_DIRECTORY}
fi
mkdir ${TMP_DIRECTORY}

curl -L "${TAR_URL}" \
  | tar xz -C ${TMP_DIRECTORY} --strip-components 1

cd ${TMP_DIRECTORY} \
  && mkdir build \
  && cd build \
  && cmake ${CMAKE_FLAGS} .. \
  && make -j${THREADS} \
  && make install \
  && cd ../..

rm -rf ${TMP_DIRECTORY}
