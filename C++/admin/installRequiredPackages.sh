#!/bin/bash
set -e
declare -i THREADS=${1:-4}

# these will be installed locally on the user's machine if they're not already installed
echo "Updating package lists ..."
sudo apt-get update

# eigen Library
echo "Installing Eigen3 ..."
sudo apt-get install -y libeigen3-dev

# PCL (Point Cloud Library)
sudo apt install libpcl-dev -y

# boost
sudo apt-get install libboost-all-dev 

# curl
sudo apt-get install -y curl

# yaml-cpp
echo "Installing Yaml-cpp ..."
declare -r YAML_CPP_URL="https://github.com/jbeder/yaml-cpp/archive/refs/tags/0.8.0.tar.gz"
THREADS=$THREADS ./installFromSource.sh "${YAML_CPP_URL}"

echo "Installing Matplot++ ..."
declare -r MATPLOTPP_URL="https://github.com/alandefreitas/matplotplusplus/archive/refs/tags/v1.2.1.tar.gz"
THREADS=$THREADS ./installFromSource.sh "${MATPLOTPP_URL}" "-DCMAKE_INSTALL_PREFIX=/usr/local -DMATPLOTPP_BUILD_EXAMPLES=OFF -DMATPLOTPP_BUILD_SHARED_LIBS=ON -DMATPLOTPP_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON"

# gnuplot
sudo apt install gnuplot

echo ""
echo "Installation of required packages complete!"
echo "Run setup.sh to build the project"