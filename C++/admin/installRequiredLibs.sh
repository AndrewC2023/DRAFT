set -e

MATPLOT_INSTALL_DIR="matplotplusplus_install"

echo "Updating package lists ..."
sudo apt-get update

echo "Installing Eigen3 ..."
sudo apt-get install -y libeigen3-dev

echo "Installing YAML ..."
sudo apt-get install -y libyaml-dev

if [ ! -d "$MATPLOT_INSTALL_DIR"]; then
    echo "Cloning Matplot++ ..."
    git clone https://github.com/alandefreitas/matplotplusplus.git

    echo "Building Matplot++ ..."
    cd matplotplusplus
    mkdir -p build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=../../$INSTALL_DIR ..
    make -j2make 

    echo "Installing Matplot++ ..."
    sudo make install
    cd ../..
else
    echo "Matplot++ is Already installed!"
fi

echo "Installation of required packages complete!"
echo "Run setup.sh in the c++ root dir to build the project"
echo "Happy hacking!"