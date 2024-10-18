set -e

# these will be installed locally on the user's machine if they're not already installed
echo "Updating package lists ..."
sudo apt-get update

echo "Installing Eigen3 ..."
sudo apt-get install -y libeigen3-dev

echo "Installing YAML ..."
sudo apt-get install -y libyaml-dev

# the following is for libraries that are either not on apt or are large enough to justify
# allowing the user to have it conveniently removed when they delete the entire project

ADMIN_DIR=$(realpath .)
echo "in admin directory: ${ADMIN_DIR}"

echo ""
echo "-------------------------------------------------------------------------------------------------------------------------"
echo 'Would you like to install Matplot++ in the project directory or locally with the rest of your libraries?'
printf '1: In usr    2: In Project: '
read answer

if [ "$answer" = "1" ]
then 
    echo "installing in /usr/include"
    cd /usr/include

    echo "Cloning Matplot++ ..."
    set +e
    git clone https://github.com/alandefreitas/matplotplusplus.git
    set -e

    echo "Building Matplot++ ..."
    cd matplotplusplus
    mkdir -p build
    cd build

    # set the install directory
    INSTALL_DIR=$(realpath ../../../)
    echo "in project install directory: ${INSTALL_DIR}"

    # set the cmake flags for matplot 
    cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"  \
        -DMATPLOTPP_BUILD_EXAMPLES=OFF \
        -DMATPLOTPP_BUILD_SHARED_LIBS=ON \
        -DMATPLOTPP_BUILD_TESTS=OFF \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
        ..
    make -j4 

    echo "Installing Matplot++ ..."
    sudo make install

    cd $ADMIN_DIR

elif [ "$answer" = "2" ]
then
    echo "installing in admin/reqLib/matplotpp"
    
    mkdir reqLib
    cd reqLib

    echo "Cloning Matplot++ ..."
    set +e
    git clone https://github.com/alandefreitas/matplotplusplus.git
    set -e

    echo "Building Matplot++ ..."
    cd matplotplusplus
    mkdir -p build
    cd build

    # set the install directory
    INSTALL_DIR=$(realpath ../../)
    echo "install directory: ${INSTALL_DIR}"
    
    # set the cmake flags for matplot
    cmake -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"  \
        -DMATPLOTPP_BUILD_EXAMPLES=OFF \
        -DMATPLOTPP_BUILD_SHARED_LIBS=ON \
        -DMATPLOTPP_BUILD_TESTS=OFF \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
        ..
    make -j4 

    echo "Installing Matplot++ ..."
    sudo make install
    
    # return to admin directory
    cd $ADMIN_DIR

else
    echo "please specify 1 or 2"
fi

echo ""
echo "Installation of required packages complete!"
echo "Run setup.sh to build the project"
echo "Happy hacking!"