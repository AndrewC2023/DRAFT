# setup the build directory

# remove the old build directory if there is one
# set -euxo pipefail

# It is recommended to run the required libraries script prior to this one
cd ..
sudo rm -r build

mkdir build
cd build
mkdir debug
mkdir release

set -e

cd release
cmake -DCMAKE_BUILD_TYPE=Release ../..
cd ..

cd debug
cmake -DCMAKE_BUILD_TYPE=Debug ../..
cd ..

# give the option to build the code
cd release

printf 'Would you like to build in release (y/n)? '
read answer

if [ "$answer" != "${answer#[Yy]}" ] ;then 
    echo "Building ..."
    make -j4
else
    echo ""
fi

echo "setup Complete"