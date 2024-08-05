#!/bin/bash

# install pistache
if [ "$(uname)" == "Darwin" ]; then
  brew install meson
  brew install doxygen
  brew install googletest
  brew install openssl
  brew install rapidjson
  brew install howard-hinnant-date
fi


git submodule update --init submodules/pistache/
cd submodules/pistache/
bldscripts/mesbuild.sh
bldscripts/mesinstall.sh
cd ../../

# INTO SUBMODULE
cd submodules/ORB_SLAM3

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
cd ../


# OUT OF SUBMODULE
cd ../../

echo "Configuring and building vision ..."

mkdir -p build
cd build
cmake .. 
make
mv vision_nav.cpython-310-darwin.so ../
cd ../