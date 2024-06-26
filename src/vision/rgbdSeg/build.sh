#!/bin/sh

cd include/MRF2.2
make
cd ../../
mkdir build
cd build
cmake ..
make
mv planeSegment ../
cd ..

echo "Built executable: planeSegment, must run with sudo"