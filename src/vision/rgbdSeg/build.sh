#!/bin/sh

export OpenCV_DIR="/Library/Frameworks/Python.framework/Versions/3.12/lib/python3.12/site-packages/cv"
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