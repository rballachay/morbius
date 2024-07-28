#!/bin/sh

mkdir build
cd build
cmake ..
make
mv esaNet ../
cd ..

echo "Built executable: esaNet, must run with sudo"