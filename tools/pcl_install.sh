#!/bin/bash
sudo apt-get update
cd ~
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build
cd build
cmake ..
make -j2
sudo make -j2 install