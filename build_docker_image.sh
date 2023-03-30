#!/bin/sh
cd catkin_ws
git submodule init
git submodule update
cd ..

docker build -t robotics-test_solution .