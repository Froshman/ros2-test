#!/bin/bash
MY_PYTHON_ENV=/home/gaming/anaconda3/envs/CondaEnvForRos/bin/python
colcon build  --ament-cmake-args -DPYTHON_EXECUTABLE=$MY_PYTHON_ENV
source ~/rec_ws/install/local_setup.bash
