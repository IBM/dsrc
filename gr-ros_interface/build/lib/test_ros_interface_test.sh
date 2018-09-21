#!/bin/sh
export VOLK_GENERIC=1
export GR_DONT_LOAD_PREFS=1
export srcdir=/home/augustojv/catkin_ws/src/dsrc/gr-ros_interface/lib
export GR_CONF_CONTROLPORT_ON=False
export PATH=/home/augustojv/catkin_ws/src/dsrc/gr-ros_interface/build/lib:$PATH
export LD_LIBRARY_PATH=/home/augustojv/catkin_ws/src/dsrc/gr-ros_interface/build/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$PYTHONPATH
test-ros_interface 
