#!/bin/bash

# Before:
# sudo apt-get -y install gcc-arm-linux-androideabi
# sudo apt-get -y install python graphviz
# sudo apt-get -y install python-pip
# sudo pip install gprof2dot


SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

LIBS_PATH=$SCRIPTPATH/app/build/intermediates/ndk/obj/local/armeabi-v7a

#LIBS=$LIBS_PATH/libslamengine.so
LIBS=$LIBS_PATH/libORB_SLAM.so

PROFILE_DIR=$SCRIPTPATH/profiletmp

mkdir $PROFILE_DIR

cd $PROFILE_DIR && adb pull /sdcard/gmon.out
cd $PROFILE_DIR && arm-linux-androideabi-gprof $LIBS > profile.txt
#cd $PROFILE_DIR && arm-linux-androideabi-gprof $LIBS | gprof2dot | dot -Tpng -o output.png
