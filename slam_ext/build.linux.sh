#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

EXTERNALDIR=$SCRIPTPATH/external

BUILD_DIR=$SCRIPTPATH/build.linux

mkdir -p $BUILD_DIR

if [ -z "$BOOST_DIR" ]
then
echo "ERROR: Could not find Boost DIR. Please set BOOST_DIR Env variable."
exit;
else
echo "Found Boost DIR: $BOOST_DIR"
fi

BOOST_DIR=$BOOST_DIR

if [ -z "$OPENCV_DIR" ]
then
echo "ERROR: Could not find OpenCV DIR. Please set OPENCV_DIR Env variable."
exit;
else
echo "Found OpenCV DIR"
fi

CLAPACKDIR_SRC=$EXTERNALDIR/clapack-3.2.1-CMAKE
CLAPACKDIR=$BUILD_DIR/clapack
CLAPACKDIR_BUILD=$CLAPACKDIR/build
cp -R -u -v $CLAPACKDIR_SRC/. $CLAPACKDIR

SUITESPARSEDIR_SRC=$EXTERNALDIR/suitesparse
SUITESPARSEDIR=$BUILD_DIR/suitesparse
SUITESPARSEDIR_BUILD=$SUITESPARSEDIR/build
cp -R -u -v $SUITESPARSEDIR_SRC/. $SUITESPARSEDIR

G2ODIR_SRC=$EXTERNALDIR/g2o/trunk
G2ODIR=$BUILD_DIR/g2o
G2ODIR_BUILD=$G2ODIR/build
cp -R -u -v $G2ODIR_SRC/. $G2ODIR

SLAMDIR_SRC=$SCRIPTPATH/.
SLAMDIR=$BUILD_DIR/slam
SLAMDIR_BUILD=$SLAMDIR/build

mkdir -p $SLAMDIR/Thirdparty
cp -R -u -v $SLAMDIR_SRC/include/. $SLAMDIR/include
cp -R -u -v $SLAMDIR_SRC/src/. $SLAMDIR/src
cp -R -u -v $SLAMDIR_SRC/slam_linux/. $SLAMDIR/slam_linux
cp -R -u -v $SLAMDIR_SRC/Thirdparty/. $SLAMDIR/Thirdparty
cp -R -u -v $SLAMDIR_SRC/cmake_modules/. $SLAMDIR/cmake_modules
cp -R -u -v $SLAMDIR_SRC/CMakeLists.txt $SLAMDIR/CMakeLists.txt

#export BOOSTDIR=$BOOSTDIR
cat << EOF | sudo tee /etc/profile.d/wt.orbslam.linux.sh

export CLAPACKDIR_LINUX=$CLAPACKDIR
export SUITESPARSEDIR_LINUX=$SUITESPARSEDIR
export G2ODIR_LINUX=$G2ODIR
export G2O_ROOT_LINUX=$G2ODIR
export SLAMDIR_LINUX=$SLAMDIR

export ORB_SLAM_LINUX_DIR=$SLAMDIR
EOF

source /etc/profile.d/wt.orbslam.linux.sh

export CLAPACKDIR=$CLAPACKDIR
export SUITESPARSEDIR=$SUITESPARSEDIR
export G2ODIR=$G2ODIR
export G2O_ROOT=$G2ODIR
export SLAMDIR=$SLAMDIR

mkdir -p $BUILD_DIR

mkdir -p $CLAPACKDIR_BUILD
cd $CLAPACKDIR_BUILD
rm $CLAPACKDIR_BUILD/CMakeCache.txt

if [ ! -f $CLAPACKDIR_BUILD/BLAS/SRC/libblas.a ]; then
cmake $CLAPACKDIR && make -j4
fi

mkdir -p $SUITESPARSEDIR_BUILD
cd $SUITESPARSEDIR_BUILD
rm $SUITESPARSEDIR_BUILD/CMakeCache.txt

if [ ! -f $SUITESPARSEDIR_BUILD/lib/libamd.a ]; then
cmake $SUITESPARSEDIR && make -j4
fi

mkdir -p $G2ODIR_BUILD
cd $G2ODIR_BUILD
rm $G2ODIR_BUILD/CMakeCache.txt

if [ ! -f $G2ODIR_BUILD/../lib/libg2o_core.so ]; then
cmake $G2ODIR && make -j4
fi

mkdir -p $SLAMDIR_BUILD
rm $SLAMDIR_BUILD/CMakeCache.txt
cd $SLAMDIR_BUILD && cmake $SLAMDIR && make -j4


