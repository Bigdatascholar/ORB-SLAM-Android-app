#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

EXTERNALDIR=$SCRIPTPATH/external

BUILD_DIR=$SCRIPTPATH/build.android

mkdir -p $BUILD_DIR

ANDROIDRESOURCES=$EXTERNALDIR/android

ANDROIDCMAKE_SRC=$ANDROIDRESOURCES/android-cmake
ANDROIDCMAKE=$BUILD_DIR/android-cmake
cp -R -u -v $ANDROIDCMAKE_SRC/. $ANDROIDCMAKE

source /etc/profile.d/wt.android.ndk.sh
# If NDK doesn't exist, downloads it
if [ -z "$ANDROID_NDK" ]
then

ANDROID_NDK_BASEDIR=/opt/android/ndk
sudo mkdir -p $ANDROID_NDK_BASEDIR

cd $ANDROID_NDK_BASEDIR && sudo wget -nc https://mil01.objectstorage.softlayer.net/v1/AUTH_8def9beb-78ec-4088-8ad0-34a26f1156fc/public/android-ndk-r10e-linux-x86_64.bin
cd $ANDROID_NDK_BASEDIR && sudo chmod a+x android-ndk-r10e-linux-x86_64.bin
cd $ANDROID_NDK_BASEDIR && sudo ./android-ndk-r10e-linux-x86_64.bin
cd $ANDROID_NDK_BASEDIR && sudo chmod 555 -R android-ndk-r10e

ANDROID_NDK=$ANDROID_NDK_BASEDIR/android-ndk-r10e

cat << EOF | sudo tee /etc/profile.d/wt.android.ndk.sh
export ANDROID_NDK=$ANDROID_NDK
EOF

source /etc/profile.d/wt.android.ndk.sh

fi

BOOST_DIR_ANDROID_ROOT=$ANDROIDRESOURCES/BoostforAndroid
BOOST_DIR_ANDROID=$BOOST_DIR_ANDROID_ROOT/build
BOOST_DIR_ANDROID_INCLUDE=$BOOST_DIR_ANDROID/include/boost/thread.hpp

if [ ! -f $BOOST_DIR_ANDROID_INCLUDE ]; then
cd $BOOST_DIR_ANDROID && tar -zxvf include.tar.gz
fi

#BOOST_DIR=$BOOST_DIR_ANDROID_ROOT/boost_1_53_0

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
cp -R -u -v $SLAMDIR_SRC/Thirdparty/. $SLAMDIR/Thirdparty
cp -R -u -v $SLAMDIR_SRC/cmake_modules/. $SLAMDIR/cmake_modules
cp -R -u -v $SLAMDIR_SRC/CMakeLists.txt $SLAMDIR/CMakeLists.txt

ANDROIDOPENCV=$ANDROIDRESOURCES/OpenCV-Android/sdk/native/jni

cat << EOF | sudo tee /etc/profile.d/wt.orbslam.android.sh
export ANDROID_CMAKE=$ANDROIDCMAKE
export ANDTOOLCHAIN=$ANDROIDCMAKE/android.toolchain.cmake

export CLAPACKDIR_ANDROID=$CLAPACKDIR
export SUITESPARSEDIR_ANDROID=$SUITESPARSEDIR
export G2ODIR_ANDROID=$G2ODIR
export G2O_ROOT_ANDROID=$G2ODIR
export SLAMDIR_ANDROID=$SLAMDIR
export ANDROIDOPENCV_ANDROID=$ANDROIDOPENCV
export BOOST_DIR_ANDROID=$BOOST_DIR_ANDROID
export ORB_SLAM_ANDROID_DIR=$SLAMDIR
EOF

source /etc/profile.d/wt.orbslam.android.sh

export CLAPACKDIR=$CLAPACKDIR
export SUITESPARSEDIR=$SUITESPARSEDIR
export G2ODIR=$G2ODIR
export G2O_ROOT=$G2ODIR
export SLAMDIR=$SLAMDIR
export BOOST_DIR=$BOOST_DIR_ANDROID

# Build NDK profiler

ANDROID_APP_DIR=$SCRIPTPATH/app_android

FORCE_REBUILD=false
ADD_PROFILER=false
BUILD_TYPE=Release

# Parameters
while [ "$1" != "" ]; do
    if [ "$1" == "force" ]; then
	echo "Force rebuild"
	FORCE_REBUILD=true
    fi

    if [ "$1" == "debug" ]; then
	echo "Debug mode - profiler"
	ADD_PROFILER=true
	BUILD_TYPE=Debug
    fi

    # Shift all the parameters down by one
    shift

done

if [ "$ADD_PROFILER" == "true" ]; then
cd $ANDROID_APP_DIR && ./build.ndk.profiler.sh BUILD_SO
export BUILD_ANDROID_PROFILER_DIR=$ANDROID_APP_DIR/modules/build/libs/armeabi-v7a
export BUILD_ANDROID_PROFILER_LIB=$BUILD_ANDROID_PROFILER_DIR/libandroid-ndk-profiler.so

echo "Set profiler lib dir: $BUILD_ANDROID_PROFILER_DIR"

else
unset BUILD_ANDROID_PROFILER_LIB
fi

# END
shopt -s expand_aliases
# /mnt/Documents/Developing/ViewTwoo/orb_slam/build.android/android-cmake/android.toolchain.cmake

echo "Android NDK: $ANDROID_NDK"
alias androidcmake="cmake -DBUILD_ANDROID=TRUE -DCMAKE_TOOLCHAIN_FILE=$ANDTOOLCHAIN -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DANDROID_NDK=$ANDROID_NDK -DANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-4.9 -DANDROID_ABI=\"armeabi-v7a with NEON\" -DANDROID_NATIVE_API_LEVEL=19 -DANDROID_NDK_TOOLCHAINS_PATH=$ANDROID_NDK"

mkdir -p $BUILD_DIR

mkdir -p $CLAPACKDIR_BUILD
cd $CLAPACKDIR_BUILD
rm $CLAPACKDIR_BUILD/CMakeCache.txt

if [ ! -f $CLAPACKDIR_BUILD/BLAS/SRC/libblas.a ] || [ $FORCE_REBUILD ]; then

	androidcmake ..

make -j4
make -j4
fi

mkdir -p $SUITESPARSEDIR_BUILD
cd $SUITESPARSEDIR_BUILD
rm $SUITESPARSEDIR_BUILD/CMakeCache.txt

if [ ! -f $SUITESPARSEDIR_BUILD/lib/libamd.a ] || [ $FORCE_REBUILD ]; then

	androidcmake ..

make -j4
fi

mkdir -p $G2ODIR_BUILD
cd $G2ODIR_BUILD
rm $G2ODIR_BUILD/CMakeCache.txt

if [ ! -f $G2ODIR_BUILD/../lib/libg2o_core.so ] || [ $FORCE_REBUILD ]; then

	androidcmake ..

make -j4
fi

mkdir -p $SLAMDIR_BUILD
cd $SLAMDIR_BUILD
rm $SLAMDIR_BUILD/CMakeCache.txt

	androidcmake ..

make -j4

