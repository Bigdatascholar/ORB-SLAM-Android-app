#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

BUILD_SO=
if [ "$1" == "BUILD_SO" ]; then
BUILD_SO="BUILD_SO=true"
fi

cd $SCRIPTPATH/modules && $ANDROID_NDK/ndk-build NDK_PROJECT_PATH=build/intermediates NDK_LIBS_OUT=build/libs APP_BUILD_SCRIPT=android-ndk-profiler/Android.mk NDK_APPLICATION_MK=android-ndk-profiler/Application.mk $BUILD_SO
