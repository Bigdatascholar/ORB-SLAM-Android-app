#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

cd $SCRIPTPATH/app && $ANDROID_NDK/ndk-build NDK_MODULE_PATH=../modules NDK_PROJECT_PATH=build/intermediates/ndk NDK_LIBS_OUT=src/main/libs APP_BUILD_SCRIPT=src/main/jni/Android.mk NDK_APPLICATION_MK=src/main/jni/Application.mk
