LOCAL_PATH := $(call my-dir)

$(info LOCAL_PATH: $(LOCAL_PATH))

SLAM_DIR_ROOT := ../../../../..
SLAM_DIR := $(SLAM_DIR_ROOT)/build.android

SLAM_DIR_ROOT_RELATIVE := ../..
SLAM_DIR_RELATIVE := $(SLAM_DIR_ROOT_RELATIVE)/build.android

######################### DIR

SUITESPARSEDIR:=$(SLAM_DIR)/suitesparse
G2ODIR:=$(SLAM_DIR)/g2o

BOOSTDIRANDROID:=$(SLAM_DIR_ROOT)/external/android/BoostforAndroid/build
BOOSTDIRANDROID_RELATIVE:=$(SLAM_DIR_ROOT_RELATIVE)/external/android/BoostforAndroid/build

OPENCVDIR:=$(SLAM_DIR_ROOT_RELATIVE)/external/android/OpenCV-Android/sdk/native/jni

ORBSLAMDIR:=$(SLAM_DIR)/slam
ORBSLAMDIR_RELATIVE:=$(SLAM_DIR_RELATIVE)/slam

DEFAULT_LIBS:=../default_libs
#WTENGINE_OUT:=$(WTENGINE_OUT)

$(info SUITESPARSEDIR: $(SUITESPARSEDIR))
$(info G2ODIR: $(G2ODIR))
$(info BOOSTDIRANDROID: $(BOOSTDIRANDROID))
$(info OPENCVDIR: $(OPENCVDIR))
$(info ORBSLAMDIR: $(ORBSLAMDIR))

######################### BOOST
define add_boost_component
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:=$(BOOSTDIRANDROID)/lib/lib$1.a
    include $(PREBUILT_STATIC_LIBRARY)
endef
BOOSTMODULES := boost_thread boost_system
$(foreach module, $(BOOSTMODULES), $(eval $(call add_boost_component,$(module))))

######################### G2O
define add_g2o_component
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:=$(G2ODIR)/lib/libg2o_$1.so
    include $(PREBUILT_SHARED_LIBRARY)
endef
G2OMODULES := core stuff solver_cholmod solver_dense solver_pcg\
              solver_structure_only solver_eigen types_data \
              types_icp types_sba types_sclam2d types_sim3 types_slam2d types_slam3d
$(foreach module, $(G2OMODULES), $(eval $(call add_g2o_component,$(module))))

######################### SUITESPARSE
define add_suitesparse_component
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:= $(SUITESPARSEDIR)/build/lib/lib$1.a
    include $(PREBUILT_STATIC_LIBRARY)
endef
SUITESPARSEMODULES := amd btf camd ccolamd cholmod colamd cxsparse klu ldl spqr suitesparseconfig umfpack
$(foreach module, $(SUITESPARSEMODULES), $(eval $(call add_suitesparse_component,$(module))))

######################### DBoW2

include $(CLEAR_VARS)
LOCAL_MODULE:=DBoW2
LOCAL_SRC_FILES:= $(ORBSLAMDIR)/lib/libDBoW2.so
include $(PREBUILT_SHARED_LIBRARY)

######################### ORBSLAM
include $(CLEAR_VARS)
ORBSLAMINCLUDE := $(ORBSLAMDIR)/Thirdparty/boost/include \
                  $(ORBSLAMDIR)/Thirdparty \
                  $(ORBSLAMDIR) \
                  $(ORBSLAMDIR)/Thirdparty/eigen3 \
                  $(ORBSLAMDIR)/include \
                  $(OPENCVDIR)/include

LOCAL_MODULE := orbslam
LOCAL_SRC_FILES := $(ORBSLAMDIR)/lib/libORB_SLAM.so
LOCAL_SHARED_LIBRARIES :=  $(SUITESPARSEMODULES) $(G2OMODULES)
LOCAL_C_INCLUDES :=  $(ORBSLAMINCLUDE)
include $(PREBUILT_SHARED_LIBRARY)

######################### WTEngine

include $(CLEAR_VARS)
LOCAL_MODULE:=WTEngine
LOCAL_SRC_FILES:=$(DEFAULT_LIBS)/libwtengine.so
include $(PREBUILT_SHARED_LIBRARY)



######################### APP
#APP_OPTIM:=debug
include $(CLEAR_VARS)
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=SHARED
OPENCVROOT:= $(OPENCVDIR)

include $(OPENCVROOT)/OpenCV.mk

BOOSTINCLUDE:= $(BOOSTDIRANDROID_RELATIVE)/include
BOOSTMODULES := boost_thread boost_system

ORBSLAMINCLUDE_RELATIVE = $(ORBSLAMDIR_RELATIVE)/Thirdparty/boost/include \
                  $(ORBSLAMDIR_RELATIVE)/Thirdparty \
                  $(ORBSLAMDIR_RELATIVE) \
                  $(ORBSLAMDIR_RELATIVE)/Thirdparty/eigen3 \
                  $(ORBSLAMDIR_RELATIVE)/include \
                  $(OPENCVDIR)/include

$(info ORBSLAMINCLUDE_RELATIVE: $(ORBSLAMINCLUDE_RELATIVE))

LOCAL_C_INCLUDES := $(LOCAL_PATH) $(ORBSLAMINCLUDE_RELATIVE) $(BOOSTINCLUDE)
LOCAL_MODULE    := slamengine

# profiling flags: LOCAL_CFLAGS := -pg -DNDK_PROFILE
LOCAL_CFLAGS := -DHAVE_NEON=1 -g -pg -DNDK_PROFILE
LOCAL_SRC_FILES := jni_part.cpp.neon android.h.neon
LOCAL_SHARED_LIBRARIES := orbslam cholmod DBoW2 android-ndk-profiler
# Profiling lib android-ndk-profiler
LOCAL_STATIC_LIBRARIES := $(foreach mod, $(OPENCV_MODULES), opencv_$(mod)) $(OPENCV_3RDPARTY_COMPONENTS) $(BOOSTMODULES)
LOCAL_LDLIBS += -llog -landroid -lc -lz -lm

include $(BUILD_SHARED_LIBRARY)

## PROFILER

include $(CLEAR_VARS)
$(call import-module,android-ndk-profiler)