LOCAL_PATH := $(call my-dir)


######################### DIR
SUITESPARSEDIR:=/home/fab/freelance/suitesparse-metis-for-windows
G2ODIR:=/home/fab/freelance/g2o/trunk
BOOSTDIR:=/home/fab/freelance/Boost-for-Android
OPENCVDIR:=/home/fab/freelance/OpenCV-android-sdk/sdk/native
ORBSLAMDIR:=/home/fab/freelance/ORB_SLAM

######################### BOOST
define add_boost_component
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:=$(BOOSTDIR)/build/lib/lib$1-gcc-mt-1_53.a
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
G2OMODULES := core stuff cli solver_cholmod solver_csparse solver_dense solver_pcg\
              solver_slam2d_linear solver_structure_only solver_eigen types_data \
              types_icp types_sba types_sclam2d types_sim3 types_slam2d types_slam3d csparse_extension ext_csparse
$(foreach module, $(G2OMODULES), $(eval $(call add_g2o_component,$(module))))

######################### SUITESPARSE
define add_suitesparse_component
    include $(CLEAR_VARS)
    LOCAL_MODULE:=$1
    LOCAL_SRC_FILES:= $(SUITESPARSEDIR)/build/lib/lib$1.so
    include $(PREBUILT_SHARED_LIBRARY)
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
ORBSLAMINCLUDE := $(BOOSTDIR)/build/include/boost-1_53 \
                  $(ORBSLAMDIR)/Thirdparty \
                  $(ORBSLAMDIR) \
                  $(ORBSLAMDIR)/Thirdparty/eigen3 \
                  $(ORBSLAMDIR)/include \
                  $(OPENCVDIR)/jni/include

LOCAL_MODULE := orbslam
LOCAL_SRC_FILES := $(ORBSLAMDIR)/lib/libORB_SLAM.so
LOCAL_SHARED_LIBRARIES :=  $(SUITESPARSEMODULES) $(G2OMODULES)
LOCAL_C_INCLUDES :=  $(ORBSLAMINCLUDE)
include $(PREBUILT_SHARED_LIBRARY)

######################### APP
include $(CLEAR_VARS)
OPENCV_LIB_TYPE:=STATIC
OPENCVROOT:= $(OPENCVDIR)
include $(OPENCVROOT)/jni/OpenCV.mk

BOOSTINCLUDE:= $(BOOSTDIR)/build/include/boost-1_53
BOOSTMODULES := boost_thread boost_system

LOCAL_C_INCLUDES := $(LOCAL_PATH) $(ORBSLAMDIR)/include $(ORBSLAMINCLUDE) $(OPENCVROOT)/jni/include $(BOOSTINCLUDE)
LOCAL_MODULE    := orbslam_cv
LOCAL_CFLAGS := -DHAVE_NEON=1
LOCAL_SRC_FILES := jni_part.cpp
LOCAL_SHARED_LIBRARIES := orbslam cholmod DBoW2
LOCAL_STATIC_LIBRARIES := $(foreach mod, $(OPENCV_MODULES), opencv_$(mod)) $(OPENCV_3RDPARTY_COMPONENTS) $(BOOSTMODULES)
LOCAL_LDLIBS += -llog -landroid -lc -lz -lm

include $(BUILD_SHARED_LIBRARY)
