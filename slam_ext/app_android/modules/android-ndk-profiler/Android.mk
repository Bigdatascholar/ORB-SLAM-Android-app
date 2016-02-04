LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_CFLAGS := -fvisibility=hidden -DDEBUG
LOCAL_MODULE    := android-ndk-profiler
LOCAL_SRC_FILES := gnu_mcount.S prof.c read_maps.c

#ifeq ($(BUILD_SO),true)
	LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -llog
	include $(BUILD_SHARED_LIBRARY)
#else
#	include $(BUILD_STATIC_LIBRARY)
#endif


