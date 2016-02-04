LOCAL_PATH := $(call my-dir)

# platform-12 defines Dl_info in <dlfcn.h>
ifeq (,$(call gte,$(APP_PLATFORM_LEVEL),12))
$(error Platform must be 12 or higher)
endif

include $(CLEAR_VARS)

LOCAL_MODULE := android-dl
LOCAL_SRC_FILES := \
	android-dl.cpp \
	jni.cpp \
	common.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_CFLAGS += -W -Wall -fno-rtti -fno-exceptions -DHAVE_NEON=1 -g
# GoogleTest requires a fully-functional C++ library
# Otherwise, we only use C++ as a "better C"
ifeq ($(APP_STL),none)
LOCAL_CFLAGS += -nodefaultlibs 
endif
LOCAL_LDLIBS := -llog
# Ensure our dependees can include <android-dl.h> too
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/inc
LOCAL_SHARED_LIBRARIES := dl log

include $(BUILD_SHARED_LIBRARY)