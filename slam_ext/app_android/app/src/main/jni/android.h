
#include <jni.h>
#include <android/log.h>

#ifndef ANDROID_H_COMMON
#define ANDROID_H_COMMON

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "slam_cv", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "slam_cv", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "slam_cv", __VA_ARGS__))

#endif
