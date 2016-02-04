#pragma once

#ifdef ANDROID
#include <android/log.h>
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "ORB_SLAM", __VA_ARGS__))
#else
#include "stdio.h"
#define LOGI(...) printf(__VA_ARGS__); printf("\n")
#endif
