# SLAM Android compiling guide

## Prerequisites

* We use Ubuntu 14.04
* You need to download [Android Studio](http://developer.android.com/tools/studio/index.html).

## Building

1. Clone our SLAM repository: 

		ssh://wt_your_username@repo.worldtwoo.com//devrepo/slam_ext

2. Run from a terminal 
	
		./build.android.sh

	from terminal to compile the code (**always use this script to build all the C++ code, even when changing it**).

3. To test the Android App:
	1. Open **app_android** folder with **Android Studio**.
	2. Edit **app_android/local.properties** file accordingly to your system: if NDK was downloaded by our script, its path will be

			/opt/android/ndk/android-ndk-r10e

	3. Build and test.