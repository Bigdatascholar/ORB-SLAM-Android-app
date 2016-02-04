# SLAM Android compiling guide

## Prerequisites

* We use Ubuntu 14.04

### OpenCV

1. Clone our OpenCV repository:

		ssh://hg@bitbucket.org/worldtwoo/opencv

2. Run from a terminal

		./build.sh

Note: This module creates this file:

	/etc/profile.d/wt.opencv.sh

that sets **OPENCV_DIR** environment variable.
In case you have errors execute, on the terminal where you are compiling:

	source /etc/profile.d/wt.opencv.sh

### Boost

1. Clone our Boost repository:

		ssh://hg@bitbucket.org/worldtwoo/boost

2. Run from a terminal

		./build.sh

Note: This module creates this file:

	/etc/profile.d/wt.boost.sh

that sets **BOOST_DIR** environment variable.
In case you have errors execute, on the terminal where you are compiling:

	source /etc/profile.d/wt.boost.sh

## Building

1. Clone our SLAM repository: 

		ssh://wt_your_username@repo.worldtwoo.com//devrepo/slam_ext

2. Run from a terminal 
	
		./build.linux.sh

	from terminal to compile the code (**always use this script to build all the C++ code, even when changing it**).

3. To test it on  pc:

	1. Move to directory 

			build.linux/slam/slam_linux/out

	2. Run 

			slam_linux