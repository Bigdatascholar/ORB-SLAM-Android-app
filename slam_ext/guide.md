### Requirements

- [Android-CMake] (https://github.com/taka-no-me/android-cmake)
- [CMake / ccmake] (http://www.cmake.org): minimum version 3.2.2
- [Boost-for-Android] (https://github.com/MysticTreeGames/Boost-for-Android): version used 1:54
- [opencv4android](http://docs.opencv.org/doc/tutorials/introduction/android_binary_package/O4A_SDK.html)
- [G2O] (https://openslam.org/g2o.html)
- [SuiteSparse-CMake] (https://github.com/jlblancoc/suitesparse-metis-for-windows)
- [CLapack] (http://www.netlib.org/clapack/)
- [Eigen] (http://eigen.tuxfamily.org/index.php?title=Main_Page): minimum version 3.2.1
- [Sophus] (https://github.com/strasdat/Sophus): contained in LSD-slam

## Android SDK / NDK

Download the SDK and NDK android from the official websites and save them in an easily accessible directory. The versions used are respectively SDK API level 19 and NDK r10d.
At this point it is convenient to set environment variables that contain the reference directory of the entire ecosystem java / android. To do this, we go to export certain environment variables directly at the end of `~ / .bashrc` system files:

|
| Other things already in the file
|

############### ANDROID DEVELOPMENT ##################
export JAVA_HOME = / usr / lib / jvm / java-8-oracle
export JDK_PATH = / usr / lib / jvm / java-8-oracle / bin
export ANDROID_SDK = / home / fab / adt-bundle-linux-x86_64-20140702 / sdk
export ANDROID_NDK = / home / fab / android-ndk-r10d
export PATH = $ PATH: $ JAVA_HOME: $ JDK_PATH: $ ANDROID_SDK / tools: $ ANDROID_SDK / platform-tools: $ ANDROID_NDK

`` ANDROID_NDK` ANDROID_SDK` and reference folders in the home of my personal user so be careful. After that, the SDK and dell'NDK paths are now easily accessible from each system. If spostiate a folder or you changed version you have no choice but to change the corresponding path.

## Android-CMake

The C in Linux software projects are typically developed using the compilation tools that facilitate the management of many files. The most widely used of all is surely CMake. You can use the same tool for compiling c libraries and applications compatible with Android. Thanks to Android-CMake ** ** you can easily manage the completion of an entire project using the compilers (eg `gcc`) distributed by Android rather than those for` x86` systems.

Download ** ** Android-CMake at the official site and save it in an easily accessible directory. Also for convenience let's add more environment variables to our system:

export ANDROID_CMAKE = / home / fab / freelance / android-cmake

** ** Android-CMake toolchain provides a * programming * that automatically goes to refer to the compilers Android, it is therefore useful to refer to this toolchain with an environment variable.

export ANDTOOLCHAIN ​​= $ ANDROID_CMAKE / android.toolchain.cmake

At this point you can use `cmake` or its visual version` ccmake` to compile a project c Android. To do this, however, you have to define to the command line variables that tell cmake which toolchain and which compiler to use. is therefore convenient to define the alias * * complicated controls that save typing time and minimize errors, these aliases make use of environment variables defined so previously independent from where you installed the various packages. Still in `~ / .bashrc` add:

Android aka-cmake = 'cmake -DCMAKE_TOOLCHAIN_FILE = $ ANDTOOLCHAIN ​​-DCMAKE_MAKE_PROGRAM = $ ANDROID_NDK / prebuilt / linux-x86_64 / bin / make'
aka Android-ccmake = 'ccmake -DCMAKE_TOOLCHAIN_FILE = $ ANDTOOLCHAIN ​​-DCMAKE_MAKE_PROGRAM = $ ANDROID_NDK / prebuilt / linux-x86_64 / bin / make'

So the command to launch cmake or ccmake renumbered `$ android-cmake <DIRECTORY_CONTENTE_CMAKELIST.TXT>` and `$ android-ccmake <DIRECTORY_CONTENTE_CMAKELIST.TXT>`.

#### FindCmake

Android-CMake allows you to still use the cmake modules to find the libraries in the system without having to set them by hand every time. To do this you must use the cmake command instead of `` find_host_package` find_package`.

# Compiling a library

ORB_SLAM, in the version * * non-ros, has as its only direct external dependencies OpenCV, G2O, Boost, Eigen and SuiteSparse. Each of these libraries must be completed separately for Android devices. G2O internally provides a version of CSparse while to compile SuiteSparse we need lapack and blas libraries, both contained in CLapack package. Let's start with the latter and then go up the chain of dependencies.

## CLapack

CLapack download from the official site and save it in a convenient location.

export CLAPACKDIR = / home / fab / freelance / clapack

CLapack provides the source code of three libraries: F2C, BLAS and LAPACK. In order to fill in the proper mode for Android libraries must be made some changes to the source code and CMakeLists library.

In `$ CLAPACKDIR / CMakeLists.txt` file

- Comment the line 3, 4, 14 and 24, which refer to the testing
- Added before `if (WIN32 AND NOT CYGWIN)` the following lines:
`Set (ANDROID TRUE)`
`Set (BUILD_ANDROID TRUE)`
`Set (CMAKE_POSITION_INDEPENDENT_CODE ON)`


In `$ CLAPACKDIR file / SRC / CMakeLists.txt`

- Line 51, delete `../ INSTALL / lsame.c`

In `$ CLAPACKDIR file / BLAS / CMakeLists.txt`

- Comment `add_subdirectory (TESTING)`

In `$ CLAPACKDIR file / F2CLIBS / libf2c / CMakeLists.txt`

- Line 2, delete main.c

Now you can compile the library. Position yourself in `$ CLAPACKDIR` and

mkdir build
cd build
Android-ccmake ..

press '* c *' At this point to configure cmake and set:

- `ANDROID_ABI -> armeabi-V7A with NEON`
- `ANDROID_NATIVE_API_LEVEL -> 19`
- `CMAKE_BUILD_TYPE -> Release`

press '* c *' to reconfigure, and then press '* g *' to generate the Makefile ** **. Run the command `cmake --build .` to start compiling. It will return an error like `arithchk: not found` but do not worry about it. Relaunch the make command to overcome the error.
At this point, if there were no errors, the three libraries were generated (static) of the package:

- `$ CLAPACKDIR / build / SRC / liblapack.a`
- `$ CLAPACKDIR / build / BLAS / SRC / libblas.a`
- `$ CLAPACKDIR / build / F2CLIBS / libf2c / libf2c.a`

## SuiteSparse

The version of SuiteSparse required is to prepared with CMakeList.txt that can be found [here] (https://github.com/jlblancoc/suitesparse-metis-for-windows).

$ Git clone https://github.com/jlblancoc/suitesparse-metis-for-windows.git

export the environment variable in `~ / .bashrc`

export SUITESPARSEDIR = / home / fab / freelance / suitesparse-metis-for-windows

In `$ SUITESPARSEDIR / CMakeLists.txt` file

- 161 line, change `` off` with ON`
- 164-165 line, add `the SET (SUITESPARSE_CUSTOM_F2C_LIB" "CACHE FILE" Path to custom library files for F2C ")`
- 163 line, change `" "` `with" $ ENV {} CLAPACKDIR /build/BLAS/SRC/libblas.a "`
- 164 line, change `" "` `with" $ ENV {} CLAPACKDIR /build/SRC/liblapack.a "`
- 165 line, change `" "` `with" $ ENV {} CLAPACKDIR /build/F2CLIBS/libf2c/libf2c.a "`
- Line 169, change with `the SET (SuiteSparse_LINKER_LAPACK_BLAS_LIBS SUITESPARSE_CUSTOM_LAPACK_LIB $ {} $ {} SUITESPARSE_CUSTOM_BLAS_LIB SUITESPARSE_CUSTOM_F2C_LIB $ {})`

In `$ SUITESPARSEDIR file / SuiteSparse / CMakeLists.txt`

- Line 5, change `-mtune = native` with` -mtune = generic-ARMv7-a -mfpu = neon`
- Line 6, edit `-mtune = native` with` -mtune = generic-ARMv7-a -mfpu = neon`

In `$ SUITESPARSEDIR file / SuiteSparse / SuiteSparse_config / CMakeLists.txt`

- Line 9, add `target_link_libraries (suitesparseconfig m)`

Now you can compile the library. Position yourself in `$ SUITESPARSEDIR` and

mkdir build
cd build
Android-ccmake ..

Now press '* c *' to configure cmake, wait for the download of some files and set

- `ANDROID_ABI -> armeabi-V7A with NEON`
- `ANDROID_NATIVE_API_LEVEL -> 19`
- `CMAKE_BUILD_TYPE -> Release`
- `SHARED -> ON`

press '* c *' to reconfigure, and then press '* g *' to generate the Makefile ** **. Run the command `cmake --build .` to start compiling.
If there are no errors the different dynamic libraries of SuiteSparse packages were generated.

## G2O

G2O is the most popular c library to slam, both 2D and 3D. The strength of G2O is the internal integration of different mathematical solvers for the management of different types of slam. G2O depends on SuiteSparse and in particular from `` csparse` cholmod` and libraries. The second is not, however, indispensable because the library contains an internal version that, for reasons of compatibility, it is preferable to use.

we download the latest version of the site G2O

svn co https://svn.openslam.org/data/svn/g2o

We define the environment variable

export G2ODIR = / home / fab / freelance / G2O / trunk

In `$ G2ODIR / CMakeLists.txt` file

replace lines 107, 108 and 109 with

set (ENV {} CHOLMODDIR "$ ENV {SUITESPARSEDIR} / build / lib")
set (ENV {} CHOLMODINCLUDEDIR "$ ENV {} SUITESPARSEDIR / SuiteSparse / CHOLMOD / Includes")
include_directories ($ ENV {} SUITESPARSEDIR / SuiteSparse / SuiteSparse_config)
find_host_package (Cholmod)
set (BLAS_LIBRARIES_DIR $ ENV {} CLAPACKDIR / BLAS / SRC)
set (BLAS_LIBRARIES $ ENV {} CLAPACKDIR /build/BLAS/SRC/libblas.a)
find_host_package (BLAS)
set (LAPACK_LIBRARIES_DIR CLAPACKDIR $ ENV {} / build / SRC)
set (LAPACK_LIBRARIES $ ENV {} CLAPACKDIR /build/SRC/liblapack.a)
find_host_package (LAPACK)

- Replace `FIND_PACKAGE (CSparse)` to the line 124 (after changing over) with `the SET (CSPARSE_FOUND FALSE)`
- Comment the lines 143 and 144 to disable `openMP`
- Comment the lines 154 and 155 to disable `openGL`
- Comment the line 163 to disable `QGLViewer`
- Comment the line 176 to disable the compilation of examples G2O
- Replace `FIND_PACKAGE (Eigen3) at line 226 with` `find_host_package (Eigen3)`
- Add `the SET (G2O_LIB_TYPE SHARED)` after the line 241

In `$ G2ODIR file / cmake_modules / FindCholmod.cmake`

- Add `$ ENV {} CHOLMODINCLUDEDIR` before `$ ENV {}` CHOLMODDIR on line 12
- Comment the lines 15, 16, 17

In `$ G2ODIR files / EXTERNAL / Ceres / fpclassify.h`

- Replace `isnan (x)` `isnormal (x)` `isFinite (x)` and `isinf (x)` with `std :: isnan (x)` `std :: isnormal (x)`, `std :: isFinite (x)` and `std :: isinf (x)` in rows 62, 63, 76 and 77.

In `$ G2ODIR file / G2O / stuff / macros.h`

- Comment on the lines 51 to 54

Now you can compile the library. Position yourself in `$ G2ODIR` and

mkdir build
cd build
Android-ccmake ..

Now press '* c *' to configure cmake, ignore the warning by pressing '* and *' and set

- `ANDROID_ABI -> armeabi-V7A with NEON`
- `ANDROID_NATIVE_API_LEVEL -> 19`
- `CMAKE_BUILD_TYPE -> Release`

press '* c *' to reconfigure, '* and *' to bypass the warning if any, and press '* g *' to generate the Makefile ** **. Run the command `cmake --build .` to start compiling.

## Boost

To compile the boost just follow the guide on the official website

https://github.com/MysticTreeGames/Boost-for-Android

export the environment variable

export BOOSTDIR = / home / fab / freelance / Boost-for-Android

In the file `$ BOOSTDIR / build / include / boost-1_53 / boost / thread / pthread / thread_data.hpp`

- Change line 28 to `#if 0`

## OpenCV

Even for OpenCV follow the steps of the guide for downloading precompiled libraries

http://docs.opencv.org/doc/tutorials/introduction/android_binary_package/O4A_SDK.html

export the environment variable

export OPENCVDIR = / home / fab / freelance / OpenCV-android-sdk / sdk / native

## ORB_SLAM

export the environment variable

export ORBSLAMDIR = / home / fab / freelance / ORB_SLAM

Replace the files `$ ORBSLAMDIR / thirdparty / G2O / G2O / types / sba / types_six_dof_expmap.cpp`,` $ ORBSLAMDIR / thirdparty / G2O / G2O / types / sba / types_six_dof_expmap.h`, `$ ORBSLAMDIR / thirdparty / G2O / G2O / types / a3 / types_seven_dof_expmap.cpp`, `$ ORBSLAMDIR / thirdparty / G2O / G2O / types / a3 / types_seven_dof_expmap.h` with the fees in the package folder G2O and recompile it.

Position yourself in `$ ORBSLAMDIR` and

mkdir build
cd build
Android-ccmake ..

Now press '* c *' to configure and set cmake

- `ANDROID_ABI -> armeabi-V7A with NEON`
- `ANDROID_NATIVE_API_LEVEL -> 19`
- `CMAKE_BUILD_TYPE -> Release`

press '* c *' to reconfigure, and then press '* g *' to generate the Makefile ** **. Run the command `cmake --build .` to start compiling.

## GENERATION DICTIONARY

in `$ ORBSLAMDIR / thirdparty / DBoW2 / demo / demo.cpp` is the generation code of vocabulary. In this file you can set the number of images to use the associated filename format.

to compile the tool just bring in `$ ORBSLAMDIR / thirdparty / and DBoW2`

mkdir build
cd build
cmake ..
make -j5

then launch the tool with the `. / demo`, the images must be placed in the` $ ORBSLAMDIR / thirdparty / DBoW2 / demo`.
