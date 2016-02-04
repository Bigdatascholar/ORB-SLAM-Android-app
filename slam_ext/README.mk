# README #

This repository contains ViewTwoo's SLAM system code, which needs some possible improvements.
Let us know which of them you like, or if there are any others coming to your mind.

Please Fork and send push requests when modifying :)

### Needed improvements

* GPU Optimization: we want some parts of the code to be optimized for mobile GPU usage. Our first target is **ORBExtractor.cc** to be rewritten entirely on GPU. Then, the entire system.
    * On **Android** we're looking for **RenderScript** implementation
    * We still need to find a good solution for **iOS**
* Camera image processing:
    * We'd like to be able to handle 1920x1080 images (showing on the screen 1080p and using lower resolution for SLAM). This requires the image to be resized with GPU processing. Right now everything is done on Java, which is slow.
    * Color field normalization: to prevent "bumps" in colors, we'd like to normalize camera image colors by compression and later decompression of it.

The actual work flow is:

1. The image gets captured from a Java camera view (it is imposed to get a low res image, ex. 640x480), and transposed automatically to OpenCV mat
2. This mat gets resized (to half of captured source one), converted to gray and passed to SLAM system
3. The SLAM system computes all features extraction

What we would like to achieve is:

1. The image gets captured by native camera and its reference stored in the GPU
2. RenderScript takes the reference and does the resizing and b/w conversion
3. RenderScript extracts all the features
4. Features are sent to SLAM system

### Optional improvements

* Camera image processing:
    * Using more RGB channels. For now, the SLAM system uses only gray channel. Using more could improve its stability.
