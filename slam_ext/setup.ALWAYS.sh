#!/bin/bash

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

sudo apt-get install -y lib32z1 lib32ncurses5

cd $SCRIPTPATH/external/cmake && bash -x make.sh
