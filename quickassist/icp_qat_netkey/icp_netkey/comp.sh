#!/bin/bash

rm -f Module.symvers

export ICP_ROOT=~/co/router/quickassist/QAT1.5
export KERNEL_SOURCE_ROOT=~/co/router/base/build_x86/linux

make
