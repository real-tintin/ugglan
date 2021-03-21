#!/bin/sh
# Compiles and runs catch C++ tests.
#

# Setup env for make.
export BUILD_FOR_TEST="TRUE"
export TARGET_NAME="catch"

# Compile and run tests.
TARGET_PATH="./build/${TARGET_NAME}"

make all -j8
exit_code=$?

if [ $exit_code -eq 0 ]; then
    ${TARGET_PATH}
fi
