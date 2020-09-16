#!/bin/sh
# Compiles and runs catch C++ tests.
#

# Setup env for make.
export BUILD_FOR_TEST="TRUE"
export TARGET_NAME="catch"

# Setup paths.
ROOT="/drone"
TARGET_PATH="${ROOT}/build/${TARGET_NAME}"

# Compile and run tests.
cd ${ROOT} && make all -j8
exit_code=$?

if [ $exit_code -eq 0 ]; then
    ${TARGET_PATH}
fi
