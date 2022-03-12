#!/bin/sh
# Compiles and runs catch C++ tests.
#

# Setup env.
export BUILD_FOR_TEST="TRUE"
export TARGET_NAME="catch"

export TEST_ROOT="${PWD}/tests"
export RESOURCES_ROOT="${TEST_ROOT}/resources"

# Compile and run tests.
export BUILD_SUFFIX="tests"
TARGET_PATH="./build_${BUILD_SUFFIX}/${TARGET_NAME}"

make all -j8
exit_code=$?

if [ $exit_code -eq 0 ]; then
    ${TARGET_PATH}
fi
