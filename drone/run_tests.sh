#!/bin/sh
# Compiles and runs catch C++ tests.
#

# Setup env for make.
export BUILD_FOR_TEST="TRUE"
export TARGET_NAME="catch"

# Setup env for catch.
export CATCH_TEST_GETENV="test_getenv"

# Compile and run tests.
TARGET_PATH="${ROOT}/build/${TARGET_NAME}"

cd ${ROOT} && make all -j8
exit_code=$?

if [ $exit_code -eq 0 ]; then
    ${TARGET_PATH}
fi
