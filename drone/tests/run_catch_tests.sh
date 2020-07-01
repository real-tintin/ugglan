#!/bin/sh
# Compiles and runs catch C++ tests.
#

# Setup env for make.
export BUILD_FOR_TEST="TRUE"
export TARGET_NAME="catch"

# Setup paths.
SCRIPT=$(readlink -f "$0")
CWD=$(dirname "$SCRIPT")

DRONE_ROOT="${CWD}/.."
TARGET_PATH="./build/${TARGET_NAME}"

# Compile and run tests.
cd "${DRONE_ROOT}"
make all -j8
exit_code=$?

if [ $exit_code -eq 0 ]; then
    ${TARGET_PATH}
fi
