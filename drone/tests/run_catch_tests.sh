#!/bin/sh

SCRIPT=$(readlink -f "$0")
CWD=$(dirname "$SCRIPT")

DRONE_ROOT="${CWD}/../"

TARGET_NAME="catch"

# Compile drone dependencies.
cd "${DRONE_ROOT}"
export DO_NOT_LINK="TRUE" && mingw32-make clean all

# Compile and run catch tests.
cd "${CWD}"
rm -f "${CWD}/${TARGET_NAME}"
export TARGET_NAME && mingw32-make && ./catch
