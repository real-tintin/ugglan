#!/bin/sh
# Compiles the executable and packs in
# a Debian package (dpkg).
#

# Setup env for make.
export TARGET_NAME="ugglan"

# Setup paths.
SCRIPT=$(readlink -f "$0")
CWD=$(dirname "$SCRIPT")

DRONE_ROOT="${CWD}/"
TARGET_PATH="./build/${TARGET_NAME}"

# Compile fresh executable.
cd "${DRONE_ROOT}"
make clean all -j8

# TODO: Build dpkg.
