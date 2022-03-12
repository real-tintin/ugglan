#!/bin/sh
# Compiles the executable and packs in
# a Debian package (dpkg).
#

# Setup env for make and compile.
export TARGET_NAME="ugglan"
export BUILD_SUFFIX="dpkg"

make all -j8

# Build dpkg.
DPKG_NAME="ugglan"
DPKG_FILE="${DPKG_NAME}.deb"

SKEL_SRC_PATH="./dpkg"
SKEL_DST_PATH="./${DPKG_NAME}"

EXEC_SRC_PATH="./build_${BUILD_SUFFIX}/${TARGET_NAME}"
EXEC_DST_PATH="${SKEL_DST_PATH}/usr/local/bin/ugglan"

mkdir -p ${SKEL_DST_PATH}
mkdir -p ${EXEC_DST_PATH}

cp -r ${SKEL_SRC_PATH}/. ${SKEL_DST_PATH}
cp -r ${EXEC_SRC_PATH} ${EXEC_DST_PATH}

dpkg-deb --build ${SKEL_DST_PATH}
rm -r ${SKEL_DST_PATH}
