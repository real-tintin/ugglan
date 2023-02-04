#!/bin/sh
# Builds the ugglan Debian package (dpkg).
#

DPKG_NAME="ugglan"

BUILD_ROOT="./build_${BUILD_SUFFIX}"

SKEL_SRC_PATH="./dpkg"
SKEL_DST_PATH="${BUILD_ROOT}/${DPKG_NAME}"

EXEC_SRC_PATH="${BUILD_ROOT}/${TARGET_NAME}"
EXEC_DST_PATH="${SKEL_DST_PATH}/usr/local/bin/ugglan/"

rm -rf ${SKEL_DST_PATH}

mkdir -p ${SKEL_DST_PATH}
mkdir -p ${EXEC_DST_PATH}

cp -r ${SKEL_SRC_PATH}/. ${SKEL_DST_PATH}
cp ${EXEC_SRC_PATH} ${EXEC_DST_PATH}

dpkg-deb --build ${SKEL_DST_PATH}
