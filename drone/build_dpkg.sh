#!/bin/sh
# Compiles the executable and packs in
# a Debian package (dpkg).
#

# Setup env for make and compile.
export TARGET_NAME="ugglan"
cd ${ROOT} && make clean all -j8

# Build dpkg.
DPKG_NAME="ugglan"
DPKG_FILE="${DPKG_NAME}.deb"

DPKG_SRC_PATH="${ROOT}/dpkg"
DPKG_DST_PATH="${ROOT}/${DPKG_NAME}"

DPKG_EXEC_ROOT="${DPKG_DST_PATH}/usr/local/bin/ugglan"
DPKG_EXEC_PATH="${ROOT}/build/${TARGET_NAME}"

mkdir -p ${DPKG_DST_PATH}
mkdir -p ${DPKG_EXEC_ROOT}

cp -r ${DPKG_SRC_PATH}/. ${DPKG_DST_PATH}
cp -r ${DPKG_EXEC_PATH} ${DPKG_EXEC_ROOT}

dpkg-deb --build ${DPKG_DST_PATH}
cp ${DPKG_FILE} ${ROOT_MOUNTED}
