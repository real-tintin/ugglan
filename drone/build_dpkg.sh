#!/bin/sh
# Compiles the executable and packs in
# a Debian package (dpkg).
#

# Setup env for make.
export TARGET_NAME="ugglan"

# Setup paths.
ROOT="/drone"
ROOT_MOUNTED="/drone_mount"

# Compile fresh executable.
cd ${ROOT} && make clean all -j8

# Build dpkg.
DPKG_NAME="ugglan"
DPKG_FILE="${DPKG_NAME}.deb"

DPKG_SRC_PATH="${ROOT}/dpkg"
DPKG_DST_PATH="${ROOT}/${DPKG_NAME}"

DPKG_EXEC_ROOT="${DPKG_DST_PATH}/usr/local/bin/ugglan"

mkdir -p ${DPKG_DST_PATH}
mkdir -p ${DPKG_EXEC_ROOT}
cp -r ${DPKG_SRC_PATH}/. ${DPKG_DST_PATH}

dpkg-deb --build ${DPKG_DST_PATH}
cp ${DPKG_FILE} ${ROOT_MOUNTED}
