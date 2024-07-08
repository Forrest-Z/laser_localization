#!/bin/bash

BUILD_TYPE=$1
WITH_VIZ=$2

if [ -z "$BUILD_TYPE" ]
then
#	BUILD_TYPE="Release"
  BUILD_TYPE="Debug"
fi

if [ -z "$WITH_VIZ" ]
then
#	WITH_VIZ=ON
	WITH_VIZ=OFF
fi

GENERATOR="Unix Makefiles"
SRC_DIR=$(pwd)
EXT_SRC_DIR="${SRC_DIR}/external"
BUILD_DIR="${SRC_DIR}/cmake-build-${BUILD_TYPE}"
EXT_BUILD_DIR=$BUILD_DIR/external

mkdir $BUILD_DIR
mkdir $EXT_BUILD_DIR

echo "[CT_ICP] -- [EXTERNAL DEPENDENCIES] -- Generating the cmake project"
cd ${EXT_BUILD_DIR}
cmake -G "$GENERATOR" -S $EXT_SRC_DIR -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DWITH_VIZ3D=$WITH_VIZ

echo "[CT_ICP] -- [EXTERNAL DEPENDENCIES] -- building CMake Project"
cmake --build . --config $BUILD_TYPE

#echo "[CT_ICP] -- [MAIN PROJECT] -- Generating the cmake project"
#cd $BUILD_DIR
#cmake -G "$GENERATOR" -S $SRC_DIR -DCMAKE_BUILD_TYPE=${BUILD_TYPE}  -DWITH_VIZ3D=$WITH_VIZ
#
#echo "[CT_ICP] -- [MAIN PROJECT] -- Building the CMake Project"
#cmake --build . --config $BUILD_TYPE --target install --parallel 6
