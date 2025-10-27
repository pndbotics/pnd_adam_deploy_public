#!/bin/bash

set -o errexit # exit on error

adam_type=$1
adam_env_type=$2
middleware_type=$3
if [ "$adam_type" != "adam_lite" ] && \
   [ "$adam_type" != "adam_inspire" ] && \
   [ "$adam_type" != "adam_standard" ] && \
   [ "$adam_type" != "adam_sp" ] && \
   [ "$adam_type" != "adam_pro" ]; then
	echo $adam_type
    echo "adam_type mast be adam_lite|adam_inspire|adam_standard|adam_sp|adam_pro. example: sh build.sh adam_lite real"
	exit 1
fi
if [ "$adam_env_type" != "real" ] && [ "$adam_env_type" != "mujoco" ] && [ "$adam_env_type" != "webots" ]; then
    echo $adam_env_type
    echo "adam_env_type mast be real|mujoco|webots. example: sh build.sh adam_lite real"
    exit 1
fi

if [ "$middleware_type" != "ros2" ] && [ "$middleware_type" != "lcm" ]; then
    middleware_type=""
fi

root=$(pwd)

build_dir=${root}/build_${adam_type}_${adam_env_type}

bin_dir=${root}/bin
if [ ! -d "$build_dir" ];then
    mkdir $build_dir
    echo "build create succeed!"
else
    echo "build already exist!"
fi
if [ ! -d "$bin_dir" ];then
    mkdir $bin_dir
    echo "bin create succeed!"
else
    echo "bin already exist!"
fi

cd ${build_dir}
cmake -DCMAKE_BUILD_TYPE=release \
    -DCMAKE_INSTALL_PREFIX=${root} \
    -Dadam_type=$adam_type \
    -Dadam_env_type=$adam_env_type \
    -Dmiddleware_type=$middleware_type \
    ..
make -j$(($(nproc) - 2))
if [ "$adam_env_type" != "mujoco" ] && [ "$adam_env_type" != "webots" ]; then
    make install
fi
cd -