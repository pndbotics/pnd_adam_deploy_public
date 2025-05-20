#!/bin/bash

set -o errexit #exit on error

root=$(pwd)
build_dir=${root}/build*/

rm -rf ${build_dir}

rm -rf ${root}/bin