#!/usr/bin/env sh

# This script generates message code from the IDL file, creates a CMake build
# directory and invokes CMake to build the examples.
# Arguments to this script are passed directly to cmake, e.g.
# $ . build.sh -DCMAKE_BUILD_TYPE=Release
# will build the examples in release mode.

rm -fr c
mkdir c
cd c
OSPL_TMPL_PATH=/usr/etc/opensplice/idlpp idlpp -S -l c ../LargeMsg.idl
echo "Generated C code under '`pwd`'"
cd ..

rm -fr build
mkdir build
cd build
cmake .. $@
make
cd ..
