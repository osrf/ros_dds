#!/usr/bin/env bash

rtiddsgen2 poll.idl -replace -language C++ -example x64Linux3.xgcc4.6.3

mkdir build
cd build
cmake .. $@
make
