#!/usr/bin/env sh
mkdir src/ros-packages
vcs import src/ros-packages < packages.rosinstall
