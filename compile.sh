#!/bin/bash

debug="DEBUG"
record="RECORD"
release="RELEASE"

shopt -s nocasematch

case $1 in
        $debug)
         echo -e "Building with build type: Debug\n"
         catkin_make -DCMAKE_BUILD_TYPE:STRING="Debug" "${@:2}"
         ;;
        $record)
         echo -e "Building with build type: Record\n"
         catkin_make -DCMAKE_BUILD_TYPE:STRING="Record" "${@:2}"
         ;;
        $release)
         echo -e "Building with build type: Release\n"
         catkin_make -DCMAKE_BUILD_TYPE:STRING="Release" "${@:2}"
         ;;
        *)
         echo -e "Unknonw build type. Building with default build type: Release"
         catkin_make -DCMAKE_BUILD_TYPE:STRING="Release" "${@:2}"
         ;;
esac

shopt -u nocasematch