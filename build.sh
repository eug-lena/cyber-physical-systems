#!/bin/bash

# Script to build locally

# Remove existing build directory
rm -rf build

# Create a new build directory
mkdir build

# Change directory to the build directory
cd build

# Run CMake to configure the project
cmake ..

# Build the project using make
make
