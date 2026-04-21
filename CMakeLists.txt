# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Set the target board (ESP32)
set(IDF_TARGET esp32)

# Include the project configuration
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project name: Aperture Controller for Brightness-Coded Motion Capture
project(aperture_controller)