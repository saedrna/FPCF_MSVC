### ---[ FPCF global CMake
#
# FPCF - Fast Point Cloud Fusion framework
#
# Copyright (c) 2014, T. Weber
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, 
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its 
# contributors may be used to endorse or promote products derived from this 
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.
#

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "configurations" FORCE)


######################################################################
#
# CMake Project Settings
#
######################################################################
project(FastPointCloudFusion LANGUAGES C CXX)
set(VERSION_MAJOR "1")
set(VERSION_MINOR "0")
set(VERSION_BugFix "0")
set(PROJECT_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BugFix})
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /MP /wd4996 /wd4819 /wd4251 /wd4267 /wd4244 /wd4275")
    add_definitions(-DWIN32_LEAN_AND_MEAN -DNOMINMAX) # windows use windows api
endif(MSVC)
if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override -Wno-unused-command-line-argument -Wno-unused-result -Wno-exceptions")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.9" CACHE STRING "Minimum OS X deployment version")
endif(APPLE)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(FPCF_USE_GUI "Build Gui application, for kinect fusion" OFF)
option(FPCF_USE_CLI "Build console application" ON)

######################################################################
#
# Packages
#
######################################################################
# PCL_MODULE_LIBRARIES
find_package(PCL 1.7 REQUIRED COMPONENTS
    keypoints
    segmentation
    registration
    surface
    visualization
    )

# Qt5::Module
if(FPCF_USE_GUI)
    find_package(Qt5 REQUIRED Core Gui Widgets OpenGL)    
endif(FPCF_USE_GUI)

add_subdirectory(fpcf)

if(FPCF_USE_GUI)
    add_subdirectory(fpcf_gui)
endif(FPCF_USE_GUI)

if(FPCF_USE_CLI)
    add_subdirectory(fpcf_examples)
endif(FPCF_USE_CLI)
