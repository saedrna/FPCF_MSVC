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

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "configurations" FORCE)

project(FastPointCloudFusion)

# maybe only for msvc win32 builds:
#add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT -DEIGEN_DONT_ALIGN)

# special compiler flags for msvc
if (MSVC)
    # add big object file support
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
    # workaround for C1027 error with precompiled headers
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zm500")
    # activate multi-core compile
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif (MSVC)

find_package(PCL 1.7 REQUIRED COMPONENTS
    keypoints
    segmentation
    registration
    surface
    visualization
    )

ADD_SUBDIRECTORY(fpcf)
ADD_SUBDIRECTORY(fpcf_examples)
ADD_SUBDIRECTORY(fpcf_gui)