### ---[ FPCF lib CMake
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

set(SUBSYS_NAME fpcf)
set(SUBSYS_DESC "Fast Point Cloud Fusion framework")

# read srcs
file(GLOB data_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/*.cpp)
file(GLOB filter_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/filter/*.cpp)
file(GLOB io_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io/*.cpp)
file(GLOB reconstruction_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/reconstruction/*.cpp)
file(GLOB registration_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/*.cpp)
file(GLOB registration_correspondences_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/correspondences/*.cpp)
file(GLOB registration_features_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/features/*.cpp)
file(GLOB registration_features_descriptors_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/features/descriptors/*.cpp)
file(GLOB registration_features_detectors_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/features/detectors/*.cpp)
file(GLOB registration_graph_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/graph/*.cpp)
file(GLOB registration_transformation_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/registration/transformation/*.cpp)
file(GLOB segmentation_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/segmentation/*.cpp)
file(GLOB tools_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/*.cpp)
# read headers
file(GLOB data_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/data/*.h)
file(GLOB filter_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/filter/*.h)
file(GLOB io_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/io/*.h)
file(GLOB reconstruction_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/reconstruction/*.h)
file(GLOB registration_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/*.h)
file(GLOB registration_correspondences_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/correspondences/*.h)
file(GLOB registration_features_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/features/*.h)
file(GLOB registration_features_descriptors_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/features/descriptors/*.h)
file(GLOB registration_features_detectors_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/features/detectors/*.h)
file(GLOB registration_graph_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/graph/*.h)
file(GLOB registration_transformation_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/registration/transformation/*.h)
file(GLOB segmentation_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/segmentation/*.h)
file(GLOB tools_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/tools/*.h)

# create source groups for *.cpp
source_group("Source Files\\data" FILES
    ${data_srcs})
source_group("Source Files\\filter" FILES
    ${filter_srcs})
source_group("Source Files\\io" FILES
    ${io_srcs})
source_group("Source Files\\reconstruction" FILES
    ${reconstruction_srcs})
source_group("Source Files\\registration" FILES
    ${registration_srcs})
source_group("Source Files\\registration\\correspondences" FILES
    ${registration_correspondences_srcs})
source_group("Source Files\\registration\\features" FILES
    ${registration_features_srcs})
source_group("Source Files\\registration\\features\\descriptors" FILES
    ${registration_features_descriptors_srcs})
source_group("Source Files\\registration\\features\\detectors" FILES
    ${registration_features_detectors_srcs})
source_group("Source Files\\registration\\graph" FILES
    ${registration_graph_srcs})
source_group("Source Files\\registration\\transformation" FILES
    ${registration_transformation_srcs})
source_group("Source Files\\segmentation" FILES
    ${segmentation_srcs})
source_group("Source Files\\tools" FILES
    ${tools_srcs})
# create source groups for *.h
source_group("Header Files\\data" FILES
    ${data_headers})
source_group("Header Files\\filter" FILES
    ${filter_headers})
source_group("Header Files\\io" FILES
    ${io_headers})
source_group("Header Files\\reconstruction" FILES
    ${reconstruction_headers})
source_group("Header Files\\registration" FILES
    ${registration_headers})
source_group("Header Files\\registration\\correspondences" FILES
    ${registration_correspondences_headers})
source_group("Header Files\\registration\\features" FILES
    ${registration_features_headers})
source_group("Header Files\\registration\\features\\descriptors" FILES
    ${registration_features_descriptors_headers})
source_group("Header Files\\registration\\features\\detectors" FILES
    ${registration_features_detectors_headers})
source_group("Header Files\\registration\\graph" FILES
    ${registration_graph_headers})
source_group("Header Files\\registration\\transformation" FILES
    ${registration_transformation_headers})
source_group("Header Files\\segmentation" FILES
    ${segmentation_headers})
source_group("Header Files\\tools" FILES
    ${tools_headers})

set(_NAME ${SUBSYS_NAME})
include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${_NAME} STATIC
    # add *.cpp
    ${data_srcs}
    ${filter_srcs}
    ${io_srcs}
    ${reconstruction_srcs}
    ${registration_srcs}
    ${registration_correspondences_srcs}
    ${registration_features_srcs}
    ${registration_features_descriptors_srcs}
    ${registration_features_detectors_srcs}
    ${registration_graph_srcs}
    ${registration_transformation_srcs}
    ${segmentation_srcs}
    ${tools_srcs}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/fpcfPrecompiled.cpp
    # add *.h
    ${data_headers}
    ${filter_headers}
    ${io_headers}
    ${reconstruction_headers}
    ${registration_headers}
    ${registration_correspondences_headers}
    ${registration_features_headers}
    ${registration_features_descriptors_headers}
    ${registration_features_detectors_headers}
    ${registration_graph_headers}
    ${registration_transformation_headers}
    ${segmentation_headers}
    ${tools_headers}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf/fpcfPrecompiled.h
    )
    
target_link_libraries(${_NAME}
    ${PCL_KEYPOINTS_LIBRARIES}
    ${PCL_SEGMENTATION_LIBRARIES}
    ${PCL_REGISTRATION_LIBRARIES}
    ${PCL_SURFACE_LIBRARIES}
    ${PCL_VISUALIZATION_LIBRARIES}
    )
    
# precompiled headers
if (MSVC)
   set_target_properties(
        ${_NAME}
        PROPERTIES
        COMPILE_FLAGS "/Yufpcf/fpcfPrecompiled.h"
        )
   set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fpcfPrecompiled.cpp
        PROPERTIES
        COMPILE_FLAGS "/Ycfpcf/fpcfPrecompiled.h"
        )
endif(MSVC)