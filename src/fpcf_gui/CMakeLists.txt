### ---[ FPCF gui CMake
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

# read srcs
file(GLOB control_fusion_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/fusion/*.cpp)
file(GLOB control_guiStates_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/control/guiStates/*.cpp)
file(GLOB data_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/data/*.cpp)
file(GLOB gui_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/*.cpp)
# read headers
file(GLOB control_fusion_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf_gui/control/fusion/*.h)
file(GLOB control_guiStates_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf_gui/control/guiStates/*.h)
file(GLOB data_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf_gui/data/*.h)
file(GLOB gui_headers
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf_gui/gui/*.h)
# read forms
file(GLOB gui_forms
    ${CMAKE_CURRENT_SOURCE_DIR}/forms/*.ui)
# read resources
file(GLOB gui_resources
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/*.qrc)
    
# Qt stuff
SET(QT_USE_QTOPENGL TRUE)

# create source groups for *.cpp
source_group("Source Files\\control\\fusion" FILES
    ${control_fusion_srcs})
source_group("Source Files\\control\\guiStates" FILES
    ${control_guiStates_srcs})
source_group("Source Files\\data" FILES
    ${data_srcs})
source_group("Source Files\\gui" FILES
    ${gui_srcs})
# create source groups for *.h
source_group("Header Files\\control\\fusion" FILES
    ${control_fusion_headers})
source_group("Header Files\\control\\guiStates" FILES
    ${control_guiStates_headers})
source_group("Header Files\\data" FILES
    ${data_headers})
source_group("Header Files\\gui" FILES
    ${gui_headers})
# create source groups for *.ui
source_group("Qt\\Form Files" FILES
    ${gui_forms})
# create source groups for *.qrc
source_group("Qt\\Resource Files" FILES
    ${gui_resources})
# create source groups for Qt moc files
source_group("Qt\\Moc Files" FILES
    ${gui_headers_moc})
# create source groups for Qt rcc files
source_group("Qt\\Rcc Files" FILES
    ${helloworld_resources_rcc})
    
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${PROJECT_SOURCE_DIR}/fpcf/include ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    )

add_executable(fpcf_gui
    # add *.cpp
    ${control_fusion_srcs}
    ${control_guiStates_srcs}
    ${data_srcs}
    ${gui_srcs}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/fpcfGuiPrecompiled.cpp
    # add *.h
    ${control_fusion_headers}
    ${control_guiStates_headers}
    ${data_headers}
    ${gui_headers}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/fpcf_gui/fpcfGuiPrecompiled.h
    # add qt mocs
    ${gui_headers_moc}
    # add qt forms
    ${gui_forms}
    # add qt resources
    ${gui_resources}
    )
    
target_link_libraries(fpcf_gui
    fpcf
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
)
    
# precompiled headers
if (MSVC)
   set_source_files_properties(
        ${control_fusion_srcs}
        PROPERTIES
        COMPILE_FLAGS "/Yufpcf_gui/fpcfGuiPrecompiled.h"
        )
    set_source_files_properties(
        ${control_guiStates_srcs}
        PROPERTIES
        COMPILE_FLAGS "/Yufpcf_gui/fpcfGuiPrecompiled.h"
        )
    set_source_files_properties(
        ${data_srcs}
        PROPERTIES
        COMPILE_FLAGS "/Yufpcf_gui/fpcfGuiPrecompiled.h"
        )
    set_source_files_properties(
        ${gui_srcs}
        PROPERTIES
        COMPILE_FLAGS "/Yufpcf_gui/fpcfGuiPrecompiled.h"
        )
   set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
        PROPERTIES
        COMPILE_FLAGS "/Yufpcf_gui/fpcfGuiPrecompiled.h"
        )
   set_source_files_properties(
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fpcfGuiPrecompiled.cpp
        PROPERTIES
        COMPILE_FLAGS "/Ycfpcf_gui/fpcfGuiPrecompiled.h"
        )
endif(MSVC)