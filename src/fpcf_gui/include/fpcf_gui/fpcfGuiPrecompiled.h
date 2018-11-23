/**
 * @file    fpcfGuiPrecompiled.h
 * @author  Thomas Weber
 *
 * Copyright (c) 2014, T. Weber
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _FPCFGUIPRECOMPILED_H_
#define _FPCFGUIPRECOMPILED_H_

// STL
#include <string>

// Boost
#include <boost/smart_ptr.hpp>

// Qt

// PCL

// FPCF
#include <fpcf/data/fpcfPointCloudData.h>
#include <fpcf/filter/fpcfDistanceFilter.h>
#include <fpcf/filter/fpcfNoiseFilter.h>
#include <fpcf/filter/fpcfPlaneFilter.h>
#include <fpcf/filter/fpcfVoxelDownsampling.h>
#include <fpcf/registration/fpcfPointCloudPreparation.h>
#include <fpcf/registration/features/descriptors/fpcfFPFHDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfPFHDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfPFHRGBDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfShotDescriptor.h>
#include <fpcf/registration/features/descriptors/fpcfShotColorDescriptor.h>
#include <fpcf/registration/features/detectors/fpcfNarfKeypoint.h>
#include <fpcf/registration/features/detectors/fpcfSiftKeypoint.h>
#include <fpcf/registration/correspondences/fpcfCorrespondenceRejector.h>
#include <fpcf/registration/transformation/fpcfICPPairAlignment.h>
#include <fpcf/registration/transformation/fpcfLuMGlobalAlignment.h>
#include <fpcf/registration/fpcfMultipleRegistration.h>
#include <fpcf/tools/fpcfPointCloudDataWrapper.h>

#include <fpcf_gui/control/fusion/fpcfFusionManager.h>
#include <fpcf_gui/control/fusion/fpcfGuiMultipleFusion.h>
#include <fpcf_gui/control/fusion/fpcfGuiPointCloudPreparation.h>
#include <fpcf_gui/control/guiStates/fpcfState.h>
#include <fpcf_gui/control/guiStates/fpcfStateMachine.h>
#include <fpcf_gui/data/fpcfFusionDataGroup.h>

#endif // #ifndef _FPCFGUIPRECOMPILED_H_