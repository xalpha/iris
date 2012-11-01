////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of iris, a lightweight C++ camera calibration library    //
//                                                                            //
// Copyright (C) 2012 Alexandru Duliu                                         //
//                                                                            //
// iris is free software; you can redistribute it and/or                      //
// modify it under the terms of the GNU Lesser General Public                 //
// License as published by the Free Software Foundation; either               //
// version 3 of the License, or (at your option) any later version.           //
//                                                                            //
// iris is distributed in the hope that it will be useful, but WITHOUT ANY    //
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  //
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU Lesser General Public           //
// License along with iris. If not, see <http://www.gnu.org/licenses/>.       //
//                                                                            //
///////////////////////////////////////////////////////////////////////////////


///
/// \file    MultiCameraCalibration.hpp
/// \class   MultiCameraCalibration
///
/// \author  Amin Abouee
/// \author  Alexandru Duliu
/// \date    Nov 1, 2012
///


#include <iris/MultiCameraCalibration.hpp>


namespace iris {

MultiCameraCalibration::MultiCameraCalibration() :
    CameraCalibration()
{
}


MultiCameraCalibration::~MultiCameraCalibration() {
	// TODO Auto-generated destructor stub
}


void MultiCameraCalibration::calibrate( CameraSet_d& cs )
{
    // this is where the actual work is performed
    //
    // the implementation is responsible for running the registered
    // finder, filtering, calibration and committing the results
    //
    // example implementations can be found under:
    // OpenCVSingleCalibration.hpp and OpenCVStereoCalibration.hpp
}


void MultiCameraCalibration::filter( CameraSet_d& cs )
{
    // filter function runs over all poses of all cameras
    // selects the poses and in the order it needs them
    // and stores a copy in m_filteredCameras
}


} // end namespace iris

