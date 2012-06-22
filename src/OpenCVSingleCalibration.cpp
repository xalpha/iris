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

/*
 * OpenCVSingleCalibration.cpp
 *
 *  Created on: Jun 14, 2012
 *      Author: duliu
 */


#include <iris/OpenCVSingleCalibration.hpp>


namespace iris {

OpenCVSingleCalibration::OpenCVSingleCalibration() : OpenCVCalibration()
{
}


OpenCVSingleCalibration::~OpenCVSingleCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVSingleCalibration::configure( bool fixPrincipalPoint, bool fixAspectRatio, bool tangentialDistortion )
{
    m_fixPrincipalPoint = fixPrincipalPoint;
    m_fixAspectRatio = fixAspectRatio;
    m_tangentialDistortion = tangentialDistortion;
}


void OpenCVSingleCalibration::calibrate()
{
    // run over all cameras and calibrate them
    for( auto it = m_cameras.begin(); it != m_cameras.end(); it++ )
        calibrateCamera( (*it).second, flags() );
}


bool OpenCVSingleCalibration::multipleCameras()
{
    return false;
}


int OpenCVSingleCalibration::flags()
{
    // init stuff
    int result = 0;

    if( m_fixPrincipalPoint )
        result = result | CV_CALIB_FIX_PRINCIPAL_POINT;

    if( m_fixAspectRatio )
        result = result | CV_CALIB_FIX_ASPECT_RATIO;

    if( !m_tangentialDistortion )
        result = result | CV_CALIB_ZERO_TANGENT_DIST;

    return result;
}


} // end namespace iris

