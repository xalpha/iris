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
 * OpenCVCalibration.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */


#include <iris/OpenCVCalibration.hpp>


namespace iris {

OpenCVCalibration::OpenCVCalibration() :
    CameraCalibration(),
    m_fixPrincipalPoint( false ),
    m_fixAspectRatio( true ),
    m_tangentialDistortion( true ),
    m_intrinsicGuess(false),
    m_minPoseCorrespondences( 8 )
{
}


OpenCVCalibration::~OpenCVCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVCalibration::setFixPrincipalPoint( bool val )
{
    m_fixPrincipalPoint = val;
}


void OpenCVCalibration::setFixAspectRatio( bool val )
{
    m_fixAspectRatio = val;
}


void OpenCVCalibration::setTangentialDistortion( bool val )
{
    m_tangentialDistortion = val;
}


void OpenCVCalibration::setMinCorrespondences( size_t val )
{
    m_minPoseCorrespondences = val;
}


void OpenCVCalibration::setIntrinsicGuess( bool val )
{
    m_intrinsicGuess = val;
}


std::vector< Eigen::Vector2d > OpenCVCalibration::projectPoints( const std::vector<cv::Point3f> points3D,
                                                                 const cv::Mat& rot,
                                                                 const cv::Mat& transl,
                                                                 const cv::Mat& cameraMatrix,
                                                                 const cv::Mat& distCoeff )
{
    // init stuff
    std::vector< cv::Point2f > projected;
    std::vector< Eigen::Vector2d > result;

    // project points and store
    cv::projectPoints( cv::Mat(points3D), rot, transl, cameraMatrix, distCoeff, projected );
    for( size_t p=0; p<projected.size(); p++ )
        result.push_back( Eigen::Vector2d( projected[p].x, projected[p].y ) );

    // return
    return result;
}


} // end namespace iris

