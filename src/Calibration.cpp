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
 * Calibration.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */

#include <iostream>
#include <stdexcept>

#include <iris/Calibration.hpp>

namespace iris {

Calibration::Calibration()
{
}


Calibration::~Calibration() {
	// TODO Auto-generated destructor stub
}


void Calibration::addPose( const Pose_d &pose, const Eigen::Vector2i& imageSize, const size_t cameraID )
{
    // check if the corresponcences match
    if( pose.points2D.size() != pose.points3D.size() )
        throw std::runtime_error( "CameraCalibration::addPose: 2D and 3D points don't match." );

    // add the pose
    m_cameras[cameraID].poses.push_back( pose );

    // make sure the image sizes are the same
    if( m_cameras[cameraID].poses.size() == 1 )
        m_cameras[cameraID].imageSize = imageSize;
    else
    {
        Eigen::Vector2i tmp = imageSize - m_cameras[cameraID].imageSize;
        if( (tmp(0) != 0) || (tmp(1) != 0) )
            throw std::runtime_error( "CameraCalibration::addPose: pose has different image size than already stored poses." );
    }
}


const Camera_d &Calibration::camera( const size_t id) const
{
    // find the camera
    std::map< size_t, Camera_d >::const_iterator camIt = m_cameras.find( id );

    // search for the camera
    if( camIt != m_cameras.end() )
        return (*camIt).second;
    else
        throw std::runtime_error("CameraCalibration::camera: camera not found.");
}


const std::map< size_t, iris::Camera_d >& Calibration::cameras() const
{
    return m_cameras;
}


} // end namespace iris

