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

#pragma once


///
/// \file    OpenCVMultiCalibration.hpp
/// \class   OpenCVMultiCalibration
///
/// \package iris
/// \version 0.1.0
///
/// \brief   Class for multi-camera calibration
///
/// \details This class performs calibration of multiple cameras
///          under the assumption that the images are acquired synchronized.
///
/// \author  Amin Abouee
/// \author  Alexandru Duliu
/// \date    Nov 1, 2012
///


#include <iris/CameraCalibration.hpp>

namespace iris {

class OpenCVMultiCalibration : public CameraCalibration
{
public:
    OpenCVMultiCalibration();
    virtual ~OpenCVMultiCalibration();

    // run the calibration
    virtual void calibrate( CameraSet_d& cs );

protected:
    virtual void filter( CameraSet_d& cs );
    bool checkImageSize (const std::vector <iris::Camera_d> & cameraRef );
    bool checkFrame( const iris::Pose_d& pose1, const iris::Pose_d& pose2 );
};

} // end namespace iris
