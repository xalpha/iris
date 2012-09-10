////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of iris, a lightweight C++ camera calibration library    //
//                                                                            //
// Copyright (C) 2010, 2011 Alexandru Duliu                                   //
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

/*
 * OpenCVStereoCalibration.hpp
 *
 *  Created on: June 12, 2012
 *      Author: duliu
 */

#include <iris/OpenCVCalibration.hpp>

namespace iris {

class OpenCVStereoCalibration : public OpenCVCalibration
{
public:
    OpenCVStereoCalibration();
    virtual ~OpenCVStereoCalibration();

    void setRelativeToPattern( bool val );
    void setSameFocalLength( bool val );
    void setFixIntrinsic( bool val );

    virtual void calibrate( CameraSet_d& cs );

protected:
    void stereoCalibrate( iris::Camera_d& cam1, iris::Camera_d& cam2 );

    virtual void filter( CameraSet_d& cs );

    bool checkFrame( const iris::Pose_d& cam1, const iris::Pose_d& cam2 );

    virtual int flags();

protected:
    bool m_relativeToPattern;
    bool m_sameFocalLength;
    bool m_fixIntrinsic;

};

} // end namespace iris
