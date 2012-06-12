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
 * Calibration.hpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */

#include <map>
#include <vector>

#include <Eigen/Core>


namespace iris {

class Calibration
{
public:
    struct Pose
    {
        std::vector<Eigen::Vector2d> points2D;
        std::vector<Eigen::Vector3d> points3D;
        size_t id;
        Eigen::Matrix4d transformation;
        std::vector<Eigen::Vector2d> projected2D;
    };

    struct Camera
    {
        std::vector< Pose > poses;
        Eigen::Vector2i imageSize;
        Eigen::Matrix3d intrinsic;
        std::vector<double> distortion;
        double error;
    };

public:
    Calibration();
    virtual ~Calibration();

    void addPose( const Pose& pose, const Eigen::Vector2i& imageSize, const size_t cameraID=0 );

    virtual void calibrate() = 0;

    const Camera& camera( const size_t id=0 ) const;
    const std::map< size_t, iris::Calibration::Camera >& cameras() const;

protected:
    // cameras
    std::map< size_t, iris::Calibration::Camera > m_cameras;
};

} // end namespace iris
