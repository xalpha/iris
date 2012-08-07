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

/*
 * CameraCalibration.hpp
 *
 *  Created on: Jun 5, 2012
 *      Author: duliu
 */

#include <stdexcept>
#include <memory>

#include <iris/Finder.hpp>
#include <iris/util.hpp>

namespace iris
{

class CameraCalibration
{
public:
    CameraCalibration();
    virtual ~CameraCalibration();

    // add single image
    virtual size_t addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const std::string& name, const size_t cameraID=0 );

    // clear
    void clear();

    // save to disk
    void save( const std::string& filename );

    // load from disk
    void load( const std::string& filename );

    // run the calibration
    virtual void calibrate() = 0;

    void setFinder( std::shared_ptr<Finder> finder );

    const Finder& finder() const;

    const std::map< size_t, iris::Camera_d >& cameras() const;

    const Camera_d& camera( const size_t id=0 ) const;
    const Pose_d& pose( const size_t id ) const;

    const size_t poseCount() const;

    void copyCameras( std::shared_ptr<CameraCalibration> cc );

protected:
    virtual void filter() = 0;
    virtual void commit();

    void check();

    void threadID();

protected:
    // these two do the work
    std::shared_ptr<Finder> m_finder;

    // this on what the work gets done
    size_t m_poseCount;
    std::map< size_t, iris::Camera_d > m_cameras;

    // filtered
    std::map< size_t, iris::Camera_d > m_filteredCameras;

    // flags
    bool m_handEye;
};


} // end namespace iris
