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
#include <iris/CameraSet.hpp>
#include <iris/util.hpp>

namespace iris
{

class CameraCalibration
{
public:
    CameraCalibration();
    virtual ~CameraCalibration();

    void setMinCorrespondences( size_t val );

    // run the calibration
    virtual void calibrate( CameraSet_d& cs ) = 0;

    void setFinder( std::shared_ptr<Finder> finder );

    const Finder& finder() const;

protected:
    virtual void filter( CameraSet_d& cs ) = 0;
    virtual void commit( CameraSet_d& cs );

    void check();

    void threadID();

protected:
    // these two do the work
    std::shared_ptr<Finder> m_finder;

    // filtered
    CameraSet_d::CameraMap m_filteredCameras;

    // properties
    size_t m_minPoseCorrespondences;

    // threads
    size_t m_threadCount;
};


} // end namespace iris
