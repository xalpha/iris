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
#include <iris/Calibration.hpp>


namespace iris
{

class CameraCalibration
{
public:

public:
    CameraCalibration();
    virtual ~CameraCalibration();

    // add single image
    virtual bool addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const size_t poseID, const size_t cameraID=0 );

    // run the calibration
    virtual void calibrate();

    void setFinder( std::shared_ptr<Finder> finder );
    void setCalibration( std::shared_ptr<Calibration> calibration );

    const Finder& finder() const;
    const Calibration& calibration() const;

protected:
    void check();

protected:
    // these two do the work
    std::shared_ptr<Finder> m_finder;
    std::shared_ptr<Calibration> m_calibration;
};


} // end namespace iris
