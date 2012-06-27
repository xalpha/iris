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
 * OpenCVSingleCalibration.hpp
 *
 *  Created on: Jun 14, 2012
 *      Author: duliu
 */

#include <iris/OpenCVCalibration.hpp>

namespace iris {

class OpenCVSingleCalibration : public OpenCVCalibration
{
public:
    OpenCVSingleCalibration();
    virtual ~OpenCVSingleCalibration();

    void configure( bool fixPrincipalPoint,
                    bool fixAspectRatio,
                    bool tangentialDistortion );

    virtual void calibrate();

 protected:
    void calibrateCamera( Camera_d &cam, int flags );

    virtual void filter();

    virtual int flags();

};

} // end namespace iris
