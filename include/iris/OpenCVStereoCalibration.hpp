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

#include <iris/Calibration.hpp>

namespace iris {

class OpenCVStereoCalibration : public Calibration
{
public:
    OpenCVStereoCalibration();
    virtual ~OpenCVStereoCalibration();

    void configure( bool fixPrincipalPoint,
                    bool fixAspectRatio,
                    bool sameFocalLength,
                    bool tangentialDistortion );

    virtual void calibrate();

protected:
    int flags();

protected:
    bool m_fixPrincipalPoint;
    bool m_fixAspectRatio;
    bool m_sameFocalLength;
    bool m_tangentialDistortion;

};

} // end namespace iris
