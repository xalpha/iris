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
 * Finder.hpp
 *
 *  Created on: Jun 5, 2012
 *      Author: duliu
 */

#include <map>
#include <vector>

#include <Eigen/Core>

#include <iris/util.hpp>


namespace iris
{

class Finder
{
public:
    Finder();
    virtual ~Finder();

    void setScale( double scale );

    virtual bool find( Pose_d& pose ) = 0;

protected:
    bool m_configured;
    double m_scale;
    std::vector<Eigen::Vector3d> m_points3D;
    std::vector<size_t> m_indices;
};

} // end namespace iris
