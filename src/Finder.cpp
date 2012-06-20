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
 * Finder.cpp
 *
 *  Created on: Jun 5, 2011
 *      Author: duliu
 */


#include <iris/Finder.hpp>

namespace iris {


Finder::Finder() :
    m_configured(false)
{
}


Finder::~Finder() {
	// TODO Auto-generated destructor stub
}


void Finder::clear()
{
    m_points2D.clear();
    m_points3D.clear();
}


const std::vector< Eigen::Vector2d >& Finder::points2D() const
{
    return m_points2D;
}


const std::vector< Eigen::Vector3d >& Finder::points3D() const
{
    return m_points3D;
}


} // end namespace iris

