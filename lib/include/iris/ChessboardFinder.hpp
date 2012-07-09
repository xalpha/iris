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
 * ChessboardFinder.hpp
 *
 *  Created on: Jun 5, 2012
 *      Author: duliu
 */


#include <iris/Finder.hpp>

namespace iris
{


class ChessboardFinder : public Finder
{
public:
    ChessboardFinder();
    virtual ~ChessboardFinder();

    void configure( const size_t columns, const size_t rows, const double squareSize );

    virtual bool find( Pose_d& pose );

protected:
    size_t m_columns;
    size_t m_rows;
    double m_squareSize;

};

} // end namespace iris
