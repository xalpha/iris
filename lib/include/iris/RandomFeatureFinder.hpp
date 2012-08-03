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
 * RandomFeatureFinder.hpp
 *
 *  Created on: Jul 25, 2012
 *      Author: duliu
 */


#include <iris/RandomFeatureDescriptor.hpp>
#include <iris/Finder.hpp>

namespace iris
{


class RandomFeatureFinder : public Finder
{
    typedef RandomFeatureDescriptor<9,7,5> RFD;

public:
    RandomFeatureFinder();
    virtual ~RandomFeatureFinder();

    void configure( const std::vector< Eigen::Vector2d >& points );
    void configure( const std::string& points );

    void setMinPoints( size_t minPoints );
    void setMaxRadiusRatio( double val );
    void setMinRadius( double val );

    virtual bool find( Pose_d& pose );

protected:
    std::vector< Eigen::Vector2d > findCircles( const cimg_library::CImg<uint8_t>& image );

protected:
    // config
    RFD m_patternRFD;
    size_t m_minPoints;

    // MSER elipse detector
    double m_mserMaxRadiusRatio;
    double m_mserMinRadius;
};

} // end namespace iris
