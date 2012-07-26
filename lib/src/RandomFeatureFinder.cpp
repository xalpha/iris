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
 * RandomFeatureFinder.cpp
 *
 *  Created on: Jul 25, 2012
 *      Author: duliu
 */

#include <iostream>

#include <opencv/cv.h>

#include <iris/RandomFeatureFinder.hpp>

namespace iris {


RandomFeatureFinder::RandomFeatureFinder() :
    Finder(),
    m_minPoints(8),
    m_mserMaxRadiusRatio( 3.0),
    m_mserMinDiameter( 10.0)
{
}


RandomFeatureFinder::~RandomFeatureFinder() {
	// TODO Auto-generated destructor stub
}


void RandomFeatureFinder::configure( const std::vector< Eigen::Vector2d >& points )
{
    // compute the descriptor for the points
    m_patternRFD( points );
}


void RandomFeatureFinder::setMinPoints( size_t minPoints )
{
    m_minPoints = minPoints;
}


bool RandomFeatureFinder::find( Pose_d& pose )
{

}


std::vector<Eigen::Vector2d> RandomFeatureFinder::findCircles( const cimg_library::CImg<uint8_t>& image )
{
    // init stuff
    cv::Mat img;
    cimg2cv( image, img );
    cv::Mat mask( img.rows, img.cols, img.type(), 255 );
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Eigen::Vector2d> centers;
    std::vector<cv::RotatedRect> ellipses;

    // detect blobs
    cv::MSER mser;
    mser( img, contours, mask );

    // fit ellipses
    for( size_t c = 0; c < contours.size(); c++ )
    {
        // fit the elipse
        bool ok = true;
        cv::RotatedRect ell = cv::fitEllipse( contours[c] );
        double rMax = std::max( ell.size.width, ell.size.height );
        double rMin = std::min( ell.size.width, ell.size.height );

        // filter the elipses
        ok = ok & ((rMax/rMin) <= m_mserMaxRadiusRatio);
        ok = ok & rMax > m_mserMinDiameter;

        // if all is well keep it
        if( ok )
        {
            centers.push_back( Eigen::Vector2d( ell.center.x, ell.center.x ) );
            ellipses.push_back( ell );
        }
    }

    // return
    return centers;
}


} // end namespace iris

