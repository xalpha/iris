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
 * ChessboardFinder.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */


#include <iris/ChessboardFinder.hpp>

namespace iris {


ChessboardFinder::ChessboardFinder() :
    Finder(),
    m_fastCheck(true),
    m_activeThreshold(true),
    m_normalizeImage(true),
    m_limitToLargeQuads(true)
{
}


ChessboardFinder::~ChessboardFinder() {
	// TODO Auto-generated destructor stub
}


void ChessboardFinder::configure( const size_t columns, const size_t rows, const double squareSize )
{
    // init stuff
    m_points3D.clear();
    m_indices.clear();

    // set stuff
    m_columns = columns;
    m_rows = rows;
    m_squareSize = squareSize;

    // generate the points
    // compute the coordinates of the corners in 3D
    float ss = static_cast<float>( m_squareSize );

    // compute the positions of the points
    for( size_t i=0; i<m_columns*m_rows; i++ )
    {
        m_points3D.push_back( Eigen::Vector3d( static_cast<double>( i % m_columns ) *ss,
                                              -static_cast<double>( i / m_columns ) *ss,
                                               0.0f ) );
        m_indices.push_back(i);
    }

    // all is well in the jungle
    m_configured = true;
}


void ChessboardFinder::setFastCheck( bool use )
{
    m_fastCheck = use;
}


void ChessboardFinder::setAdaptiveThreshold( bool use )
{
    m_activeThreshold = use;
}


void ChessboardFinder::setNormalizeImage( bool use )
{
    m_normalizeImage = use;
}


bool ChessboardFinder::find( Pose_d& pose )
{
    // check if configured
    if( !m_configured )
        throw std::runtime_error("ChessboardFinder::find: pattern not configured not configured.");

    // init stuff
    cimg_library::CImg<uint8_t>& image = *(pose.image);
    cv::Mat imageCV( image.height(), image.width(), CV_8UC3 );
    std::vector< cv::Point2f > corners;
    pose.points2D.clear();
    pose.points3D.clear();
    pose.pointIndices.clear();

    // convert it to the openCV internal format
    for( int y=0; y<image.height(); y++ )
        for( int x=0; x<image.width(); x++ )
            for( int c=0; c<3; c++ )
                imageCV.at<cv::Vec3b>(y,x)[c] = static_cast<unsigned char>( image( x, y, 0, c ) );

    // now detech the corners
    bool found = cv::findChessboardCorners( imageCV, cv::Size( m_columns, m_rows ), corners, flags() );

    // if found, refine the corners
    if( found )
    {
        // check if all corners were found (not sure this is necessary)
        if( corners.size() != m_points3D.size() )
            throw std::runtime_error("ChessboardFinder::find: found less corners then the grid should have.");

        // try to refine the corners (example from the opencv doc)
        cv::Mat grayImage;
        cv::cvtColor(imageCV, grayImage, CV_RGB2GRAY);
        cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.1 ));

        // convert to eigen
        for( size_t i=0; i<corners.size(); i++ )
            pose.points2D.push_back( Eigen::Vector2d( corners[i].x, corners[i].y ) );

        // set the 3d Points
        pose.points3D = m_points3D;
        pose.pointIndices = m_indices;
    }

    // return
    return found;
}


int ChessboardFinder::flags()
{
    int results = 0;

    if( m_fastCheck )
        results += CV_CALIB_CB_FAST_CHECK;

    if( m_activeThreshold )
        results += CV_CALIB_CB_ADAPTIVE_THRESH;

    if( m_normalizeImage )
        results += CV_CALIB_CB_NORMALIZE_IMAGE;

    return results;
}


} // end namespace iris

