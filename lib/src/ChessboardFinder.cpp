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
    m_limitToLargeQuads(true),
    m_subpixelCorner(true)
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
    double ss = static_cast<double>( m_squareSize ) * m_scale;

    // compute the positions of the points
    for( size_t i=0; i<m_columns*m_rows; i++ )
    {
        m_points3D.push_back( Eigen::Vector3d( static_cast<double>( i % m_columns ) *ss,
                                               static_cast<double>( i / m_columns ) *ss,
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


void ChessboardFinder::setSubpixelCorner( bool val )
{
    m_subpixelCorner = val;
}


bool ChessboardFinder::find( Pose_d& pose )
{
    // check if configured
    if( !m_configured )
        throw std::runtime_error("ChessboardFinder::find: pattern not configured not configured.");

    // init stuff
    cimg_library::CImg<uint8_t>& image = *(pose.image);
    cv::Mat imageCV;
    std::vector< cv::Point2f > corners;
    cv::Size patternSize( m_columns, m_rows );
    bool found = false;
    pose.points2D.clear();
    pose.points3D.clear();
    pose.pointIndices.clear();

    // convert it to the openCV internal format
    iris::cimg2cv( image, imageCV );

    // determine scale factor for the image
    size_t divFac = devideFactor( image );

    // check if the image needs scaling and detect corners
    if( 1 == divFac )
        found = cv::findChessboardCorners( imageCV, patternSize, corners, flags() );
    else
    {
        // scale down untill targets are met
        cimg_library::CImg<uint8_t> imageNew = image.get_resize_halfXY();
        for( size_t f=divFac/2; f>1; f=f/2 )
            imageNew.resize_halfXY();

        // convert to openCV
        cv::Mat imageNewCV;
        iris::cimg2cv( imageNew, imageNewCV );

        // now detect the corners
        found = cv::findChessboardCorners( imageNewCV, patternSize, corners, flags() );

        // if anything found, scale the points back
        float facX = static_cast<float>(image.width())/static_cast<float>(imageNew.width());
        float facY = static_cast<float>(image.height())/static_cast<float>(imageNew.height());
        for( size_t i=0; i<corners.size(); i++ )
        {
            corners[i].x *= facX;
            corners[i].y *= facY;
        }
    }

    // if found, refine the corners
    if( found )
    {
        // check if all corners were found (not sure this is necessary)
        if( corners.size() != m_points3D.size() )
            throw std::runtime_error("ChessboardFinder::find: found less corners then the grid should have.");

        // try to refine the corners (example from the opencv doc)
        if( m_subpixelCorner )
        {
            cv::Mat grayImage;
            cv::cvtColor(imageCV, grayImage, CV_RGB2GRAY);
            cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.1 ));
        }

        // convert to eigen
        for( size_t i=0; i<corners.size(); i++ )
            pose.points2D.push_back( Eigen::Vector2d( corners[i].x, corners[i].y ) );

        // set the 3d Points
        pose.detected2D = pose.points2D;
        pose.points3D = m_points3D;
        pose.pointIndices = m_indices;
        pose.pointsMax = m_indices.size();
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


int ChessboardFinder::devideFactor( const cimg_library::CImg<uint8_t>& image )
{
    // init stuff
    int f = 1;
    int mapPix = 2000000;
    int slack = 1000000;
    int pixelCount = image.width()*image.height();

    // determine the best scale factor
    while( (pixelCount > (mapPix + slack)) &&
           ( abs( pixelCount - mapPix ) > slack ) )
    {
        pixelCount = pixelCount / 4;
        f *= 2;
    }

    std::cout << "ChessboardFinder::devideFactor: width: " << image.width() << ", height: " << image.height() << ", MP: " << static_cast<double>(pixelCount)/1000000.0 << ", f: " << f << std::endl;

    return f;
}


} // end namespace iris

