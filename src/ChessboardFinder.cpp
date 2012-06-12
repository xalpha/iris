    /*
 * ChessboardFinder.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */


#include <iris/vision.hpp>
#include <iris/ChessboardFinder.hpp>

namespace iris {


ChessboardFinder::ChessboardFinder() : Finder()
{
}


ChessboardFinder::~ChessboardFinder() {
	// TODO Auto-generated destructor stub
}


void ChessboardFinder::configure( const size_t columns, const size_t rows, const double squareSize )
{
    // set stuff
    m_columns = columns;
    m_rows = rows;
    m_squareSize = squareSize;

    // generate the points
    // compute the coordinates of the corners in 3Ds
    m_points3D.clear();
    float ss = static_cast<float>( m_squareSize );

    // compute the positions of the points
    for( size_t i=0; i<m_columns*m_rows; i++ )
        m_points3D.push_back( Eigen::Vector3d( static_cast<double>( i % m_columns ) *ss,
                                              -static_cast<double>( i / m_columns ) *ss,
                                               0.0f ) );

    // all is well in the jungle
    m_configured = true;
}


bool ChessboardFinder::find( std::shared_ptr<cimg_library::CImg<uint8_t> > image )
{
    // check if configured
    if( !m_configured )
        throw std::runtime_error("ChessboardFinder::find: pattern not configured not configured.");

    // init stuff
    cv::Mat imageCV( image->height(), image->width(), CV_8UC3 );
    std::vector< cv::Point2f > corners;

    // convert it to the openCV internal format
    for( int y=0; y<image->height(); y++ )
        for( int x=0; x<image->width(); x++ )
            for( int c=0; c<3; c++ )
                imageCV.at<cv::Vec3b>(y,x)[c] = static_cast<unsigned char>( (*image)( x, y, 0, c ) );

    // now detech the corners
    bool found = cv::findChessboardCorners( imageCV, cv::Size( m_columns, m_rows ), corners );//, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK );

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
        m_points2D.clear();
        for( size_t i=0; i<corners.size(); i++ )
            m_points2D.push_back( Eigen::Vector2d( corners[i].x, corners[i].y ) );
    }

    // return
    return found;
}


} // end namespace iris

