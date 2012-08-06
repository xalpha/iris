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
#include <strstream>

#include <opencv/cv.h>

#include <iris/RandomFeatureFinder.hpp>

namespace iris {


RandomFeatureFinder::RandomFeatureFinder() :
    Finder(),
    m_minPoints(8),
    m_mserMaxRadiusRatio( 3.0),
    m_mserMinRadius( 10.0)
{
}


RandomFeatureFinder::~RandomFeatureFinder() {
    // TODO Auto-generated destructor stub
}


void RandomFeatureFinder::configure( const std::vector< Eigen::Vector2d >& points )
{
    // compute the descriptor for the points
    if( points.size() > m_minPoints )
    {
        // generate feature vectors
        m_patternRFD( points, true );

        // set the 3d points
        m_points3D.clear();
        for( size_t i=0; i<points.size(); i++ )
            m_points3D.push_back( Eigen::Vector3d( points[i](0), points[i](1), 0 ) );

        // we are happy
        m_configured = true;
    }
    else
        throw std::runtime_error("RandomFeatureFinder::configure: insufficient points.");
}


void RandomFeatureFinder::configure( const std::string& points )
{
    // init stuff
    std::stringstream ss;
    double x, y;
    std::vector< Eigen::Vector2d > points2D;

    // convert to stringstram
    ss << points;
    ss.seekg( 0, std::ios::beg );

    // parse the points
    while( !ss.eof() )
    {
        ss >> x;
        ss >> y;

        points2D.push_back( Eigen::Vector2d( m_scale*x, m_scale*y) );
    }
    points2D.pop_back();

    // configure
    configure( points2D );
}


void RandomFeatureFinder::setMinPoints( size_t minPoints )
{
    m_minPoints = minPoints;
}


void RandomFeatureFinder::setMaxRadiusRatio( double val )
{
    m_mserMaxRadiusRatio = val;
}


void RandomFeatureFinder::setMinRadius( double val )
{
    m_mserMinRadius = val;
}


bool RandomFeatureFinder::find( Pose_d& pose )
{
    if( !m_configured )
        throw std::runtime_error("RandomFeatureFinder::find: not configured.");

    // find circles in pose
    std::vector<Eigen::Vector2d> posePoints = findCircles( *pose.image );
    if( posePoints.size() < m_minPoints )
        return false;

    // generate descriptors for detected points
    RFD poseRFD;
    poseRFD( posePoints );

    // compare the resulting descriptors with the configured
    Pose_d matchedPose = m_patternRFD & poseRFD;

    // assemble the result
    if( matchedPose.pointIndices.size() >= m_minPoints )
    {
        // set the indices and 2D points
        pose.pointIndices = matchedPose.pointIndices;
        pose.points2D = matchedPose.points2D;

        // run over the indices and set the 3D points
        pose.points3D.clear();
        for( size_t i=0; i<pose.pointIndices.size(); i++ )
            pose.points3D.push_back( m_points3D[pose.pointIndices[i] ] );

        // we are happy
        return true;
    }
    else
        return false;
}


std::vector<Eigen::Vector2d> RandomFeatureFinder::findCircles( const cimg_library::CImg<uint8_t>& image )
{
    // init stuff
    cv::Mat img;
    cv::Mat gray;
    cimg2cv( image, img );
    cvtColor(img, gray, CV_RGB2GRAY);
    cv::Mat mask( img.rows, img.cols, gray.type(), 255 );
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Eigen::Vector2d> centers;
    std::vector<cv::RotatedRect> ellipses;
    std::vector<float> areas;

    // detect blobs
    cv::MSER mser;
    mser( gray, contours, mask );

    // fit ellipses
    float pi = std::acos(-1.0);
    for( size_t c = 0; c < contours.size(); c++ )
    {
        // fit ellipse
        cv::RotatedRect ell = cv::fitEllipse( contours[c] );
        ellipses.push_back( ell );

        // get area of ellipse
        areas.push_back( pi * ell.size.width * ell.size.height );
    }

    // filter ellipses
    float mean_area = mean( areas );
    for( size_t e = 0; e < ellipses.size(); e++ )
    {
        // fit the elipse
        bool ok = true;
        cv::RotatedRect ell = ellipses[e];
        double rMax = std::max( ell.size.width, ell.size.height );
        double rMin = std::min( ell.size.width, ell.size.height );

        // filter the elipses
        ok = ok & ((rMax/rMin) <= m_mserMaxRadiusRatio);
        ok = ok & rMin > m_mserMinRadius;
        ok = ok & fabs( areas[e] - mean_area ) < 0.5*mean_area;

        // if all is well keep it
        if( ok )
        {
            centers.push_back( Eigen::Vector2d( ell.center.x, ell.center.y ) );
            ellipses.push_back( ell );
        }
    }

    // return
    return centers;
}


} // end namespace iris

