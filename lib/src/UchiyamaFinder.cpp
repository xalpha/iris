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
 * UchiyamaFinder.cpp
 *
 *  Created on: Jul 18, 2011
 *      Author: duliu
 *
 * Acknowledgement: the author would like to thank Hideaki Uchiyama for his
 *                  great library to detect random dots:
 *                  H Uchiyama and H Saito: Random Dot Markers [VR 2011]
 *
 *                  and to Yuji Oyamada for his help in integrating random
 *                  dot detection into the iris library
 */

#include <algorithm>
#include <iostream>

#include <iris/UchiyamaFinder.hpp>

#define NO_BOOST
#include <llah.h>

namespace iris {


UchiyamaFinder::UchiyamaFinder() :
    Finder(),
    m_adaptiveExtract(false),
    m_minPoints(8)
{
    m_useOpenMP = false;
}


UchiyamaFinder::~UchiyamaFinder() {
	// TODO Auto-generated destructor stub
}


void UchiyamaFinder::configure( const std::string& patternDescriptor )
{
    m_patternDescriptor = patternDescriptor;
    m_patternDescriptorStream.clear();
    m_patternDescriptorStream << m_patternDescriptor;
    m_configured = true;
}


void UchiyamaFinder::setAdaptiveExtract( bool val )
{
    m_adaptiveExtract = val;
}


void UchiyamaFinder::setMinPoints( size_t minPoints )
{
    m_minPoints = minPoints;
}


template <typename T>
inline void cimg2myimage( const cimg_library::CImg<T>& src, MyImage& dst )
{
    // cimg -> cv::Mat
    cv::Mat imageCV;
    cimg2cv( src, imageCV );

    // cv::Mat -> IplImage
    IplImage imageIpl( imageCV );

    // iPlImage -> MyImage
    // note: MyImage wants to own the IplImage, that's the reason for all the pointers
    dst.setImage( &imageIpl );
}


template <typename T>
inline void getCorrespondences( LLAH& llah, Pose<T> &pose, double scale, double fx, double fy )
{
    // init stuff
    pose.points2D.clear();
    pose.points3D.clear();
    pose.pointIndices.clear();
    size_t idx, p3dx, p3dy, p3dz;
    double p2dx, p2dy, tmp;

    // get the correspondences
    std::stringstream stream;
    llah.SaveCorrespondingPoints( stream );
    stream.seekg( 0, std::ios::beg );

    // count lines
    std::string text = stream.str();
    size_t lineCount = std::count( text.begin(), text.end(), '\n' );

    // now parse them
    for( size_t i=0; i<lineCount && !stream.eof(); i++ )
    {
        // point index
        stream >> idx;

        // point 3d pos
        stream >> p3dx;
        stream >> p3dy;
        stream >> p3dz;

        // point 2d pos
        stream >> p2dx;
        stream >> p2dy;

        // finish up
        stream >> tmp;
        stream >> tmp;

        // update pose
        pose.pointIndices.push_back( idx );
        pose.points3D.push_back( Eigen::Vector3d( scale*static_cast<double>(p3dx),
                                                  scale*static_cast<double>(p3dy),
                                                  scale*static_cast<double>(p3dz) ) );
        pose.points2D.push_back( Eigen::Vector2d( fx*p2dx, fy*p2dy ) );
    }
}


bool UchiyamaFinder::find( Pose_d& pose )
{
    // check if we have a pattern
    if( !m_configured )
        throw std::runtime_error( "UchiyamaFinder::find: not pattern descriptor set." );

    // init stuff
    cimg_library::CImg<uint8_t> image = pixelLimit( *pose.image );
    LLAH llah;
    bool found;
    llah.Init( image.width(), image.height() );

    // add the pattern descriptor
    // TODO: should be in the configure function
    m_patternDescriptorStream.seekg( 0, std::ios::beg );
    llah.AddPaper( m_patternDescriptorStream );

    // comvert image to myimage (cimg -> cv::Mat -> IplImage -> MyImage)
    MyImage imageMy;
    cimg2myimage( image, imageMy );

    // extract keypoints
    if( m_adaptiveExtract )
        llah.AdaptiveExtract( imageMy );
    else
        llah.Extract( imageMy );
    llah.SetPts();

    // detect marker
    llah.RetrievebyTracking();
    found = llah.FindPaper( static_cast<int>(m_minPoints) );
    llah.RetrievebyMatching();
    found |= llah.FindPaper( static_cast<int>(m_minPoints) );

    // if succesfull store extracted points
    if( found )
    {
        getCorrespondences( llah, pose, m_scale,
                            static_cast<double>(pose.image->width())/static_cast<double>(image.width()),
                            static_cast<double>(pose.image->height())/static_cast<double>(image.height()));
    }
}


} // end namespace iris

