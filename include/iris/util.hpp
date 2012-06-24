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

#include <cstdint>
#include <list>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

using std::ptrdiff_t;

#include <opencv/cv.h>
#include <opencv2/core/eigen.hpp>

#define cimg_display 0
#include <CImg.h>

namespace iris
{

/////
// Pose
///
template <typename T>
class Pose
{
public:
    std::shared_ptr< cimg_library::CImg<uint8_t> > image;
    bool rejected;
    std::vector< Eigen::Matrix<T,2,1> > points2D;
    std::vector< Eigen::Matrix<T,3,1> > points3D;
    size_t id;
    Eigen::Matrix<T,4,4> transformation;
    std::vector< Eigen::Matrix<T,2,1> > projected2D;
};
typedef Pose<double> Pose_d;


/////
// Camera
///
template <typename T>
class Camera
{
public:
    std::vector< Pose<T> > poses;
    Eigen::Vector2i imageSize;
    Eigen::Matrix<T,3,3> intrinsic;
    std::vector<T> distortion;
    T error;
};
typedef Camera<double> Camera_d;


/////
// OpenCV Points
///
template <typename To>
inline std::vector<cv::Point_<To> > eigen2cv( const std::vector<Eigen::Vector2d >& points2D )
{
    // init stuff
    std::vector<cv::Point_<To> > cvPoints2D;

    // convert the points
    for( size_t j=0; j<points2D.size(); j++ )
        cvPoints2D.push_back( cv::Point_<To>( static_cast<To>( points2D[j](0) ),
                                              static_cast<To>( points2D[j](1) ) ) );

    // return
    return cvPoints2D;
}


template <typename To>
inline std::vector<cv::Point3_<To> > eigen2cv( const std::vector<Eigen::Vector3d>& points3D )
{
    // init stuff
    std::vector<cv::Point3_<To> > cvPoints3D;

    // convert the points
    for( size_t j=0; j<points3D.size(); j++ )
        cvPoints3D.push_back( cv::Point3_<To>( static_cast<To>( points3D[j](0) ),
                                               static_cast<To>( points3D[j](1) ),
                                               static_cast<To>( points3D[j](2) ) ) );

    // return
    return cvPoints3D;
}



/////
// OpenCV Transformation
///
template <typename T, int Rows, int Cols>
void cv2eigen( const cv::Mat& rot, const cv::Mat& transl, Eigen::Matrix<T,Rows,Cols>& mat )
{
    // init stuff
    Eigen::Transform<T,3,Eigen::Affine> trans;
    trans.setIdentity();

    try
    {
        // get the rotation
        cv::Mat rotationCV( 3, 3, rot.type() );
        cv::Rodrigues( rot, rotationCV );
        Eigen::Matrix<T,3,3> rotation;
        cv::cv2eigen( rotationCV, rotation );

        // get the translation
        Eigen::Matrix<T,3,1> translation;
        cv::cv2eigen( transl, translation );

        // convert to a blas transformation
        trans.translate( translation );
        trans.rotate( rotation );
    }
    catch( std::exception& e )
    {
        throw std::runtime_error("iris::cv2eigen: type of openCV rotation and translation matrices don't match that of the output matrix.");
    }

    // return
    //mat = trans.matrix().cast<T>;
    mat = trans.matrix();
}


template <typename T, int Rows, int Cols>
inline void eigen2cv( const Eigen::Matrix<T, Rows, Cols>& trans, cv::Mat& rot, cv::Mat& transl )
{
    // generate the affine trans
    Eigen::Transform<T, 3, Eigen::Affine> affine( trans );

    // decompose the affine transformation
    Eigen::Matrix<T, 3, 3> rotation = affine.rotation();
    Eigen::Matrix<T, 3, 1> translation = affine.translation();

    // convert rotation
    cv::Mat rotationCV( 3, 3, rot.type() );
    cv::eigen2cv( rotation, rotationCV );
    cv::Rodrigues( rotationCV, rot );

    // convert translation
    cv::eigen2cv( translation, transl );
}


/////
// OpenCV Matrix Access
///

//template<typename T> T get( const cv::Mat &mat, int row, int col)
//{
//    switch( mat.type() )
//    {
//        case CV_8U :
//            return static_cast<T>( mat.at<uint8_t>(row,col) );

//        case CV_8S :
//            return static_cast<T>( mat.at<int8_t>(row,col) );

//        case CV_16U :
//            return static_cast<T>( mat.at<uint16_t>(row,col) );

//        case CV_16S :
//            return static_cast<T>( mat.at<int16_t>(row,col) );

//        case CV_32S :
//            return static_cast<T>( mat.at<int32_t>(row,col) );

//        case CV_32F :
//            return static_cast<T>( mat.at<float>(row,col) );

//        case CV_64F :
//            return static_cast<T>( mat.at<double>(row,col) );

//        default:
//            throw std::runtime_error("vision::get: unsupported OpenCV type.");
//    }
//}

//template<typename T> void set( cv::Mat &mat, int row, int col, T val)
//{
//    switch( mat.type() )
//    {
//        case CV_8U :
//            mat.at<uint8_t>(row,col) = static_cast<uint8_t>(val);
//            break;

//        case CV_8S :
//            mat.at<int8_t>(row,col) = static_cast<int8_t>(val);
//            break;

//        case CV_16U :
//            mat.at<uint16_t>(row,col) = static_cast<uint16_t>(val);
//            break;

//        case CV_16S :
//            mat.at<int16_t>(row,col) = static_cast<int16_t>(val);
//            break;

//        case CV_32S :
//            mat.at<int32_t>(row,col) = static_cast<int32_t>(val);
//            break;

//        case CV_32F :
//            mat.at<float>(row,col) = static_cast<float>(val);
//            break;

//        case CV_64F :
//            mat.at<double>(row,col) = static_cast<double>(val);
//            break;

//        default:
//            throw std::runtime_error("vision::set: unsupported OpenCV type.");
//    }
//}


/////
// convert images
///

//// convert image from eos to openCV
//template< typename T>
//inline void convert( const eos::util::image<T> &image, cv::Mat& imageCV )
//{
//    // init output
//    imageCV.create( image.height(), image.width(), CV_8UC3 );

//    // convert it to the openCV internal format
//    for( size_t y=0; y<image.height(); y++ )
//        for( size_t x=0; x<image.width(); x++ )
//            for( size_t i=0; i<3; i++ )
//                imageCV.at<unsigned char[3]>(y,x)[i] = static_cast<unsigned char>( image( x, y, i ) );
//}

//// convert image from openCV to eos
//template< typename T>
//inline void convert( const cv::Mat& imageCV, eos::util::image<T> &image )
//{
//    // check if the opencv image is stored continuously
//    if( !imageCV.isContinuous() )
//        throw std::runtime_error( "vision::convert: the openCV image is not stored continuously." );

//    // determine the image format
//    int channels = imageCV.channels();
//    int format;
//    switch( channels )
//    {
//        case 1 : format = eos::util::image<T>::intensity; break;
//        case 2 : format = eos::util::image<T>::intensity_alpha; break;
//        case 3 : format = eos::util::image<T>::RGB; break;
//        case 4 : format = eos::util::image<T>::RGBA; break;
//        default :
//            throw std::runtime_error( "vision::convert: unsupported openCV image format." );
//    }

//    // init output
//    image.set_pixels( imageCV.data, format, imageCV.size().width, imageCV.size().height );
//}


///////
//// 3D Calibration
/////

//template< typename T>
//inline bool findChessboardCorners( const cv::Mat &image,
//                                   size_t columns,
//                                   size_t rows   ,
//                                   std::vector< cv::Point2f > &corners )
//{
//    bool found = cv::findChessboardCorners( image, cv::Size( columns, rows ), corners );//, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK );

//    if( found )
//    {
//        // try to refine the corners (example from the opencv doc)
//        cv::Mat grayImage;
//        cv::cvtColor(image, grayImage, CV_RGB2GRAY);
//        cv::cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 0.1 ));
//    }

//    // return result
//    return found;
//}


//// TODO: convert to blas types as soon as template aliasing is available
//template< typename T >
//inline void projectPoints( const Eigen::Matrix<T,3,3> &K,
//                           const Eigen::Matrix<T,4,4> &Rt,
//                           const std::vector< Eigen::Matrix<T,3,1> > &points,
//                           std::vector< Eigen::Matrix<T,2,1> > &projectedPoints )
//{
//    // init stuff
//    Eigen::Matrix<T,3,1> X;
//    Eigen::Matrix<T,4,1> Xcam;
//    Eigen::Matrix<T,3,4> I = Eigen::Matrix<T,3,4>::Identity();

//    // transform points
//    projectedPoints.clear();
//    for( size_t i=0; i<points.size(); i++ )
//    {
//        // bring to camera space
//        Xcam = Rt * Eigen::Matrix<T,4,1>( points[i].x(),
//                                          points[i].y(),
//                                          points[i].z(),
//                                          1 );

//        // bring to image space
//        X = K * I * Xcam;

//        // push back the homogenized result
//        projectedPoints.push_back( Eigen::Matrix<T,2,1>( X.x()/X.z(), X.y()/X.z() ) );
//    }
//}


//template< typename T >
//inline void projectPoints( const Eigen::Matrix<T,4,4> &P,
//                           const std::vector< Eigen::Matrix<T,3,1> > &points,
//                           std::vector< Eigen::Matrix<T,2,1> > &projectedPoints )
//{
//    // init stuff
//    Eigen::Matrix<T,4,1> X;

//    // transform points
//    projectedPoints.clear();
//    for( size_t i=0; i<points.size(); i++ )
//    {
//        // bring to image space
//        X = P * Eigen::Matrix<T,4,1>( points[i].x(),
//                                      points[i].y(),
//                                      points[i].z(),
//                                      1 );

//        // push back the homogenized result
//        projectedPoints.push_back( Eigen::Matrix<T,2,1>( X.x()/X.z(), X.y()/X.z() ) );
//    }
//}



} // end namespace iris


