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
 * OpenCVCalibration.cpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */


#include <iris/OpenCVCalibration.hpp>


namespace iris {

OpenCVCalibration::OpenCVCalibration() :
    Calibration(),
    m_fixPrincipalPoint( false ),
    m_fixAspectRatio( true ),
    m_tangentialDistortion( true )
{
}


OpenCVCalibration::~OpenCVCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVCalibration::calibrateCamera( Camera_d &cam, int flags )
{
    // init stuff
    std::vector< std::vector<cv::Point2f> > cvVectorPoints2D;
    std::vector< std::vector<cv::Point3f> > cvVectorPoints3D;

    // run over all the poses of this camera and assemble the correspondences
    for( size_t i=0; i<cam.poses.size(); i++ )
    {
        cvVectorPoints2D.push_back( iris::eigen2cv<float>( cam.poses[i].points2D ) );
        cvVectorPoints3D.push_back( iris::eigen2cv<float>( cam.poses[i].points3D ) );
    }

    // try to compute the intrinsic and extrinsic parameters
    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
    cv::Mat distCoeff(5,1,CV_64F);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    double error = cv::calibrateCamera( cvVectorPoints3D,
                                        cvVectorPoints2D,
                                        cv::Size( cam.imageSize(0), cam.imageSize(1) ),
                                        cameraMatrix,
                                        distCoeff,
                                        rotationVectors,
                                        translationVectors,
                                        flags );

    // instrinsic matrix
    cv::cv2eigen( cameraMatrix, cam.intrinsic );

    // distortion coefficeints
    cam.distortion.clear();
    for( int i=0; i<distCoeff.size().width; i++ )
        cam.distortion.push_back( distCoeff.at<double>( i, 0 ) );

    // error
    cam.error = error;

    // compute and save the poses
    for( size_t i=0; i<rotationVectors.size(); i++ )
    {
        // store the transformation
        iris::cv2eigen( rotationVectors[i], translationVectors[i], cam.poses[i].transformation );

        // reproject points from the opencv poses
        cam.poses[i].projected2D = projectPoints( cvVectorPoints3D[i],
                                                  rotationVectors[i],
                                                  translationVectors[i],
                                                  cameraMatrix,
                                                  distCoeff );
    }
}


std::vector< Eigen::Vector2d > OpenCVCalibration::projectPoints( const std::vector<cv::Point3f> points3D,
                                                                 const cv::Mat& rot,
                                                                 const cv::Mat& transl,
                                                                 const cv::Mat& cameraMatrix,
                                                                 const cv::Mat& distCoeff )
{
    // init stuff
    std::vector< cv::Point2f > projected;
    std::vector< Eigen::Vector2d > result;

    // project points and store
    cv::projectPoints( cv::Mat(points3D), rot, transl, cameraMatrix, distCoeff, projected );
    for( size_t p=0; p<projected.size(); p++ )
        result.push_back( Eigen::Vector2d( projected[p].x, projected[p].y ) );

    // return
    return result;
}


} // end namespace iris

