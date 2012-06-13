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


#include <iris/vision.hpp>
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


void OpenCVCalibration::configure( bool fixPrincipalPoint, bool fixAspectRatio, bool tangentialDistortion )
{
    m_fixPrincipalPoint = fixPrincipalPoint;
    m_fixAspectRatio = fixAspectRatio;
    m_tangentialDistortion = tangentialDistortion;
}


void OpenCVCalibration::calibrate()
{
    // run over all cameras and calibrate them
    for( std::map< size_t, Camera >::iterator it = m_cameras.begin(); it != m_cameras.end(); it++ )
        calibrateCamera( (*it).second );
}


void OpenCVCalibration::calibrateCamera( Calibration::Camera& cam )
{
    // init stuff
    std::vector< std::vector<cv::Point2f> > cvVectorPoints2D;
    std::vector< std::vector<cv::Point3f> > cvVectorPoints3D;
    Eigen::Vector2i imageSize = cam.imageSize;

    // run over all the poses of this camera and assemble the correspondences
    for( size_t i=0; i<cam.poses.size(); i++ )
    {
        // init stuff
        std::vector<cv::Point2f> cvPoints2D;
        std::vector<cv::Point3f> cvPoints3D;
        const std::vector<Eigen::Vector2d>& points2D = cam.poses[i].points2D;
        const std::vector<Eigen::Vector3d>& points3D = cam.poses[i].points3D;

        // convert the points
        for( size_t j=0; j<points2D.size(); j++ )
        {
            cvPoints2D.push_back( cv::Point2f( points2D[j](0), points2D[j](1) ) );
            cvPoints3D.push_back( cv::Point3f( points3D[j](0), points3D[j](1), points3D[j](2) ) );
        }

        // add them to the opemcv vectors
        cvVectorPoints2D.push_back( cvPoints2D );
        cvVectorPoints3D.push_back( cvPoints3D );
    }

    // try to compute the intrinsic and extrinsic parameters
    cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64F);
    cv::Mat distCoeff(5,1,CV_64F);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    double error = cv::calibrateCamera( cvVectorPoints3D,
                                        cvVectorPoints2D,
                                        cv::Size( imageSize(0), imageSize(1) ),
                                        cameraMatrix,
                                        distCoeff,
                                        rotationVectors,
                                        translationVectors,
                                        flags() );

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
        // get the rotation
        cv::Mat rotationCV( 3, 3, rotationVectors[i].type() );
        cv::Rodrigues( rotationVectors[i], rotationCV );
        Eigen::Matrix3f rotation;
        cv::cv2eigen( rotationCV, rotation );

        // get the translation
        Eigen::Vector3f translation;
        cv::cv2eigen( translationVectors[i], translation );

        // convert to a blas transformation
        Eigen::Affine3f trans;
        trans.setIdentity();
        trans.translate( translation );

        trans.rotate( rotation );

        // store the transformation
        cam.poses[i].transformation = trans.matrix().cast<double>();

        // reproject points from the opencv poses
        std::vector< cv::Point2f > projected;
        cv::projectPoints( cv::Mat(cvVectorPoints3D[i]), rotationVectors[i], translationVectors[i], cameraMatrix, distCoeff, projected );
        cam.poses[i].projected2D.clear();
        for( size_t p=0; p<projected.size(); p++ )
            cam.poses[i].projected2D.push_back( Eigen::Vector2d( projected[p].x, projected[p].y ) );
    }
}


int OpenCVCalibration::flags()
{
    // init stuff
    int result = 0;

    if( m_fixPrincipalPoint )
        result = result | CV_CALIB_FIX_PRINCIPAL_POINT;

    if( m_fixAspectRatio )
        result = result | CV_CALIB_FIX_ASPECT_RATIO;

    if( !m_tangentialDistortion )
        result = result | CV_CALIB_ZERO_TANGENT_DIST;

    return result;
}


} // end namespace iris

