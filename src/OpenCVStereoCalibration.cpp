////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of iris, a lightweight C++ camera calibration library    //
//                                                                            //
// Copyright (C) 2010, 2011 Alexandru Duliu                                   //
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
 * OpenCVStereoCalibration.cpp
 *
 *  Created on: Jun 12, 2012
 *      Author: duliu
 */


#include <iris/vision.hpp>
#include <iris/OpenCVStereoCalibration.hpp>


namespace iris {

OpenCVStereoCalibration::OpenCVStereoCalibration() : Calibration()
{
}


OpenCVStereoCalibration::~OpenCVStereoCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVStereoCalibration::configure( bool fixPrincipalPoint, bool fixAspectRatio, bool sameFocalLength, bool tangentialDistortion )
{
    m_fixPrincipalPoint = fixPrincipalPoint;
    m_fixAspectRatio = fixAspectRatio;
    m_sameFocalLength = sameFocalLength;
    m_tangentialDistortion = tangentialDistortion;
}


void OpenCVStereoCalibration::calibrate()
{
    // assuming there are only two cameras
    if( m_cameras.size() != 2 )
        throw std::runtime_error("OpenCVStereoCalibration::calibrate: exactly 2 cameras are required.");

    // get cameras
    Calibration::Camera& cam1 = m_cameras.begin()->second;
    Calibration::Camera& cam2 = (m_cameras.begin()++)->second;

    // check that both cameras have the same number of poses
    if( cam1.poses.size() != cam2.poses.size() )
        throw std::runtime_error("OpenCVStereoCalibration::calibrate: cameras don't have the same number of poses.");

    // init stuff
    std::vector< std::vector<cv::Point2f> > cvVectorPoints2D_1;
    std::vector< std::vector<cv::Point2f> > cvVectorPoints2D_2;
    std::vector< std::vector<cv::Point3f> > cvVectorPoints3D;
    Eigen::Vector2i imageSize = cam1.imageSize;

    // run over all the poses of this camera and assemble the correspondences
    for( size_t i=0; i<cam1.poses.size(); i++ )
    {
        // init stuff
        std::vector<cv::Point2f> cvPoints2D_1;
        std::vector<cv::Point2f> cvPoints2D_2;
        std::vector<cv::Point3f> cvPoints3D;
        const std::vector<Eigen::Vector2d>& points2D_1 = cam1.poses[i].points2D;
        const std::vector<Eigen::Vector2d>& points2D_2 = cam2.poses[i].points2D;
        const std::vector<Eigen::Vector3d>& points3D = cam1.poses[i].points3D;

        // check that the number of correspondences is the same for both cameras
        if( points2D_1.size() != points2D_2.size() )
            throw std::runtime_error("OpenCVStereoCalibration::calibrate: cameras don't have the same number of correspondences in this pose.");

        // convert the points
        for( size_t j=0; j<points2D_1.size(); j++ )
        {
            cvPoints2D_1.push_back( cv::Point2f( points2D_1[j](0), points2D_1[j](1) ) );
            cvPoints2D_2.push_back( cv::Point2f( points2D_2[j](0), points2D_2[j](1) ) );
            cvPoints3D.push_back( cv::Point3f( points3D[j](0), points3D[j](1), points3D[j](2) ) );
        }

        // add them to the opemcv vectors
        cvVectorPoints2D_1.push_back( cvPoints2D_1 );
        cvVectorPoints2D_2.push_back( cvPoints2D_2 );
        cvVectorPoints3D.push_back( cvPoints3D );
    }

    // try to compute the intrinsic and extrinsic parameters
    cv::Mat cameraMatrix_1 = cv::Mat::eye(3,3,CV_64F);
    cv::Mat cameraMatrix_2 = cv::Mat::eye(3,3,CV_64F);
    cv::Mat essentialMatrix = cv::Mat::eye(3,3,CV_64F);
    cv::Mat fundamentalMatrix = cv::Mat::eye(3,3,CV_64F);
    cv::Mat distCoeff_1(5,1,CV_64F);
    cv::Mat distCoeff_2(5,1,CV_64F);
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    double error = cv::stereoCalibrate( cvVectorPoints3D,
                                        cvVectorPoints2D_1,
                                        cvVectorPoints2D_2,
                                        cameraMatrix_1,
                                        distCoeff_1,
                                        cameraMatrix_2,
                                        distCoeff_2,
                                        cv::Size( imageSize(0), imageSize(1) ),
                                        rotationVectors,
                                        translationVectors,
                                        essentialMatrix,
                                        fundamentalMatrix );

    // instrinsic matrix
    cv::cv2eigen( cameraMatrix_1, cam1.intrinsic );
    cv::cv2eigen( cameraMatrix_2, cam2.intrinsic );

    // distortion coefficeints
    cam1.distortion.clear();
    cam2.distortion.clear();
    for( int i=0; i<distCoeff_1.size().width; i++ )
    {
        cam1.distortion.push_back( distCoeff_1.at<double>( i, 0 ) );
        cam2.distortion.push_back( distCoeff_2.at<double>( i, 0 ) );
    }

    // error
    cam1.error = error;
    cam2.error = error;

//    // compute and save the poses
//    for( size_t i=0; i<rotationVectors.size(); i++ )
//    {
//        // get the rotation
//        cv::Mat rotationCV( 3, 3, rotationVectors[i].type() );
//        cv::Rodrigues( rotationVectors[i], rotationCV );
//        Eigen::Matrix3f rotation;
//        cv::cv2eigen( rotationCV, rotation );

//        // get the translation
//        Eigen::Vector3f translation;
//        cv::cv2eigen( translationVectors[i], translation );

//        // convert to a blas transformation
//        Eigen::Affine3f trans;
//        trans.setIdentity();
//        trans.translate( translation );

//        trans.rotate( rotation );

//        // store the transformation
//        cam.poses[i].transformation = trans.matrix().cast<double>();

//        // reproject points from the opencv poses
//        std::vector< cv::Point2f > projected;
//        cv::projectPoints( cv::Mat(cvVectorPoints3D[i]), rotationVectors[i], translationVectors[i], cameraMatrix, distCoeff, projected );
//        cam.poses[i].projected2D.clear();
//        for( size_t p=0; p<projected.size(); p++ )
//            cam.poses[i].projected2D.push_back( Eigen::Vector2d( projected[p].x, projected[p].y ) );
//    }
}


int OpenCVStereoCalibration::flags()
{
    // init stuff
    int result = 0;

    if( m_fixPrincipalPoint )
        result = result | CV_CALIB_FIX_PRINCIPAL_POINT;

    if( m_fixAspectRatio )
        result = result | CV_CALIB_FIX_ASPECT_RATIO;

    if( m_sameFocalLength )
        result = result | CV_CALIB_SAME_FOCAL_LENGTH;

    if( !m_tangentialDistortion )
        result = result | CV_CALIB_ZERO_TANGENT_DIST;

    return result;
}


} // end namespace iris

