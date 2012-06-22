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


#include <iris/util.hpp>
#include <iris/OpenCVStereoCalibration.hpp>


namespace iris {

OpenCVStereoCalibration::OpenCVStereoCalibration() :
    OpenCVCalibration(),
    m_relativeToPattern(true),
    m_sameFocalLength(false)
{
}


OpenCVStereoCalibration::~OpenCVStereoCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVStereoCalibration::configure( bool relativeToPattern,
                                         bool fixPrincipalPoint,
                                         bool fixAspectRatio,
                                         bool sameFocalLength,
                                         bool tangentialDistortion )
{
    m_relativeToPattern = relativeToPattern;
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
    Camera_d& cam1 = m_cameras.begin()->second;
    Camera_d& cam2 = (m_cameras.begin()++)->second;

    // check that both cameras have the same number of poses
    size_t poseCount = cam1.poses.size();
    if( cam1.poses.size() != cam2.poses.size() )
        throw std::runtime_error("OpenCVStereoCalibration::calibrate: cameras don't have the same number of poses.");

    // init stuff
    std::vector< std::vector<cv::Point2f> > cvVectorPoints2D_1;
    std::vector< std::vector<cv::Point2f> > cvVectorPoints2D_2;
    std::vector< std::vector<cv::Point3f> > cvVectorPoints3D;

    // run over all the poses of this camera and assemble the correspondences
    for( size_t i=0; i<poseCount; i++ )
    {
        // check that the number of correspondences is the same for both cameras
        if( cam1.poses[i].points2D.size() != cam2.poses[i].points2D.size() )
            throw std::runtime_error("OpenCVStereoCalibration::calibrate: cameras don't have the same number of correspondences in this pose.");

        // add them to the opemcv vectors
        cvVectorPoints2D_1.push_back( iris::eigen2cv<float>( cam1.poses[i].points2D ) );
        cvVectorPoints2D_2.push_back( iris::eigen2cv<float>( cam2.poses[i].points2D ) );
        cvVectorPoints3D.push_back( iris::eigen2cv<float>( cam1.poses[i].points3D ) );
    }

    // try to compute the intrinsic and extrinsic parameters
    cv::Mat A_1 = cv::Mat::eye(3,3,CV_64F); // intrinsic matrix 1
    cv::Mat A_2 = cv::Mat::eye(3,3,CV_64F); // intrinsic matrix 2
    cv::Mat E = cv::Mat::eye(3,3,CV_64F); // essential matrix
    cv::Mat F = cv::Mat::eye(3,3,CV_64F); // fundamental matrix
    cv::Mat dc_1(5,1,CV_64F); // distortion coefficients
    cv::Mat dc_2(5,1,CV_64F); // distortion coefficients
    cv::Mat R; // rotation
    cv::Mat T; // translation
    double error = cv::stereoCalibrate( cvVectorPoints3D,
                                        cvVectorPoints2D_1,
                                        cvVectorPoints2D_2,
                                        A_1,
                                        dc_1,
                                        A_2,
                                        dc_2,
                                        cv::Size( cam1.imageSize(0), cam1.imageSize(1) ),
                                        R, T, E, F,
                                        cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
                                        flags() );

    // instrinsic matrix
    cv::cv2eigen( A_1, cam1.intrinsic );
    cv::cv2eigen( A_2, cam2.intrinsic );

    // distortion coefficeints
    cam1.distortion.clear();
    cam2.distortion.clear();
    for( int i=0; i<dc_1.size().width; i++ )
    {
        cam1.distortion.push_back( dc_1.at<double>( i, 0 ) );
        cam2.distortion.push_back( dc_2.at<double>( i, 0 ) );
    }

    // error
    cam1.error = error;
    cam2.error = error;

    // compute the poses
    std::vector<cv::Mat> rVec_cam1, tVec_cam1;
    cv::solvePnP( cvVectorPoints3D, cvVectorPoints2D_1, A_1, dc_1, rVec_cam1, tVec_cam1 );

    // convert and save the poses
    Eigen::Matrix4d RT;
    iris::cv2eigen( R, T, RT );
    for( size_t i=0; i<poseCount; i++ )
    {
        // get the extrinsics
        Eigen::Matrix4d trans_cam1;
        iris::cv2eigen( rVec_cam1[i], tVec_cam1[i], trans_cam1 );
        cam1.poses[i].transformation = trans_cam1;
        cam2.poses[i].transformation = trans_cam1 * RT;

        // now the ugly part, convert back to openCV for back projection
        cv::Mat rv1, rv2, tv1, tv2;
        iris::eigen2cv( cam1.poses[i].transformation, rv1, tv1 );
        iris::eigen2cv( cam2.poses[i].transformation, rv2, tv2 );

        // reproject points from the opencv poses
        cam1.poses[i].projected2D = projectPoints( cvVectorPoints3D[i], rv1, tv1, A_1, dc_1 );
        cam2.poses[i].projected2D = projectPoints( cvVectorPoints3D[i], rv2, tv2, A_2, dc_2 );
    }

    // if only the relative pose is desired, just blank it all
    for( size_t i=0; !m_relativeToPattern && i<poseCount; i++ )
    {
        // get the extrinsics
        cam1.poses[i].transformation = Eigen::Matrix4d::Identity();
        cam2.poses[i].transformation = RT;
    }
}


bool OpenCVStereoCalibration::multipleCameras()
{
    return true;
}


int OpenCVStereoCalibration::flags()
{
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

