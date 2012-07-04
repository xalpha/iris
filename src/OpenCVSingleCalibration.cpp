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
 * OpenCVSingleCalibration.cpp
 *
 *  Created on: Jun 14, 2012
 *      Author: duliu
 */

#ifdef IRIS_OPENMP
#include <omp.h>
#endif


#include <iris/OpenCVSingleCalibration.hpp>


namespace iris {

OpenCVSingleCalibration::OpenCVSingleCalibration() : OpenCVCalibration()
{
}


OpenCVSingleCalibration::~OpenCVSingleCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVSingleCalibration::configure( bool fixPrincipalPoint, bool fixAspectRatio, bool tangentialDistortion )
{
    m_fixPrincipalPoint = fixPrincipalPoint;
    m_fixAspectRatio = fixAspectRatio;
    m_tangentialDistortion = tangentialDistortion;
}


void OpenCVSingleCalibration::calibrate()
{
    // check that all is OK
    check();

    // run over all cameras and calibrate them
    for( auto it = m_cameras.begin(); it != m_cameras.end(); it++ )
    {
        // update stuff
        std::vector< Pose_d >& poses = it->second.poses;
        size_t poseCount = poses.size();

        // run feature detection
        #pragma omp parallel for
        for( size_t p=0; p<poseCount; p++ )
        {
            m_finder->find( poses[p] );
        }
    }

    // filter the poses
    filter();

    // calibrate all cameras
    for( auto it = m_filteredCameras.begin(); it != m_filteredCameras.end(); it++ )
        calibrateCamera( (*it).second, flags() );

    // commit the calibration calibrated frames
    commit();
}


void OpenCVSingleCalibration::calibrateCamera( Camera_d &cam, int flags )
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


void OpenCVSingleCalibration::filter()
{
    // init stuff
    m_filteredCameras.clear();

    // run over all the poses and only
    for( auto it = m_cameras.begin(); it != m_cameras.end(); it++ )
    {
        // update stuff
        std::vector<size_t> pointIndices;

        for( size_t p=0; p<it->second.poses.size(); p++ )
        {
            // guilty untill proven innocent
            it->second.poses[p].rejected = true;

            // check that all is well
            if( it->second.poses[p].pointIndices.size() > 0 )
            {
                // get the point indices of first pose
                if( pointIndices.size() == 0 )
                    pointIndices = it->second.poses[p].pointIndices;

                // check that the indices match with first pose
                if( it->second.poses[p].pointIndices == pointIndices )
                    m_filteredCameras[it->second.id].poses.push_back( it->second.poses[p] );
            }
        }

        // set image size
        m_filteredCameras[it->second.id].id = it->second.id;
        m_filteredCameras[it->second.id].imageSize = it->second.imageSize;
    }
}


int OpenCVSingleCalibration::flags()
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

