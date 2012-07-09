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
 * CameraCalibration.cpp
 *
 *  Created on: Jun 5, 2011
 *      Author: duliu
 */

#include <iostream>

#ifdef IRIS_OPENMP
#include <omp.h>
#endif

#include <Eigen/Geometry>

#include <iris/CameraCalibration.hpp>

namespace iris {

CameraCalibration::CameraCalibration() :
    m_finder(0),
    m_poseCount(0),
    m_handEye(false)
{
    // use all available threads
    omp_set_num_threads( omp_get_max_threads() );
}


CameraCalibration::~CameraCalibration() {
	// TODO Auto-generated destructor stub
}


size_t CameraCalibration::addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const size_t cameraID )
{
    return addImage( image, Eigen::Matrix4d::Zero(), cameraID );
}


size_t CameraCalibration::addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const Eigen::Matrix4d& relativePose, const size_t cameraID )
{
    // assemble the pose
    Pose_d pose;
    pose.id = m_poseCount;
    pose.image = image;
    pose.handTrans = relativePose;

    // add the pose
    m_cameras[cameraID].poses.push_back( pose );

    // get the image size
    Eigen::Vector2i imageSize( image->width(), image->height() );

    // make sure the image sizes are the same
    if( m_cameras[cameraID].poses.size() == 1 )
    {
        m_cameras[cameraID].id = cameraID;
        m_cameras[cameraID].imageSize = imageSize;
    }
    else
    {
        Eigen::Vector2i tmp = imageSize - m_cameras[cameraID].imageSize;
        if( (tmp(0) != 0) || (tmp(1) != 0) )
            throw std::runtime_error( "CameraCalibration::addImage: pose has different image size than already stored poses." );
    }

    // increment pose count and return id
    m_poseCount++;
    return pose.id;
}


void CameraCalibration::clear()
{
    m_poseCount = 0;
    m_cameras.clear();
}



void CameraCalibration::setFinder( std::shared_ptr<Finder> finder )
{
    m_finder = finder;
}


const Finder& CameraCalibration::finder() const
{
    return *m_finder;
}


const std::map< size_t, iris::Camera_d >& CameraCalibration::cameras() const
{
    return m_cameras;
}


const Camera_d& CameraCalibration::camera( const size_t id ) const
{
    // find the camera
    std::map< size_t, Camera_d >::const_iterator camIt = m_cameras.find( id );

    // search for the camera
    if( camIt != m_cameras.end() )
        return (*camIt).second;
    else
        throw std::runtime_error("CameraCalibration::camera: camera not found.");
}


const Pose_d& CameraCalibration::pose( const size_t id ) const
{
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            if( camIt->second.poses[p].id == id )
                return camIt->second.poses[p];

    // nothing found
    throw std::runtime_error("IrisCC::pose: pose not found.");
}


const size_t CameraCalibration::poseCount() const
{
    // count all poses over all cameras
    size_t pc = 0;
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        pc += camIt->second.poses.size();
    return pc;
}


void CameraCalibration::copyCameras( std::shared_ptr<CameraCalibration> cc )
{
    if( cc.get() != 0 )
        m_cameras = cc->m_cameras;
}


void CameraCalibration::commit()
{
    for( auto camIt=m_filteredCameras.begin(); camIt != m_filteredCameras.end(); camIt++ )
    {
        // get the poses for the target
        std::vector<Pose_d>& srcPoses = camIt->second.poses;
        std::vector<Pose_d>& targetPoses = m_cameras[ camIt->second.id ].poses;

        // run over all found poses and commit
        for( size_t p=0; p<srcPoses.size(); p++ )
        {
            // find the corresponding target pose
            for( size_t tp=0; tp<targetPoses.size(); tp++ )
            {
                if( targetPoses[tp].id == srcPoses[p].id )
                {
                    targetPoses[tp] = srcPoses[p];
                    targetPoses[tp].rejected = false;
                }
            }
        }

        // get camera params
        m_cameras[ camIt->second.id ].intrinsic = camIt->second.intrinsic;
        m_cameras[ camIt->second.id ].distortion = camIt->second.distortion;
    }
}


void CameraCalibration::calibrateHandEye( Camera_d& cam )
{
    // init stuff
    size_t poseCount = 0;
    Eigen::MatrixXd tA(3*poseCount,3);
    Eigen::VectorXd tB(3*poseCount);

    // generate rotation and translation vectors 3 everything
    std::vector< Eigen::Quaterniond > eyeR, handR;
    std::vector< Eigen::Vector3d > eyeT, handT;
    for( size_t i=0; i<cam.poses.size(); i++)
    {
        if( !cam.poses[i].rejected && cam.poses[i].handTrans.norm() > 1.0 )
        {
            // eye
            eyeR.push_back( Eigen::Quaterniond( Eigen::Affine3d( cam.poses[i].eyeTrans ).rotation() ) );
            eyeT.push_back( Eigen::Vector3d( Eigen::Affine3d( cam.poses[i].eyeTrans ).translation() ) );

            // hand
            handR.push_back( Eigen::Quaterniond( Eigen::Affine3d( cam.poses[i].handTrans ).rotation() ) );
            handT.push_back( Eigen::Vector3d( Eigen::Affine3d( cam.poses[i].handTrans ).translation() ) );

            poseCount++;
        }
    }

    // check if there are enough poses
    // TODO: how many poses is enough
    if( poseCount < 5 )
    {
        std::cerr << "CameraCalibration::calibrateHandEye: not enough poses." << std::endl;
        return;
    }

    // Rotation
    // assemble the LSE
    for( size_t i=0; i<poseCount; i++)
    {
        // update stuff
        Eigen::Matrix3d cm = crossMatrix( Eigen::Vector3d( handR[i].vec() + eyeR[i].vec() ) );
        Eigen::Vector3d deltaR = eyeR[i].vec() - handR[i].vec();

        // assemble
        tA.block<3,3>(3*i,0) = cm;
        tB.block<3,1>(3*i,0) = deltaR;
    }

    // Do a QR decomposition and get the rotation R
    Eigen::FullPivHouseholderQR<Eigen::Matrix3d> jQR(tA);
    Eigen::VectorXd tPcg_ = jQR.solve(tB);

    Eigen::Vector3d Pcg(tPcg_(0), tPcg_(1), tPcg_(2));
    Pcg = (2.0/sqrt(1.0+Pcg.dot(Pcg))) * Pcg;

    Eigen::Matrix3d R = (1.0 - 0.5*Pcg.dot(Pcg)) * Eigen::Matrix3d::Identity() +
                        0.5 * (vectorProduct(Pcg,Pcg) + sqrt( 4.0 - Pcg.dot(Pcg)) *  crossMatrix( Pcg ) );

    // Translation
    // assemble the LSE
    for( size_t i=0; i<poseCount; i++)
    {
        Eigen::Matrix3d tmpA = Eigen::Matrix3d( handR[i].matrix() ) - Eigen::Matrix3d::Identity();
        Eigen::Vector3d tmpB = R*eyeT[i] - handT[i];

        // assemble
        tA.block<3,3>(3*i,0) = tmpA;
        tB.block<3,1>(3*i,0) = tmpB;
    }

    // Do another QR decomposition and get the translation T
    jQR = Eigen::FullPivHouseholderQR<Eigen::Matrix3d>(tA);
    Eigen::VectorXd tTcg = jQR.solve(tB);
    Eigen::Vector3d T( tTcg(0), tTcg(1), tTcg(2) );

    // set the result
    Eigen::Affine3d trans;
    trans.translate( T );
    trans.rotate( R );

    // set the result
    cam.handEye = trans.matrix();
}


void CameraCalibration::check()
{
    if( !m_finder )
        throw std::runtime_error("CameraCalibration: Finder not set.");
}


void CameraCalibration::threadID()
{
    int tid=omp_get_thread_num();
    int nThreads=omp_get_num_threads();
    #pragma omp critical
    {
        std::cout<<"Thread ID: "<< tid << " / " << nThreads <<std::endl;
    }
}


} // end namespace iris

