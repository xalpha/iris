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

#include <tinyxml2.h>

#include <iris/CameraCalibration.hpp>

namespace iris {

CameraCalibration::CameraCalibration() :
    m_finder(0),
    m_poseCount(0),
    m_handEye(false)
{
    // use all available threads
#ifdef IRIS_OPENMP
    omp_set_num_threads( omp_get_max_threads() );
#endif
}


CameraCalibration::~CameraCalibration() {
	// TODO Auto-generated destructor stub
}


size_t CameraCalibration::addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const std::string& name, const size_t cameraID )
{
    // assemble the pose
    Pose_d pose;
    pose.id = m_poseCount;
    pose.name = name;
    pose.image = image;

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


template<typename C>
inline void appendTextElement( tinyxml2::XMLDocument& doc,
                               tinyxml2::XMLNode& node,
                               std::basic_string<C> name,
                               std::basic_string<C> val )
{
    tinyxml2::XMLNode* tmp = node.InsertEndChild( doc.NewElement( name.c_str() ) );
    tmp->InsertEndChild( doc.NewText( val.c_str() ));
}


void CameraCalibration::save( const std::string& filename )
{
    // init stuff
    tinyxml2::XMLDocument doc;

    // add root
    tinyxml2::XMLNode* root = doc.InsertEndChild( doc.NewElement( "CameraCalibration" ) );

    // run over the camers
    tinyxml2::XMLNode* cameras = root->InsertEndChild( doc.NewElement( "Cameras" ) );
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
    {
        // init camera
        tinyxml2::XMLNode* camera = cameras->InsertEndChild( doc.NewElement( "Camera" ) );

        // add camera properties
        appendTextElement( doc, *camera, std::string("Id"), toString(camIt->second.id) );
        appendTextElement( doc, *camera, std::string("ImageSize"), toString( camIt->second.imageSize ) );
        appendTextElement( doc, *camera, std::string("Intrinsic"), toString( camIt->second.intrinsic ) );
        appendTextElement( doc, *camera, std::string("Distortion"), toString( camIt->second.distortion ) );
        appendTextElement( doc, *camera, std::string("Error"), toString( camIt->second.error ) );

        // run over all its poses and add them
        tinyxml2::XMLNode* poses = camera->InsertEndChild( doc.NewElement( "Poses" ) );
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
        {
            // add the pose
            tinyxml2::XMLNode* pose = poses->InsertEndChild( doc.NewElement( "Pose" ) );

            // pose identification
            appendTextElement( doc, *pose, std::string("Id"), toString(camIt->second.poses[p].id) );
            appendTextElement( doc, *pose, std::string("Name"), camIt->second.poses[p].name );

            // if not rejected, also add the rest
            if( !camIt->second.poses[p].rejected )
            {
                appendTextElement( doc, *pose, std::string("Points2D"), toString( camIt->second.poses[p].points2D ) );
                appendTextElement( doc, *pose, std::string("Points3D"), toString( camIt->second.poses[p].points3D ) );
                appendTextElement( doc, *pose, std::string("PointIndices"), toString( camIt->second.poses[p].pointIndices ) );
                appendTextElement( doc, *pose, std::string("Transformation"), toString( camIt->second.poses[p].transformation ) );
                appendTextElement( doc, *pose, std::string("ProjectedPoints"), toString( camIt->second.poses[p].projected2D ) );
            }
        }
    }

    // save to disk
    doc.SaveFile( filename.c_str() );
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


void CameraCalibration::check()
{
    if( !m_finder )
        throw std::runtime_error("CameraCalibration: Finder not set.");
}


void CameraCalibration::threadID()
{
#ifdef IRIS_OPENMP
     int tid=omp_get_thread_num();
    int nThreads=omp_get_num_threads();
    #pragma omp critical
    {
        std::cout<<"Thread ID: "<< tid << " / " << nThreads <<std::endl;
    }
#endif
}


} // end namespace iris

