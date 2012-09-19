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

/*
 * CameraSet.hpp
 *
 *  Created on: Aug 9, 2012
 *      Author: duliu
 */

#include <tinyxml2.h>

#include <iris/util.hpp>

namespace iris
{

template <typename T>
class CameraSet
{
public:
    // constructor
    CameraSet();

    // destructor
    CameraSet( const CameraSet& cs );

    // add single image
    size_t add( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const std::string& name, const size_t cameraID=0 );

    // get the cameras
    std::map< size_t, iris::Camera<T> >& cameras();
    const std::map< size_t, iris::Camera<T> >& cameras() const;

    // get a particular camera
    bool hasCamera( const size_t id=0 ) const;
    Camera_d& camera( const size_t id=0 );
    const Camera_d& camera( const size_t id=0 ) const;

    // get a particular pose
    bool hasPose( const size_t id ) const;
    bool hasPose( const std::string& name ) const;
    const Pose_d& pose( const size_t id ) const;
    const Pose_d& pose( const std::string& name ) const;

    // number of poses over all cameras
    size_t poseCount();

    // save to disk
    void save( const std::string& filename );

    // load from disk
    void load( const std::string& filename );

    // copy operator
    void operator =( const CameraSet& cam );

private:
    void appendTextElement( tinyxml2::XMLDocument& doc, tinyxml2::XMLNode& node, std::string name, std::string val );

    std::string getElementValue( tinyxml2::XMLNode* node, std::string name );

private:
    size_t m_poseCount;
    std::map< size_t, iris::Camera<T> > m_cameras;
};
typedef CameraSet<double> CameraSet_d;


/////
// Implementation
///

template <typename T>
inline CameraSet<T>::CameraSet() :
    m_poseCount(0)
{
}


template <typename T>
inline CameraSet<T>::CameraSet( const CameraSet& cs )
{
    *this = cs;
}


template <typename T>
inline size_t CameraSet<T>::add( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const std::string& name, const size_t cameraID )
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
            throw std::runtime_error( "CameraSet::addImage: pose has different image size than already stored poses." );
    }

    // increment pose count and return id
    m_poseCount++;
    return pose.id;
}


template <typename T>
inline std::map< size_t, iris::Camera<T> >& CameraSet<T>::cameras()
{
    return m_cameras;
}


template <typename T>
inline const std::map< size_t, iris::Camera<T> >& CameraSet<T>::cameras() const
{
    return m_cameras;
}


template <typename T>
inline bool CameraSet<T>::hasCamera( const size_t id ) const
{
    // find the camera
    std::map< size_t, Camera_d >::const_iterator camIt = m_cameras.find( id );
    return camIt != m_cameras.end();
}


template <typename T>
inline Camera_d& CameraSet<T>::camera( const size_t id )
{
    // find the camera
    std::map< size_t, Camera_d >::iterator camIt = m_cameras.find( id );

    // search for the camera
    if( camIt != m_cameras.end() )
        return (*camIt).second;
    else
        throw std::runtime_error("CameraSet::camera: camera not found.");
}


template <typename T>
inline const Camera_d& CameraSet<T>::camera( const size_t id ) const
{
    // find the camera
    std::map< size_t, Camera_d >::const_iterator camIt = m_cameras.find( id );

    // search for the camera
    if( camIt != m_cameras.end() )
        return (*camIt).second;
    else
        throw std::runtime_error("CameraSet::camera: camera not found.");
}


template <typename T>
inline bool CameraSet<T>::hasPose( const size_t id ) const
{
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            if( camIt->second.poses[p].id == id )
                return true;

    // nothing found
    return false;
}


template <typename T>
inline bool CameraSet<T>::hasPose( const std::string& name ) const
{
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            if( camIt->second.poses[p].name.compare( name ) == 0 )
                return true;

    // nothing found
    return false;
}


template <typename T>
inline const Pose_d& CameraSet<T>::pose( const size_t id ) const
{
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            if( camIt->second.poses[p].id == id )
                return camIt->second.poses[p];

    // nothing found
    throw std::runtime_error("CameraSet::pose: pose not found.");
}


template <typename T>
inline const Pose_d& CameraSet<T>::pose( const std::string& name ) const
{
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            if( camIt->second.poses[p].name.compare( name ) == 0 )
                return camIt->second.poses[p];

    // nothing found
    throw std::runtime_error("CameraSet::pose: pose \"" + name + "\" not found.");
}


template <typename T>
inline size_t CameraSet<T>::poseCount()
{
    // count all poses over all cameras
    size_t pc = 0;
    for( auto camIt=m_cameras.begin(); camIt != m_cameras.end(); camIt++ )
        pc += camIt->second.poses.size();
    return pc;
}


template <typename T>
inline void CameraSet<T>::save( const std::string& filename )
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


template <typename T>
inline void CameraSet<T>::load( const std::string& filename )
{
    // load the document
    tinyxml2::XMLDocument doc;
    int loadStatus = doc.LoadFile( filename.c_str() );
    if( tinyxml2::XML_SUCCESS != loadStatus )
        throw std::runtime_error( "CameraSet::load: " + std::string( doc.GetErrorStr1() ) );

    // get the root
    tinyxml2::XMLNode* root = doc.FirstChildElement( "CameraCalibration" );
    if( root == 0 )
        throw std::runtime_error( "CameraSet::load: root node not found." );

    // get the cameras
    tinyxml2::XMLNode* cameras = root->FirstChildElement( "Cameras" );
    if( cameras == 0 )
        throw std::runtime_error( "CameraSet::load: no cameras." );

    // run over all the cameras
    m_cameras.clear();
    for( tinyxml2::XMLNode* camPtr=cameras->FirstChildElement( "Camera" ); camPtr != 0; camPtr = camPtr->NextSiblingElement( "Camera" ) )
    {
        Camera<T> camera;

        // get camera parameters
        str2scalar( getElementValue( camPtr, "Id" ), camera.id );
        str2eigen( getElementValue( camPtr, "ImageSize" ), camera.imageSize );
        str2eigen( getElementValue( camPtr, "Intrinsic" ), camera.intrinsic );
        str2vector( getElementValue( camPtr, "Distortion" ), camera.distortion );

        // read the poses
        tinyxml2::XMLNode* poses = camPtr->FirstChildElement( "Poses" );
        if( poses != 0 )
        {
            for( tinyxml2::XMLNode* posePtr=poses->FirstChildElement( "Pose" ); posePtr != 0; posePtr = posePtr->NextSiblingElement( "Pose" ) )
            {
                Pose<T> pose;

                // get the pose attributes
                str2scalar( getElementValue( posePtr, "Id" ), pose.id );
                pose.name = getElementValue( posePtr, "Name" );
                str2eigenVector( getElementValue( posePtr, "Points2D" ), pose.points2D );
                str2eigenVector( getElementValue( posePtr, "Points3D" ), pose.points3D );
                str2vector( getElementValue( posePtr, "PointIndices" ), pose.pointIndices );
                str2eigen( getElementValue( posePtr, "Transformation" ), pose.transformation );
                str2eigenVector( getElementValue( posePtr, "ProjectedPoints" ), pose.projected2D );

                pose.rejected = pose.pointIndices.size() == 0;

                if( !pose.rejected &&
                    ( pose.points2D.size() != pose.pointIndices.size() ||
                      pose.points2D.size() != pose.points3D.size() ) )
                {
                    pose.rejected = false;
                    std::cerr << "CameraSet::load: point arrays' sizes do not match:";
                    std::cerr << " Points3D=" << pose.points2D.size();
                    std::cerr << ", Points3D=" << pose.points3D.size();
                    std::cerr << ", PointIndices=" << pose.pointIndices.size() << "." << std::endl;
                }

                camera.poses.push_back( pose );
            }
        }

        m_cameras[ camera.id ] = camera;
    }
}


template <typename T>
inline void CameraSet<T>::operator =( const CameraSet& cam )
{
    m_poseCount = cam.m_poseCount;
    m_cameras = cam.m_cameras;
}


template <typename T>
inline void CameraSet<T>::appendTextElement( tinyxml2::XMLDocument& doc,
                                             tinyxml2::XMLNode& node,
                                             std::string name,
                                             std::string val )
{
    tinyxml2::XMLNode* tmp = node.InsertEndChild( doc.NewElement( name.c_str() ) );
    tmp->InsertEndChild( doc.NewText( val.c_str() ));
}


template <typename T>
inline std::string CameraSet<T>::getElementValue( tinyxml2::XMLNode* node, std::string name )
{
    tinyxml2::XMLNode* childTag = node->FirstChildElement( name.c_str() );
    if( childTag != 0 )
    {
        tinyxml2::XMLNode* childText = childTag->FirstChild();
        if( childText != 0 )
            return std::string( childText->Value() );
    }
        return std::string();
}



} // end namespace iris
