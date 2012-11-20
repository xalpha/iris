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


///
/// \file    OpenCVMultiCalibration.hpp.hpp
/// \class   OpenCVMultiCalibration.hpp
///
/// \author  Amin Abouee
/// \author  Alexandru Duliu
/// \date    Nov 1, 2012
///

#include <multiCalibrate.hpp>

#include <iris/OpenCVMultiCalibration.hpp>


namespace iris {

OpenCVMultiCalibration::OpenCVMultiCalibration() :
    CameraCalibration()
{
}


OpenCVMultiCalibration::~OpenCVMultiCalibration() {
	// TODO Auto-generated destructor stub
}


void OpenCVMultiCalibration::calibrate( CameraSet_d& cs )
{
    // this is where the actual work is performed
    //
    // the implementation is responsible for running the registered
    // finder, filtering, calibration and committing the results
    //
    // example implementations can be found under:
    // OpenCVSingleCalibration.hpp and OpenCVStereoCalibration.hpp
    if( cs.cameras().size() <= 3 )
        throw std::runtime_error("OpenCVStereoCalibration::calibrate: exactly 3 cameras are required.");

    // get refs of camera
    std::vector <iris::Camera_d> cameraRef ;
    for( auto it = cs.cameras().begin(); it != cs.cameras().end(); it++ )
    {
        cameraRef.push_back(it->second);
    }

//    iris::Camera_d& cam1 = cs.cameras().begin()->second;
//    iris::Camera_d& cam2 = (++(cs.cameras().begin()))->second;

    // detect correspondences over all poses
    if( m_finder->useOpenMP() )
    {
        #pragma omp parallel for
        for(int i=0;i<cameraRef.size();i++)
            for( int p=0; p<cameraRef[i].poses.size(); p++ )
                m_finder->find( cameraRef[i].poses[p] );
    }
    else
    {
        for(int i=0;i<cameraRef.size();i++)
            for( int p=0; p<cameraRef[i].poses.size(); p++ )
                m_finder->find( cameraRef[i].poses[p] );
    }


    // filter the poses
    filter( cs );

    // calibrate the cameras
//    stereoCalibrate( m_filteredCameras.begin()->second,
//                 (++(m_filteredCameras.begin()))->second );

    // commit the results
    commit( cs );

}


void OpenCVMultiCalibration::filter( CameraSet_d& cs )
{
    // filter function runs over all poses of all cameras
    // selects the poses and in the order it needs them
    // and stores a copy in m_filteredCameras
    // hihi
    // init stuff
    m_filteredCameras.clear();
    size_t framesAdded = 0;

    // get refs of camera
    std::vector <iris::Camera_d> cameraRef;
    for( auto it = cs.cameras().begin(); it != cs.cameras().end(); it++ )
    {
        cameraRef.push_back(it->second);
    }


    // check that the cameras have the same image size
    if( /*!(m_intrinsicGuess || m_fixIntrinsic) &&*/ checkImageSize(cameraRef) )
       throw std::runtime_error("OpenCVMultiCalibration::filter: cameras do not have the same images size.");

    // do the actual filtering
    for( size_t p=0; p<cameraRef[0].poses.size(); p++ )
    {
        // guilty untill proven innocent
        for(int i=0;i<cameraRef.size();i++)
            cameraRef[i].poses[p].rejected = true;

        // check if this frame is valid
        bool posescheck = true;
        for(int i=1;i<cameraRef.size();i++)
        {
            if(! checkFrame( cameraRef[0].poses[p], cameraRef[i].poses[p] ) )
            {
                posescheck = false;
                std::cout << "OpenCVStereoCalibration::filter: frame rejected." << std::endl;
                break;
            }
        }
        if(posescheck)
        {
            for(int i=0;i<cameraRef.size();i++)
                m_filteredCameras[cameraRef[i].id].poses.push_back( cameraRef[i].poses[p] );
            framesAdded++;
        }
    }

    // set image size
    if( framesAdded > 0 )
    {
        for(int i=0;i<cameraRef.size();i++)
        {
            m_filteredCameras[cameraRef[i].id].imageSize = cameraRef[i].imageSize;
            m_filteredCameras[cameraRef[i].id].id = cameraRef[i].id;
            m_filteredCameras[cameraRef[i].id].intrinsic = cameraRef[i].intrinsic;
        }
    }
}

bool OpenCVMultiCalibration::checkImageSize(const std::vector <iris::Camera_d> &cameraRef)
{
    if(cameraRef.empty())
        return false;

    for(int i=1; i<cameraRef.size();i++)
        if (cameraRef[0].imageSize(0) != cameraRef[i].imageSize(0) || cameraRef[0].imageSize(1) != cameraRef[i].imageSize(1))
            return false;

    return true;
}

bool OpenCVMultiCalibration::checkFrame(const Pose_d & pose1, const Pose_d & pose2)
{
    // check if the arrays have the same length
    if( pose1.points2D.size() == 0 ||
        pose1.pointIndices.size() == 0 ||
        (pose1.points2D.size() != pose2.points2D.size() ) ||
        (pose1.pointIndices.size() != pose2.pointIndices.size() ) )
        return false;

    // check that all the indices match
    for( size_t i=0; i<pose1.pointIndices.size(); i++ )
        if( pose1.pointIndices[i] != pose2.pointIndices[i] )
            return false;

    return true;
}


} // end namespace iris

