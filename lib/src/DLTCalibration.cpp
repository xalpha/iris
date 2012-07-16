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
 * DLTCalibration.cpp
 *
 *  Created on: Jul 2, 2012
 *      Author: duliu
 */

#ifdef IRIS_OPENMP
#include <omp.h>
#endif


#include <iris/DLTCalibration.hpp>


namespace iris {

DLTCalibration::DLTCalibration() : CameraCalibration()
{
}


DLTCalibration::~DLTCalibration() {
	// TODO Auto-generated destructor stub
}


void DLTCalibration::calibrate()
{
    // check that all is OK
    check();

    // run over all cameras and find correspondences
    for( auto it = m_cameras.begin(); it != m_cameras.end(); it++ )
    {
        // update stuff
        std::vector< Pose_d >& poses = it->second.poses;
        size_t poseCount = poses.size();

        // run feature detection
#       pragma omp parallel for
        for( int p=0; p<poseCount; p++ )
            m_finder->find( poses[p] );
    }

    // filter the poses
    filter();

    // run over all cameras and calibrate the poses
    for( auto it = m_cameras.begin(); it != m_cameras.end(); it++ )
    {
        // update stuff
        std::vector< Pose_d >& poses = it->second.poses;
        size_t poseCount = poses.size();

        // run feature detection
#       pragma omp parallel for
        for( int p=0; p<poseCount; p++ )
            calibratePose( poses[p] );
    }

    // commit the calibration calibrated frames
    commit();
}


void DLTCalibration::calibratePose( Pose_d &pose )
{

}


void DLTCalibration::filter()
{
    // init stuff
    m_filteredCameras.clear();

    // run over all the poses and only
    for( auto it = m_cameras.begin(); it != m_cameras.end(); it++ )
    {
        for( size_t p=0; p<it->second.poses.size(); p++ )
        {
            // guilty untill proven innocent
            it->second.poses[p].rejected = true;

            // check that all is well
            if( ( it->second.poses[p].points2D.size() > 0 ) &&
                ( it->second.poses[p].points2D.size() == it->second.poses[p].pointIndices.size() ) )
                m_filteredCameras[it->second.id].poses.push_back( it->second.poses[p] );
        }

        // set image size
        m_filteredCameras[it->second.id].id = it->second.id;
        m_filteredCameras[it->second.id].imageSize = it->second.imageSize;
    }
}


} // end namespace iris

