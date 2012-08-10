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


void CameraCalibration::setFinder( std::shared_ptr<Finder> finder )
{
    m_finder = finder;
}


void CameraCalibration::commit( CameraSet_d &cs )
{
    for( auto camIt=m_filteredCameras.begin(); camIt != m_filteredCameras.end(); camIt++ )
    {
        // get the poses for the target
        std::vector<Pose_d>& srcPoses = camIt->second.poses;
        std::vector<Pose_d>& targetPoses = cs.cameras()[ camIt->second.id ].poses;

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
        cs.cameras()[ camIt->second.id ].intrinsic = camIt->second.intrinsic;
        cs.cameras()[ camIt->second.id ].distortion = camIt->second.distortion;
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

