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
 * RandomFeatureDescriptor.hpp
 *
 *  Created on: Jul 26, 2012
 *      Author: duliu
 */

#include <algorithm>

#include <iris/util.hpp>
#include <Eigen/Core>

namespace iris
{

template <size_t M, size_t N, size_t K>
class RandomFeatureDescriptor
{
public:
    struct kGroup
    {
        double rotations[K];
    };

    struct Descriptor
    {
        std::vector<kGroup> vec;
    };

    struct Point
    {
        std::vector<Eigen::Vector2d> neighbors;
        std::vector<Descriptor> descriptors;
    };

public:
    RandomFeatureDescriptor();
    virtual ~RandomFeatureDescriptor();

    void operator() ( const std::vector<Eigen::Vector2d>& points );

    Pose_d operator& ( const RandomFeatureDescriptor& rfd ) const;

protected:
    void describePoint( Point& point );

protected:
    // params
    cvflann::IndexParams m_flannIndexParams;
    cvflann::SearchParams m_flannSearchParams;

    // input data
    std::vector<Eigen::Vector2d> m_points;

    // point descriptors
    std::vector<Point> m_pd;

};


/////
// Implementation
///

template <size_t M, size_t N, size_t K>
inline RandomFeatureDescriptor<M,N,K>::RandomFeatureDescriptor() :
    m_flannSearchParams( 128 )
{

}


template <size_t M, size_t N, size_t K>
inline RandomFeatureDescriptor<M,N,K>::~RandomFeatureDescriptor()
{
    // TODO Auto-generated destructor stub
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::operator() ( const std::vector<Eigen::Vector2d>& points )
{
    // init stuff
    m_points = points;
    Eigen::Vector2d up(0,1);
    m_pd.clear();
    m_pd.reserve( m_points.size() );

    // generate the combinations
    std::vector< std::vector<size_t> > mCn = possible_combinations<M,N>();
    std::vector< std::vector<size_t> > nCk = possible_combinations<N,K>();

    // init flann
    std::vector<cv::Point2d> pointsCV = eigen2cv<double>( m_points ) ;
    //cv::flann::GenericIndex< cv::flann::L2<double> > flann( cv::Mat(pointsCV), m_flannIndexParams);

    // generate the descriptor tree for each point
    for( size_t p=0; p<m_points.size(); p++ )
    {
        // get the M nearest neighbors of point
        std::vector<cv::Point2d> pCV;
        pCV.push_back( pointsCV[p] );
        std::vector<int> nearestM;
        std::vector<cv::Point2d> dists;
        //flann.knnSearch( pCV, nearestM, dists, M, m_flannSearchParams );

        // check if the returned points are at least as M
        if( nearestM.size() < M )
        {
            std::cout << "RandomFeatureDescriptor::operator(): fewer than M nearest... skipping." << std::endl;
            continue;
        }

        // copy points
        std::vector<Eigen::Vector2d> nearestPoints(M);
        for( size_t m=0; m<M; m++ )
            nearestPoints[m] = m_points[nearestM[m]];

        // sort counter clockwise
        counter_clockwise_comparisson<double> ccc( m_points[p], up );
        std::sort( nearestPoints.begin(), nearestPoints.end(), ccc );

        /////
        // WARNING: we are leaving Kansas
        ///

        // initialize the point
        Point point;
        point.neighbors = nearestPoints;

        // compute the descriptor vectors for this point
        describePoint( point );

        // add the point
        m_pd.push_back( point );
    }
}


template <size_t M, size_t N, size_t K>
inline Pose_d RandomFeatureDescriptor<M,N,K>::operator& ( const RandomFeatureDescriptor<M,N,K>& rfd ) const
{
    // TODO
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::describePoint( Point& point )
{

}


} // end namespace iris
