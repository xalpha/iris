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
    typedef std::vector<double> FeatureVector;

    struct Point
    {
        Eigen::Vector2d pos;
        size_t index;
        std::vector<Eigen::Vector2d> neighbors;
    };


public:
    RandomFeatureDescriptor();
    virtual ~RandomFeatureDescriptor();

    void operator() ( const std::vector<Eigen::Vector2d>& points, bool generateShiftPermutations=false );

    Pose_d operator& ( const RandomFeatureDescriptor& rfd ) const;

protected:
    void describePoint( Point& point, bool generateShiftPermutations );

    double computeDescriptor( const Eigen::Vector2d& c, const std::vector<Eigen::Vector2d>& n ); // center & neighbors

protected:
    // params
    cvflann::IndexParams m_flannIndexParams;
    cvflann::SearchParams m_flannSearchParams;

    // input data
    std::vector<Eigen::Vector2d> m_points;

    // point descriptors
    std::vector<Point> m_pd;
    std::vector<FeatureVector> m_featureVectors;

    // prebacked combinations
    std::vector< std::vector<size_t> > m_mCn;
    std::vector< std::vector<size_t> > m_nCk;
    size_t m_featureVectorsPerPoint;
};


/////
// Implementation
///

template <size_t M, size_t N, size_t K>
inline RandomFeatureDescriptor<M,N,K>::RandomFeatureDescriptor() :
    m_flannSearchParams( 128 )
{
    m_mCn = possible_combinations<M,N>();
    m_nCk = possible_combinations<N,K>();
    m_featureVectorsPerPoint = m_mCn.size() * m_nCk.size();
}


template <size_t M, size_t N, size_t K>
inline RandomFeatureDescriptor<M,N,K>::~RandomFeatureDescriptor()
{
    // TODO Auto-generated destructor stub
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::operator() ( const std::vector<Eigen::Vector2d>& points, bool generateShiftPermutations )
{
    // init stuff
    m_points = points;
    Eigen::Vector2d up(0,1);
    m_pd.clear();
    m_pd.reserve( m_points.size() );
    if( generateShiftPermutations )
        m_featureVectorsPerPoint *= K;
    m_featureVectors.clear();
    m_featureVectors.reserve( m_points.size() * m_featureVectorsPerPoint );

    // init flann
    std::vector<cv::Point2d> pointsCV = eigen2cv<double>( m_points ) ;
    cv::flann::GenericIndex< cv::flann::L2_Simple<double> > pointsFlann( cv::Mat(pointsCV), m_flannIndexParams);

    // generate the descriptor tree for each point
    for( size_t p=0; p<m_points.size(); p++ )
    {
        // get the M nearest neighbors of point
        cv::Mat_<int> nearestM;
        cv::Mat_<float> dists;
        pointsFlann.knnSearch( cv::Mat(pointsCV[p]), nearestM, dists, M, m_flannSearchParams );

        // copy points
        std::vector<Eigen::Vector2d> nearestPoints(M);
        for( size_t m=0; m<M; m++ )
            nearestPoints[m] = m_points[ nearestM.at<int>(m) ];

        // sort counter clockwise
        counter_clockwise_comparisson<double> ccc( m_points[p], up );
        std::sort( nearestPoints.begin(), nearestPoints.end(), ccc );

        // initialize the point
        Point point;
        point.pos = m_points[p];
        point.index = p;
        point.neighbors = nearestPoints;

        // compute the descriptor vectors for this point
        describePoint( point, generateShiftPermutations );

        // add the point
        m_pd.push_back( point );
    }


}


template <size_t M, size_t N, size_t K>
inline Pose_d RandomFeatureDescriptor<M,N,K>::operator& ( const RandomFeatureDescriptor<M,N,K>& rfd ) const
{
    // gen a flan data structure for the current
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::describePoint( Point& point, bool generateShiftPermutations )
{
    // run over all N combinations
    size_t shiftCount = generateShiftPermutations ? K : 1;
    for( size_t n=0; n<m_mCn.size(); n++ )
    {
        /////
        // WARNING: we are now leaving Kansas
        ///

        // reserve enough for all feature vectors
        std::vector<FeatureVector> fvs(shiftCount);
        for( size_t i=0; i<shiftCount; i++ )
            fvs[i].reserve( m_featureVectorsPerPoint / (m_mCn.size() * shiftCount) );

        // run over all combinations of K elements from mCn
        for( size_t k=0; k<m_nCk.size(); k++ )
        {
            // assemble the points
            std::vector<Eigen::Vector2d> points;
            points.reserve( K );
            for( size_t i=0; i<K; i++ )
                points.push_back( point.neighbors[ m_mCn[n][ m_nCk[k][i] ] ] );

            // check if shift permutations are required
            if( generateShiftPermutations )
            {
                // generate shift combinations
                std::vector< std::vector<Eigen::Vector2d> > shift_points = shift_combinations( points );

                // run over all shift combinations and generate descriptors
                for( size_t i=0; i<K; i++ )
                    fvs[i].push_back( computeDescriptor( point.pos, shift_points[i] ) );
            }
            else
                for( size_t i=0; i<K; i++ )
                    fvs[0].push_back( computeDescriptor( point.pos, points ) );
        }

        // add the feature vector(s)
        for( size_t i=0; i<fvs.size(); i++ )
            m_featureVectors.push_back(fvs[i]);
    }
}


template <size_t M, size_t N, size_t K>
inline double RandomFeatureDescriptor<M,N,K>::computeDescriptor( const Eigen::Vector2d& c, const std::vector<Eigen::Vector2d>& n )
{
    // compute the cross ratio of five coplanar points
    // area(A,B,C)*area(A,D,E) / area(A,B,D)*area(A,C,E)

    return ( area( c, n[0], n[1] ) * area( c, n[2], n[3] ) ) /
           ( area( c, n[0], n[2] ) * area( c, n[1], n[3] ) );
}


} // end namespace iris
