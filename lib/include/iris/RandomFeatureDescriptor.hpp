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
#include <limits>



#include <iris/util.hpp>
#include <Eigen/Core>

namespace iris
{

template <size_t M, size_t N, size_t K>
class RandomFeatureDescriptor
{
public:
    typedef Eigen::VectorXd FeatureVector;

    class Point
    {
    public:
        Point() : neighbors(M) {}

        Point( const Point& point )
        {
            *this = point;
        }

        void operator =( const Point& point )
        {
            pos = point.pos;
            neighbors = point.neighbors;
            featureVectors = point.featureVectors;
        }

        Eigen::Vector2d pos;
        std::vector<Eigen::Vector2d> neighbors;
        std::list<FeatureVector> featureVectors;
    };


public:
    RandomFeatureDescriptor( bool generateShiftPermutations=false );
    virtual ~RandomFeatureDescriptor();

    void operator() ( const std::vector<Eigen::Vector2d>& points );

    Pose_d operator& ( const RandomFeatureDescriptor& rfd ) const;

    void operator =( const RandomFeatureDescriptor& pose );

    void describePoint( Point& point );

    double descriptor( const std::vector<Eigen::Vector2d>& points );

protected:
    // point descriptors
    std::vector<Point> m_pd;

    // prebacked combinations
    std::vector< std::vector<size_t> > m_mCn;
    std::vector< std::vector<size_t> > m_nCk;
    std::vector< std::vector<size_t> > m_kS;

    // prebacked combinations
    size_t m_featureVectorsPerPoint;
    size_t m_featureVectorLenth;
};


/////
// Implementation
///

template <size_t M, size_t N, size_t K>
inline RandomFeatureDescriptor<M,N,K>::RandomFeatureDescriptor( bool generateShiftPermutations )
{
    // generate combinations
    m_mCn = possible_combinations(M,N);
    m_nCk = possible_combinations(N,K);
    m_featureVectorsPerPoint = iris::n_choose_k(M,N);
    m_featureVectorLenth = iris::n_choose_k(N,K);

    // generate shift permutatins
    std::vector<size_t> shift(K);
    for( size_t i=0; i<K; i++ )
        shift[i] = i;
    if( generateShiftPermutations )
    {
        m_kS = iris::shift_combinations( shift );
        m_featureVectorsPerPoint *= m_kS.size();
    }
    else
        m_kS.push_back( shift );
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
    cv::Mat_<int> nearestM( 1, M+1 );
    cv::Mat_<double> distsM( 1, M+1 );
    m_pd.resize( points.size() );
    cv::Mat_<double> pointsCV( static_cast<int>(points.size()), 2 );

    // get the position of the points and init stuff
    for( size_t p=0; p<points.size(); p++ )
    {
        m_pd[p].pos = points[p];
        pointsCV( p, 0 ) = points[p](0);
        pointsCV( p, 1 ) = points[p](1);
    }

    // build kd-tree
    cv::flann::GenericIndex< cv::flann::L2_Simple<double> > pointsFlann( pointsCV, cvflann::KDTreeIndexParams(5) );

    // generate the descriptor tree for each point
    for( size_t p=0; p<m_pd.size(); p++ )
    {
        // get the M nearest neighbors of point
        pointsFlann.knnSearch( pointsCV.row(p).clone(), nearestM, distsM, M+1, cvflann::SearchParams(128) );

        // copy points
        for( size_t m=0; m<M; m++ )
            m_pd[p].neighbors[m] = m_pd[ nearestM(m+1) ].pos;

        // sort counter clockwise
        clockwise_comparisson<double> ccc( m_pd[p].pos );
        std::sort( m_pd[p].neighbors.begin(), m_pd[p].neighbors.end(), ccc );

        // compute the feature vectors for this point
        describePoint( m_pd[p] );
    }
}


template <size_t M, size_t N, size_t K>
inline Pose_d RandomFeatureDescriptor<M,N,K>::operator& ( const RandomFeatureDescriptor<M,N,K>& rfd ) const
{
    // addemble the descriptors
//    cv::Mat_<float> queryFVs = vector2cv<float>(m_featureVectors);
//    cv::Mat_<float> trainFVs = vector2cv<float>(rfd.m_featureVectors);

//    cv::flann::GenericIndex< cv::flann::L2<double> > flann( trainFVs, cvflann::KDTreeIndexParams(5) );

//    // get the M nearest neighbors of point
//    cv::Mat_<int> nearestM( int(m_featureVectors.size()), 1 );
//    cv::Mat_<double> distsM( int(m_featureVectors.size()), 1 );
//    flann.knnSearch( queryFVs, nearestM, distsM, 1, cvflann::SearchParams(128) );

//    std::cout << std::setprecision(2) << queryFVs << std::endl << std::endl << std::endl;
//    std::cout << std::setprecision(2) << trainFVs << std::endl << std::endl << std::endl;

    // match the feature vectors
//    cv::FlannBasedMatcher matcher;
//    //cv::BruteForceMatcher<cv::L2<float> > matcher;
//    std::vector< cv::DMatch > matches;
//    matcher.match( queryFVs, trainFVs, matches );

//    double max_dist = 0; double min_dist = 100;

//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < queryFVs.rows; i++ )
//    { double dist = matches[i].distance;
//      if( dist > 0 && dist < min_dist ) min_dist = dist;
//      if( dist > max_dist ) max_dist = dist;
//    }

//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );

//    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
//    //-- PS.- radiusMatch can also be used here.
//    std::vector< cv::DMatch > good_matches;

//    for( int i = 0; i < queryFVs.rows; i++ )
//    { if( matches[i].distance > 0 && matches[i].distance <= 2*min_dist )
//      { good_matches.push_back( matches[i]); }
//    }

//    if( good_matches.size() == 0 )
//        std::cout << "RandomFeatureDescriptor: NO good Matches." << std::endl;

//    for( int i = 0; i < good_matches.size(); i++ )
//    { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

//    // init vector of best matches
//    std::vector< cv::DMatch > minMatches( m_points.size() );

//    // run over the matches and choose the best one
//    int fvspp = static_cast<int>(m_featureVectorsPerPoint);
//    for( size_t i=0; i<matches.size(); i++ )
//        if( matches[i].distance < minMatches[ matches[i].queryIdx / fvspp ].distance )
//            minMatches[ matches[i].queryIdx / fvspp ] = matches[i];

//    // extract the distances
//    std::vector<float> dists;
//    for( size_t i=0; i<minMatches.size(); i++ )
//        dists.push_back( minMatches[i].distance );

//    // analyse a bit
//    float m = mean( dists );

//    // assemble the pose
//    Pose_d pose;
//    for( size_t i=0; i<minMatches.size(); i++ )
//    {
//        if( minMatches[i].distance < m )
//        {
//            pose.pointIndices.push_back( static_cast<size_t>(minMatches[i].queryIdx / fvspp) );
//            pose.points2D.push_back( rfd.m_points[ minMatches[i].trainIdx / static_cast<int>(rfd.m_featureVectorsPerPoint) ] );
//        }
//    }

//    return pose;
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::operator =( const RandomFeatureDescriptor<M,N,K>& rfd )
{
    m_pd = rfd.m_pd;
    m_mCn = rfd.m_mCn;
    m_nCk = rfd.m_nCk;
    m_kS = rfd.m_kS;
    m_featureVectorsPerPoint = rfd.m_featureVectorsPerPoint;
    m_featureVectorLenth = rfd.m_featureVectorLenth;
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::describePoint( Point& point )
{
    /////
    // WARNING: we are now leaving Kansas
    ///

    // run over all N combinations
    std::vector<FeatureVector> featureVectors( m_featureVectorsPerPoint, FeatureVector::Zero(m_featureVectorLenth) );
    for( size_t n=0; n<m_mCn.size(); n++ )
    {
        // run over all combinations of K elements from mCn
        for( size_t k=0; k<m_nCk.size(); k++ )
        {
            // run over all shift permutations
            for( size_t s=0; s<m_kS.size(); s++ )
            {
                // assemble the points needed for the descriptor
                std::vector<Eigen::Vector2d> points(K);
                for( size_t p=0; p<K; p++ )
                    points[p] = point.neighbors[ m_mCn[n][ m_nCk[k][ m_kS[s][p] ] ] ]; // cache rest in peace ;)

                // write the descriptor
                featureVectors[n*m_kS.size() + s][k] = descriptor( points );
            }
        }
    }

    // sanity check all feature vectors
    point.featureVectors.clear();
    for( FeatureVector& fv : featureVectors )
        if( fv.sum() > 0.0 )
            point.featureVectors.push_back( fv );
}


template <size_t M, size_t N, size_t K>
inline double RandomFeatureDescriptor<M,N,K>::descriptor( const std::vector<Eigen::Vector2d>& points )
{
    if( K == 5 )
        return crossRatio( points[0], points[1], points[2], points[3], points[4] );
    else if( K == 4 )
        return affineInvariant( points[0], points[1], points[2], points[3] );
    else
        throw std::runtime_error( "RandomFeatureDescriptor::descriptor: unknown invariant with \"" + iris::toString(K) + "\" points." );
}



} // end namespace iris
