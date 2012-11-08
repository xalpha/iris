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
        Point()
        {
            neighbors.resize(M);
        }

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

    void match( const RandomFeatureDescriptor& rfd, Pose_d& pose ) const;

    void operator =( const RandomFeatureDescriptor& pose );

protected:

    void describePoint( Point& point );

    double descriptor( const std::vector<Eigen::Vector2d>& points );

    void fv2cv( const std::vector<Point>& points, cv::Mat_<float>& fvs, std::vector<size_t>& indices ) const;

protected:
    // point descriptors
    std::vector<Point> m_points;

    // prebacked combinations
    std::vector< std::vector<size_t> > m_mCn;
    std::vector< std::vector<size_t> > m_nCk;
    std::vector< std::vector<size_t> > m_kS;

    // prebacked combinations
    size_t m_featureVectorsPerPoint;
    size_t m_featureVectorLenth;

    // flann
    cv::Mat_<float> m_flannMatrix;
    std::vector<size_t> m_flannIndices;
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
    m_points.resize( points.size() );
    cv::Mat_<double> pointsCV( static_cast<int>(points.size()), 2 );

    // get the position of the points and init stuff
    for( size_t p=0; p<points.size(); p++ )
    {
        m_points[p].pos = points[p];
        pointsCV( p, 0 ) = points[p](0);
        pointsCV( p, 1 ) = points[p](1);
    }

    // build kd-tree
    cv::flann::GenericIndex< cv::flann::L2_Simple<double> > pointsFlann( pointsCV, cvflann::KDTreeIndexParams(5) );

    // compute multiple feature vectors for all points
    for( size_t p=0; p<m_points.size(); p++ )
    {
        // get the M nearest neighbors of point
        pointsFlann.knnSearch( pointsCV.row(p).clone(), nearestM, distsM, M+1, cvflann::SearchParams(128) );

        // copy points
        for( size_t m=0; m<M; m++ )
            m_points[p].neighbors[m] = m_points[ nearestM(m+1) ].pos;

        // sort counter clockwise
        clockwise_comparisson<double> ccc( m_points[p].pos );
        std::sort( m_points[p].neighbors.begin(), m_points[p].neighbors.end(), ccc );

        // compute the feature vectors for this point
        describePoint( m_points[p] );
    }

    // convert feature vectors to matrices for opencv
    fv2cv( m_points, m_flannMatrix, m_flannIndices );
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::match( const RandomFeatureDescriptor<M,N,K>& rfd, Pose_d& pose ) const
{
    // match the feature vectors
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( m_flannMatrix, rfd.m_flannMatrix, matches );

    // mark all the matches
    std::vector<Eigen::Vector2d> matchQueryPoints, matchTrainPoints;
    for( size_t i=0; i<matches.size(); i++ )
    {
        const cv::DMatch& m = matches[i];
        matchQueryPoints.push_back( m_points[m_flannIndices[m.queryIdx]].pos );
        matchTrainPoints.push_back( rfd.m_points[rfd.m_flannIndices[m.trainIdx]].pos );
    }

    // run RANSAC and compute reprojection error
    Eigen::Matrix3d H;
    std::vector<cv::Point2f> matchQueryPointsCV = iris::eigen2cv<float>(matchQueryPoints);
    std::vector<cv::Point2f> matchTrainPointsCV = iris::eigen2cv<float>(matchTrainPoints);
    cv::Mat_<float> HCV = cv::findHomography( matchQueryPointsCV, matchTrainPointsCV, CV_RANSAC, 5.0 );
    cv::cv2eigen( HCV, H );

    // compute matrix of reprojection errors
    Eigen::MatrixXf reprojMatrix = Eigen::MatrixXf::Constant( m_points.size(), rfd.m_points.size(), std::numeric_limits<float>::max() );
    for( size_t i=0; i<matches.size(); i++ )
    {
		// project point and compute the reprojection error
        const cv::DMatch& m = matches[i];
        Eigen::Vector2d pp = iris::project_point<double,2>( H, m_points[m_flannIndices[m.queryIdx]].pos );
        float err = static_cast<float>( (pp-rfd.m_points[rfd.m_flannIndices[m.trainIdx]].pos).norm() );
        reprojMatrix( m_flannIndices[m.queryIdx], rfd.m_flannIndices[m.trainIdx] ) = std::min( reprojMatrix( m_flannIndices[m.queryIdx], rfd.m_flannIndices[m.trainIdx] ), err );
    }

    //  select the best matches
    std::vector<bool> trainMask(rfd.m_points.size(), true);
    std::vector<Eigen::Vector2d> queryPoints, trainPoints;
    std::vector<size_t> queryIndices;
    for( size_t q=0; q<m_points.size(); q++ )
    {
        // get the best free point
        float minErr = std::numeric_limits<float>::max();
        bool found = false;
        size_t idx;
        for( size_t t=0; t<rfd.m_points.size(); t++ )
            if( trainMask[t] && reprojMatrix(q,t) < minErr && reprojMatrix(q,t) < 15.0 )
            {
                minErr = reprojMatrix(q,t);
                found = true;
                idx = t;
            }

        // if successfull process
        if( found )
        {
            queryPoints.push_back( m_points[q].pos );
            queryIndices.push_back( q );
            trainPoints.push_back(rfd.m_points[idx].pos);
            trainMask[idx] = false;
        }
    }

    // project these points with the homography
    std::vector<Eigen::Vector2d> projected2D = iris::project_points<double,2>( H, queryPoints );

    pose.points2D = trainPoints;
    pose.pointIndices = queryIndices;
    pose.projected2D = projected2D;




//    // write results
//    pose.points2D.clear();
//    pose.pointIndices.clear();
//    pose.projected2D.clear();
//    for( size_t i=0; i<queryPoints.size(); i++ )
//        if( (trainPoints[i]-projected2D[i]).norm() < 15.0 )
//        {
//            pose.points2D.push_back( trainPoints[i] );
//            pose.pointIndices.push_back( queryIndices[i] );
//            pose.projected2D.push_back( projected2D[i] );
//        }
}


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::operator =( const RandomFeatureDescriptor<M,N,K>& rfd )
{
    m_points = rfd.m_points;
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
                {
                    size_t a = m_kS[s][p];
                    size_t b = m_nCk[k][a];
                    size_t c = m_mCn[n][b];
                    points[p] = point.neighbors[c];
                    //points[p] = point.neighbors[ m_mCn[n][ m_nCk[k][ m_kS[s][p] ] ] ]; // rest in peace cache ;)
                }

                // write the descriptor
                featureVectors[n*m_kS.size() + s][k] = descriptor( points );
            }
        }
    }

    // sanity check all feature vectors
    point.featureVectors.clear();
    for( size_t i=0; i<featureVectors.size(); i++ )
        if( featureVectors[i].sum() > 0.0 )
            point.featureVectors.push_back( featureVectors[i] );
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


template <size_t M, size_t N, size_t K>
inline void RandomFeatureDescriptor<M,N,K>::fv2cv( const std::vector<Point>& points, cv::Mat_<float>& fvs, std::vector<size_t>& indices ) const
{
    // count vectors
    size_t fvCount = 0;
    for( size_t i=0; i<points.size(); i++ )
        fvCount += points[i].featureVectors.size();

    // assemble matrix
    size_t row = 0;
    fvs = cv::Mat_<float>( fvCount, m_featureVectorLenth );
    indices.resize( fvCount );
    for( size_t p=0; p<points.size(); p++ )
        for( auto it=points[p].featureVectors.begin(); it!=points[p].featureVectors.end(); it++ )
        {
			const FeatureVector& fv = *it;
            for( size_t j=0; j<m_featureVectorLenth; j++ )
                fvs( row, j ) = static_cast<float>(fv[j]);
            indices[row] = p;
            row++;
        }
}



} // end namespace iris
