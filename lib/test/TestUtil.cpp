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

#include <assert.h>
#include <iostream>
#include <stdexcept>

#include <iris/util.hpp>





void test_count_bits()
{
    // test zero
    assert( iris::count_bits(0) == 0 );

    // test one bit
    for( uint64_t i=1; i!=0; i*=2 )
        assert( iris::count_bits(i) == 1 );

    // test many bits
    uint64_t number = 0;
    for( uint64_t i=0; i<64; number= number | ( static_cast<uint64_t>(1) << i), i++ )
        assert( iris::count_bits(number) == i );
}


void test_possible_combinations()
{
    // test identity
    for( size_t i=0; i<64; i++ )
        assert( iris::possible_combinations(i,i).size() == 1 );

    // test many posibilities
    for( uint64_t n=1; n<16; n++ )
        for( uint64_t k=1; k<n; k++ )
            assert( iris::possible_combinations(n,k).size() == static_cast<size_t>(iris::n_choose_k(n,k) ) );

    // more in-depth tests
    for( uint64_t n=1; n<16; n++ )
    {
        for( uint64_t k=1; k<n; k++ )
        {
            // test for duplicates
            std::vector< std::vector<size_t> > pc = iris::possible_combinations(n,k);
            for( size_t c=0; c<pc.size(); c++ )
                assert( std::count( pc.begin(), pc.end(), pc[c] ) == 1 );

            // test the range of the values
            for( auto a : pc )
                for( size_t b : a )
                    assert( b>=0 && b<n );
        }
    }
}


template <typename T>
inline void test_shift_combinations()
{
    // test shift with full rotation
    for( T i=1; i<128; i++ )
    {
        // init a simple vector
        std::vector<T> v(i);
        for( T j=0; j<i; j++ )
            v[j] = j;

        // generate shift permutations
        std::vector< std::vector<T> > sv = iris::shift_combinations(v);

        // number of shift combinations should be the same as the vector size
        assert( v.size() == static_cast<T>(i) );

        // check that all shift combinations have the same length
        for( T j=0; j<i; j++ )
            assert( sv.size() == i );

        // check that all combinations are unique
        for( T j=0; j<i; j++ )
        {
            // test that this permutation is unique
            assert( std::count( sv.begin(), sv.end(), sv[j] ) == 1 );

            // test that the values in this permutation are also unique
            for( T k=0; k<i; k++ )
                assert( std::count( sv[j].begin(), sv[j].end(), sv[j][k] ) == 1 );
        }
    }
}


template <typename T>
inline void test_angle( T f )
{
    // init stuff
    T two_pi = 2.0 * (atan(1.0) * 4.0);
    T lastAngle = static_cast<T>(0);
    T delta = f * std::numeric_limits<T>::epsilon();

    // test the function for ridiculus many values
    for( T alpha=std::numeric_limits<T>::epsilon(); alpha<two_pi; alpha+=delta )
    {
        T angle = iris::angle( cos(alpha), sin(alpha) );
        assert( angle >= lastAngle );
        lastAngle = angle;
    }
}


template <typename T>
inline void test_clockwise_comparisson( const std::vector<Eigen::Matrix<T,2,1> >& points, const Eigen::Matrix<T,2,1>& origin )
{
    // make a local copy for random points
    std::vector<Eigen::Matrix<T,2,1> > rp = points;
    for( size_t i=0; i<points.size(); i++ )
    {
        // randomize points
        std::random_shuffle( rp.begin(), rp.end() );

        // sort
        std::vector<Eigen::Matrix<T,2,1> > sp = rp;
        iris::clockwise_comparisson<T> ccc( origin );
        std::sort( sp.begin(), sp.end(), ccc );

        // check if all was well
        assert( sp == points );
    }
}


template <typename T>
inline void test_counter_clockwise_comparisson( const std::vector<Eigen::Matrix<T,2,1> >& points, const Eigen::Matrix<T,2,1>& origin  )
{
    // make a local copy for random points
    std::vector<Eigen::Matrix<T,2,1> > rp = points;
    for( size_t i=0; i<points.size(); i++ )
    {
        // randomize points
        std::random_shuffle( rp.begin(), rp.end() );

        // sort
        std::vector<Eigen::Matrix<T,2,1> > sp = rp;
        iris::counter_clockwise_comparisson<T> ccc( origin );
        std::sort( sp.begin(), sp.end(), ccc );

        // check if all was well
        assert( sp == points );
    }
}


template <typename T>
void test_angular_comparisson()
{
    typedef Eigen::Matrix<T,2,1> Vector2;
    T pi = atan(1.0) * 4.0;

    // test varying number of points
    for( size_t i=1; i<256; i*=2 )
    {
        // assemble the points
        std::vector<Vector2> pointsCW(i);
        std::vector<Vector2> pointsCCW(i);
        T lastAngleCW = 2*pi;
        T lastAngleCCW = 0.0;
        for( size_t p=0; p<i; p++ )
        {
            // assemble some clockwise and random values
            T alpha = 2.0*pi * (static_cast<T>(p) / static_cast<T>(i)) + 0.0001*pi;
            T beta = 2.0*pi - alpha;
            T dist = 0.5*(static_cast<T>(rand()%32767)/32767.0) + 1.0;

            // assemble the point
            pointsCCW[p] = dist * Vector2( cos(alpha), sin(alpha) );
            pointsCW[p] = dist * Vector2( cos(beta), sin(beta) );;

            // test that the angle are correct
            T angleCCW = iris::angle( pointsCCW[p](0), pointsCCW[p](1) );
            T angleCW = iris::angle( pointsCW[p](0), pointsCW[p](1) );
            assert( angleCCW > lastAngleCCW );
            assert( angleCW < lastAngleCW );
            lastAngleCCW = angleCCW;
            lastAngleCW = angleCW;
        }

        // test the sorting at [0,0]
        test_clockwise_comparisson( pointsCW, Vector2(0,0) );
        test_counter_clockwise_comparisson( pointsCCW, Vector2(0,0) );
    }
}


template <typename T>
inline void test_generate_points()
{
    for( size_t i=16; i<=4096; i*=2 )
    {
        // generate points
        T minDist = static_cast<T>( (rand()%10) + 2);
        Eigen::Matrix<T,2,1> minP( 0.0, 0.0 );
        Eigen::Matrix<T,2,1> maxP( 1024.0, 1024.0);
        std::vector<Eigen::Matrix<T,2,1> > points = iris::generate_points( i, minDist, minP, maxP );

        // test the number of points
        assert( points.size() == i );

        // run over the points
        for( size_t p=0; p<i; p++ )
        {
            // test points are greater than min
            assert( points[p](0) >= (minP(0) + minDist) );
            assert( points[p](1) >= (minP(1) + minDist) );

            // test points are lower than max
            assert( points[p](0) <= (maxP(0) + minDist) );
            assert( points[p](1) <= (maxP(1) + minDist) );

            // test that the point is not too close to other points
            for( size_t j=0; j<i; j++ )
                if( j != p )
                    assert( (points[p]-points[j]).norm() >= minDist );
        }
    }
}


int main(int argc, char** argv)
{
    try
    {
        // count_bits
        test_count_bits();

        // test permutations
        test_possible_combinations();

        // test shift permutations
        test_shift_combinations<int>();
        test_shift_combinations<size_t>();
        test_shift_combinations<uint64_t>();

        // test the angle function
        test_angle<float>( 10.0f );
        test_angle<double>( 10000000000.0 );

        // test the counter clockwise comparisson
        test_angular_comparisson<float>();
        test_angular_comparisson<double>();

        // test generate points
        test_generate_points<float>();
        test_generate_points<double>();
    }
    catch( std::exception &e )
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
