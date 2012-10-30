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

#include <iris/RandomFeatureDescriptor.hpp>

void test_count_bits()
{
    iris::RandomFeatureDescriptor<1,1,1> rfd;

    // test zero
    assert( rfd.count_bits(0) == 0 );

    // test one bit
    for( uint64_t i=1; i!=0; i*=2 )
        assert( rfd.count_bits(i) == 1 );

    // test many bits
    uint64_t number = 0;
    for( uint64_t i=0; i<64; number= number | ( static_cast<uint64_t>(1) << i), i++ )
        assert( rfd.count_bits(number) == i );
}


void test_possible_combinations()
{
    iris::RandomFeatureDescriptor<1,1,1> rfd;

    // test identity
    for( size_t i=0; i<64; i++ )
        assert( rfd.possible_combinations(i,i).size() == 1 );

    // test many posibilities
    for( uint64_t n=1; n<16; n++ )
        for( uint64_t k=1; k<n; k++ )
            assert( rfd.possible_combinations(n,k).size() == static_cast<size_t>(iris::n_choose_k(n,k) ) );

    // test for duplicates
    for( uint64_t n=1; n<16; n++ )
    {
        for( uint64_t k=1; k<n; k++ )
        {
            std::vector< std::vector<size_t> > pc = rfd.possible_combinations(n,k);
            for( size_t c=0; c<pc.size(); c++ )
                assert( std::count( pc.begin(), pc.end(), pc[c] ) == 1 );
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
    }
    catch( std::exception &e )
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
