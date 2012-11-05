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


template <size_t M, size_t N, size_t K>
inline void test_rfd()
{
    // init stuff
    iris::Pose_d pose;
    iris::RandomFeatureDescriptor<M,N,K> pattern(true), image(false);

    // gen random points
    std::vector<Eigen::Vector2d> points = iris::generate_points( 100, 10.0, Eigen::Vector2d(0,0), Eigen::Vector2d(1024.0, 768.0) );

    // do it
    pattern(points);
    image(points);
    pattern.match( image, pose );

    // check the reprojection error
    for( size_t i=0; i<pose.points2D.size(); i++ )
        assert( (pose.points2D[i]-pose.projected2D[i]).norm() < 5.0 );
}


int main(int argc, char** argv)
{
    try
    {
        test_rfd<8,7,5>();
    }
    catch( std::exception &e )
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}







