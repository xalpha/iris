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

#include <iris/util.hpp>

namespace iris
{

template <size_t N, size_t M, size_t K>
class RandomFeatureDescriptor
{
public:
    struct mCk{ double descriptors[K]; };
    struct nCm{ mCk ggg[M]; };
    struct Cn{  nCm ggg[N]; };

public:
    RandomFeatureDescriptor();
    virtual ~RandomFeatureDescriptor();

    void operator() ( const std::vector<Eigen::Vector2d>& points );

    Pose_d operator& ( const RandomFeatureDescriptor& rfd ) const;

protected:
    std::vector<Eigen::Vector2d> m_points;

};


/////
// Implementation
///

template <size_t N, size_t M, size_t K>
inline RandomFeatureDescriptor<N,M,K>::RandomFeatureDescriptor()
{
}


template <size_t N, size_t M, size_t K>
inline RandomFeatureDescriptor<N,M,K>::~RandomFeatureDescriptor()
{
    // TODO Auto-generated destructor stub
}


template <size_t N, size_t M, size_t K>
inline void RandomFeatureDescriptor<N,M,K>::operator() ( const std::vector<Eigen::Vector2d>& points )
{

}


template <size_t N, size_t M, size_t K>
inline Pose_d RandomFeatureDescriptor<N,M,K>::operator& ( const RandomFeatureDescriptor<N,M,K>& rfd ) const
{

}

} // end namespace iris
