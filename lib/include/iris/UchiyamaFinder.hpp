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
 * UchiyamaFinder.hpp
 *
 *  Created on: Jul 18, 2012
 *      Author: duliu
 *
 * Acknowledgement: the author would like to thank Hideaki Uchiyama for his
 *                  great library to detect random dots:
 *                  H Uchiyama and H Saito: Random Dot Markers [VR 2011]
 *                  ( http://hvrl.ics.keio.ac.jp/uchiyama/me/code/UCHIYAMARKERS )
 *
 *                  and to Yuji Oyamada for his help in integrating random
 *                  dot detection into the iris library
 */




#include <iris/Finder.hpp>

namespace iris
{


class UchiyamaFinder : public Finder
{
public:
    UchiyamaFinder();
    virtual ~UchiyamaFinder();

    void configure( const std::string& patternDescriptor );
    void setAdaptiveExtract( bool val );
    void setMinPoints( size_t minPoints );

    virtual bool find( Pose_d& pose );

protected:
    std::string m_patternDescriptor;
    std::stringstream m_patternDescriptorStream;
    size_t m_minPoints;
    double m_patternWidth;
    double m_patternHeight;
    size_t m_patternPointCount;
    bool m_adaptiveExtract;


};

} // end namespace iris
