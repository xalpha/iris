#pragma once

/*
 * ChessboardFinder.hpp
 *
 *  Created on: Jun 5, 2012
 *      Author: duliu
 */


#include <iris/Finder.hpp>

namespace iris
{


class ChessboardFinder : public Finder
{
public:
    ChessboardFinder();
    virtual ~ChessboardFinder();

    void configure( const size_t columns, const size_t rows, const double squareSize );

    virtual bool find( std::shared_ptr<cimg_library::CImg<uint8_t> > image );

protected:
    size_t m_columns;
    size_t m_rows;
    double m_squareSize;

};

} // end namespace iris
