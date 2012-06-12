#pragma once

/*
 * Finder.hpp
 *
 *  Created on: Jun 5, 2012
 *      Author: duliu
 */

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Core>

#define cimg_display 0
#include <CImg.h>


namespace iris
{

//class FinderSettings
//{
//public:
//    FinderSettings();
//    FinderSettings( const FinderSettings& settings );
//    virtual ~FinderSettings();
//    FinderSettings operator = ( const FinderSettings& settings );
//};


class Finder
{
public:
    Finder();
    virtual ~Finder();

    virtual bool find( std::shared_ptr<cimg_library::CImg<uint8_t> > image ) = 0;

    const std::vector< Eigen::Vector2d >& points2D() const;
    const std::vector< Eigen::Vector3d >& points3D() const;

protected:
    bool m_configured;
    std::vector< Eigen::Vector2d > m_points2D;
    std::vector< Eigen::Vector3d > m_points3D;
};

} // end namespace iris
