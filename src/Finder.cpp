    /*
 * Finder.cpp
 *
 *  Created on: Jun 5, 2011
 *      Author: duliu
 */


#include <iris/Finder.hpp>

namespace iris {


//FinderSettings::FinderSettings(){}
//FinderSettings::FinderSettings( const FinderSettings& settings ){}
//FinderSettings::~FinderSettings(){}
//FinderSettings FinderSettings::operator = ( const FinderSettings& settings ){ return FinderSettings(settings); }


Finder::Finder() :
    m_configured(false)
{
}


Finder::~Finder() {
	// TODO Auto-generated destructor stub
}


const std::vector< Eigen::Vector2d >& Finder::points2D() const
{
    return m_points2D;
}


const std::vector< Eigen::Vector3d >& Finder::points3D() const
{
    return m_points3D;
}


} // end namespace iris

