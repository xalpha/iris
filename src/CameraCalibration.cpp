    /*
 * CameraCalibration.cpp
 *
 *  Created on: Jun 5, 2011
 *      Author: duliu
 */


#include <iris/CameraCalibration.hpp>

namespace iris {

CameraCalibration::CameraCalibration() :
    m_finder(0),
    m_calibration(0)
{
}


CameraCalibration::~CameraCalibration() {
	// TODO Auto-generated destructor stub
}


bool CameraCalibration::addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const size_t poseID, const size_t cameraID )
{
    // check that all is well
    check();

    // find the pattern (whatever it may be)
    bool found = m_finder->find( image );

    // if detection succedeed, add the correspondences to the calibration
    if( found )
    {
        // assemble the pose
        Calibration::Pose pose;
        pose.points2D = m_finder->points2D();
        pose.points3D = m_finder->points3D();
        pose.id = poseID;

        // add correspondences
        m_calibration->addPose( pose, Eigen::Vector2i( image->width(), image->height() ), cameraID );
    }

    // return
    return found;
}


void CameraCalibration::calibrate()
{
    // check that all is well
    check();

    // run the calibration
    m_calibration->calibrate();
}


void CameraCalibration::setFinder( std::shared_ptr<Finder> finder )
{
    m_finder = finder;
}


void CameraCalibration::setCalibration( std::shared_ptr<Calibration> calibration )
{
    m_calibration = calibration;
}


const Finder& CameraCalibration::finder() const
{
    return *m_finder;
}


const Calibration& CameraCalibration::calibration() const
{
    return *m_calibration;
}


void CameraCalibration::check()
{
    if( m_finder == 0 && m_calibration == 0 )
    {
        throw std::runtime_error("CameraCalibration: Finder and Calibration not set.");
    }
    else
    {
        if( m_finder == 0 )
            throw std::runtime_error("CameraCalibration: Finder not set.");
        if( m_calibration == 0 )
            throw std::runtime_error("CameraCalibration: Calibration not set.");
    }
}


} // end namespace iris

