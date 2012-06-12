#pragma once

/*
 * CameraCalibration.hpp
 *
 *  Created on: Jun 5, 2012
 *      Author: duliu
 */

#include <stdexcept>
#include <memory>

#include <iris/Finder.hpp>
#include <iris/Calibration.hpp>


namespace iris
{

class CameraCalibration
{
public:

public:
    CameraCalibration();
    virtual ~CameraCalibration();

    virtual bool addImage( std::shared_ptr<cimg_library::CImg<uint8_t> > image, const size_t poseID, const size_t cameraID=0 );
    virtual void calibrate();

    void setFinder( std::shared_ptr<Finder> finder );
    void setCalibration( std::shared_ptr<Calibration> calibration );

    const Finder& finder() const;
    const Calibration& calibration() const;

protected:
    void check();

protected:
    // these two do the work
    std::shared_ptr<Finder> m_finder;
    std::shared_ptr<Calibration> m_calibration;
};


} // end namespace iris
