#pragma once

/*
 * CameraCalibration.hpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */

#include <iris/Calibration.hpp>

namespace iris {

class OpenCVCalibration : public Calibration
{
public:
    OpenCVCalibration();
    virtual ~OpenCVCalibration();

    virtual void calibrate();

 protected:
    void calibrateCamera( Calibration::Camera& cam );

};

} // end namespace iris
