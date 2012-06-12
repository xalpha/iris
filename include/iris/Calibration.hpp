#pragma once

/*
 * Calibration.hpp
 *
 *  Created on: Mar 14, 2011
 *      Author: duliu
 */

#include <map>
#include <vector>

#include <Eigen/Core>


namespace iris {

class Calibration
{
public:
    struct Pose
    {
        std::vector<Eigen::Vector2d> points2D;
        std::vector<Eigen::Vector3d> points3D;
        size_t id;
        Eigen::Matrix4d transformation;
        std::vector<Eigen::Vector2d> projected2D;
    };

    struct Camera
    {
        std::vector< Pose > poses;
        Eigen::Vector2i imageSize;
        Eigen::Matrix3d intrinsic;
        std::vector<double> distortion;
        double error;
    };

public:
    Calibration();
    virtual ~Calibration();

    void addPose( const Pose& pose, const Eigen::Vector2i& imageSize, const size_t cameraID=0 );

    virtual void calibrate() = 0;

    const Camera& camera( const size_t id=0 ) const;
    const std::map< size_t, iris::Calibration::Camera >& cameras() const;

protected:
    // cameras
    std::map< size_t, iris::Calibration::Camera > m_cameras;
};

} // end namespace iris
