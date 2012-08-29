////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of IrisCC, a C++ UI for camera calibration               //
//                                                                            //
// Copyright (C) 2012 Alexandru Duliu                                         //
//                                                                            //
// IrisCC is free software; you can redistribute it and/or                    //
// modify it under the terms of the GNU  General Public License               //
// as published by the Free Software Foundation; either version 3             //
// of the License, or (at your option) any later version.                     //
//                                                                            //
// IrisCC is distributed in the hope that it will be useful,                  //
// but WITHOUT ANY WARRANTY; without even the implied warranty of             //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU General Public License          //
// along with IrisCC. If not, see <http://www.gnu.org/licenses/>.             //
//                                                                            //
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <memory>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <QMainWindow>

#include <iris/CameraCalibration.hpp>

#include <nox/plot.hpp>



namespace Ui {
    class IrisCC;

    // Finders
    class ChessboardFinder;
    class RandomFeatureFinder;
#ifdef UCHIYAMA_FOUND
    class UchiyamaFinder;
#endif

    // CameraCalibration
    class OpenCVSingleCalibration;
    class OpenCVStereoCalibration;
}


class IrisCC : public QMainWindow
{
    Q_OBJECT

public:
    explicit IrisCC(QWidget *parent = 0);
    ~IrisCC();

protected:

    void update();

    void updateList();
    void updateErrorPlot();
    void updateImage( int idx );
    void updatePosesPlot();

    void critical( const std::string& message );
    void warning( const std::string& message );

    void clear();

    void check( bool complain=false );

protected slots:
    void on_configureFinder();
    void on_configureCalibration();

    void on_load();
    void on_clear();
    void on_update();
    void on_save();

//    void on_inputChanged( int page );

//    void on_cameraOpen();
//    void on_cameraClose();
//    void on_capture();

    void on_detectedImageChanged( int idx );

protected:
    // ui's
    Ui::IrisCC *ui;
    std::vector< std::shared_ptr<QDialog> > m_finderDialogs;
    std::vector< std::shared_ptr<QDialog> > m_calibrationDialogs;
    Ui::ChessboardFinder* ui_ChessboardFinder;
    Ui::RandomFeatureFinder* ui_RandomFeatureFinder;
#ifdef UCHIYAMA_FOUND
    Ui::UchiyamaFinder* ui_UchiyamaFinder;
#endif
    Ui::OpenCVSingleCalibration* ui_OpenCVSingleCalibration;
    Ui::OpenCVStereoCalibration* ui_OpenCVStereoCalibration;

    // opengl
    nox::plot<double> m_worldPoses;

//    // camera capture
//    cv::VideoCapture m_videoCapture;

    // camera set
    iris::CameraSet_d m_cs;

    // images
    std::vector< size_t > m_poseIndices;
};

