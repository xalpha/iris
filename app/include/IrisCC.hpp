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

#include <QDialog>
#include <QMainWindow>

#include <iris/CameraCalibration.hpp>

#include <nox/plot.hpp>



namespace Ui {
    class IrisCC;

    class CameraConfig;
    class CameraInfo;

    // Finders
    class ChessboardFinder;
    class RandomFeatureFinder;

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

    void calibrate();

    void updateImageList();
    void updateErrorPlot();
    void updateImage( int row );
    void updatePosesPlot();
    void updatePosesPlotCurrent();
    void updateCameraList();

    void critical( const std::string& message );
    void warning( const std::string& message );

    void clear();

    size_t getPoseId( int row );
    size_t getCameraId( int comboBoxIdx );

protected slots:
    void on_selectCamera();
    void on_configureCamera();
    void on_cameraInfo();
    void on_acceptConfigureCamera();

    void on_configureFinder();
    void on_configureCalibration();

    void on_load();
    void on_clear();
    void on_erase();
    void on_calibrate();
    void on_save();

//    void on_inputChanged( int page );

//    void on_cameraOpen();
//    void on_cameraClose();
//    void on_capture();

    void on_detectedImageChanged( int idx );

protected:
    // ui's
    Ui::IrisCC *ui;
    QDialog m_cameraConfigDialog;
    QDialog m_cameraInfoDialog;
    std::vector< std::shared_ptr<QDialog> > m_finderDialogs;
    std::vector< std::shared_ptr<QDialog> > m_calibrationDialogs;
    Ui::CameraConfig* ui_CameraConfig;
    Ui::CameraInfo* ui_CameraInfo;
    Ui::ChessboardFinder* ui_ChessboardFinder;
    Ui::RandomFeatureFinder* ui_RandomFeatureFinder;
    Ui::OpenCVSingleCalibration* ui_OpenCVSingleCalibration;
    Ui::OpenCVStereoCalibration* ui_OpenCVStereoCalibration;

    // opengl
    nox::plot<double> m_worldPoses;

    // camera set
    iris::CameraSet_d m_cs;

    // indices
    std::vector< size_t > m_poseIndices;
    std::vector< size_t > m_cameraIndices;
};

