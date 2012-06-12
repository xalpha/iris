////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of iris, a lightweight C++ camera calibration library    //
//                                                                            //
// Copyright (C) 2010, 2011 Alexandru Duliu                                   //
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

#include <cstdint>
#include <memory>

#include <QMainWindow>

#include <iris/CameraCalibration.hpp>



namespace Ui {
    class IrisCC;
    class ChessboardFinder;
}


class IrisCC : public QMainWindow
{
    Q_OBJECT

public:
    explicit IrisCC(QWidget *parent = 0);
    ~IrisCC();

protected:
    void updateCalibration();

    void updateErrorPlot();
    void updateImage( int idx, bool detected );

    void critical( const std::string& message );

    const iris::Calibration::Pose& getPose( size_t idx );

protected slots:
    void on_configureFinder();
    void on_configureCalibration();

    void on_load();
    void on_clear();

    void on_detectedImageChanged( int idx );
    void on_rejectedImageChanged( int idx );

protected:
    // ui's
    Ui::IrisCC *ui;
    Ui::ChessboardFinder* ui_chessboardFinder;

    // actual work gets done here
    iris::CameraCalibration m_cc;

    // finder
    std::shared_ptr<iris::Finder> m_finder;

    // calibration
    std::shared_ptr<iris::Calibration> m_calibration;

    // images
    std::vector< std::shared_ptr< cimg_library::CImg<uint8_t> > > m_images;
    std::vector< QString > m_filenames;
    std::vector< std::shared_ptr< cimg_library::CImg<uint8_t> > > m_detected;
    std::vector< std::shared_ptr< cimg_library::CImg<uint8_t> > > m_rejected;
    std::vector< size_t > m_detected_idx;
    std::vector< size_t > m_rejected_idx;
};
