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

#include <QMainWindow>

#include <iris/CameraCalibration.hpp>



namespace Ui {
    class IrisCC;
}


class IrisCC : public QMainWindow
{
    Q_OBJECT

public:
    explicit IrisCC(QWidget *parent = 0);
    ~IrisCC();

protected:
    void update();

    void updateErrorPlot();
    void updateImage( int idx );

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

    void on_detectedImageChanged( int idx );

protected:
    // ui's
    Ui::IrisCC *ui;

    // finder
    std::shared_ptr<iris::Finder> m_finder;

    // calibration
    std::shared_ptr<iris::CameraCalibration> m_calibration;

    // images
    std::vector< size_t > m_poseIndices;
    std::vector< QString > m_poseFilenames;
};
