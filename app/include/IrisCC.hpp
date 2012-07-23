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
#include <QDomElement>

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

    void addImage( std::shared_ptr< cimg_library::CImg<uint8_t> > image, const QString& name );

    void critical( const std::string& message );
    void warning( const std::string& message );

    void clear();

    void check( bool complain=false );

    QDomElement addDomElement( QDomDocument &doc,
                               QDomNode &node,
                               const QString &tag,
                               const QString &value = QString::null );

    template <typename T, int Rows, int Cols>
    QString toString( const Eigen::Matrix<T,Rows,Cols>& mat );

    template <typename T>
    QString toString( const std::vector<T>& vec );

    template <typename T, int Rows, int Cols>
    QString toString( const std::vector<Eigen::Matrix<T,Rows,Cols> >& vec );

protected slots:
    void on_configureFinder();
    void on_configureCalibration();

    void on_load();
    void on_clear();
    void on_update();
    void on_save();

    void on_inputChanged( int page );

    void on_cameraOpen();
    void on_cameraClose();
    void on_capture();

    void on_detectedImageChanged( int idx );

protected:
    // ui's
    Ui::IrisCC *ui;

    // camera capture
    cv::VideoCapture m_videoCapture;

    // finder
    std::shared_ptr<iris::Finder> m_finder;

    // calibration
    std::shared_ptr<iris::CameraCalibration> m_calibration;

    // images
    std::vector< size_t > m_poseIndices;
    std::vector< QString > m_poseFilenames;
};



template <typename T, int Rows, int Cols>
inline QString IrisCC::toString( const Eigen::Matrix<T,Rows,Cols>& mat )
{
    QString result;
    for( int i=0; i<Rows*Cols; i++ )
        result.append( QString::number( mat.data()[i] ) + " " );

    return result;
}


template <typename T>
inline QString IrisCC::toString( const std::vector<T>& vec )
{
    QString result;
    for( size_t i=0; i<vec.size(); i++ )
        result.append( QString::number( vec[i] ) + " " );

    return result;
}


template <typename T, int Rows, int Cols>
inline QString IrisCC::toString( const std::vector< Eigen::Matrix<T,Rows,Cols> >& vec )
{
    QString result;
    for( size_t i=0; i<vec.size(); i++ )
    {
        result.append( toString( vec[i] ) + "; " );
    }

    return result;
}

