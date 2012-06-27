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

#include <iostream>
#include <stdexcept>
#include <cmath>

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QProgressDialog>
#include <QProgressBar>
#include <QStringList>
#include <QGraphicsPixmapItem>

#include "ui_IrisCC.h"
#include "ui_ChessboardFinder.h"
#include "ui_OpenCVSingleCalibration.h"
#include "ui_OpenCVStereoCalibration.h"

#include <IrisCC.hpp>

#include <iris/ChessboardFinder.hpp>
#include <iris/OpenCVSingleCalibration.hpp>
#include <iris/OpenCVStereoCalibration.hpp>


IrisCC::IrisCC(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::IrisCC)
{
    ui->setupUi(this);

    // connect combo boxes
    connect( ui->select_finder, SIGNAL(currentIndexChanged(int)), this, SLOT(on_configureFinder(void)) );
    connect( ui->select_calibration, SIGNAL(currentIndexChanged(int)), this, SLOT(on_configureCalibration(void)) );
    connect( ui->configure_finder, SIGNAL(clicked(bool)), this, SLOT(on_configureFinder(void)) );
    connect( ui->configure_calibration, SIGNAL(clicked(bool)), this, SLOT(on_configureCalibration(void)) );

    connect( ui->load, SIGNAL(clicked(bool)), this, SLOT(on_load(void)) );
    connect( ui->clear, SIGNAL(clicked(bool)), this, SLOT(on_clear(void)) );
    connect( ui->update, SIGNAL(clicked(bool)), this, SLOT(on_update(void)) );
    connect( ui->image_list_detected, SIGNAL(currentRowChanged(int)), this, SLOT(on_detectedImageChanged(int)) );

    // init image plot
    ui->plot_image->xAxis->setRange(0, 1);
    ui->plot_image->yAxis->setRange(0, 1);

    // init error plot
    ui->plot_error->xAxis->setRange(-1.5, 1.5);
    ui->plot_error->yAxis->setRange(-1.5, 1.5);
    ui->plot_error->xAxis->setAutoTickStep( false );
    ui->plot_error->yAxis->setAutoTickStep( false );
    ui->plot_error->xAxis->setTickStep( 0.5 );
    ui->plot_error->yAxis->setTickStep( 0.5 );

    // fresh start
    clear();
}


IrisCC::~IrisCC()
{
    delete ui;
}


void IrisCC::update()
{
    try
    {
        check();

        // configure CameraCalibration
        m_calibration->setFinder( m_finder );

        // run the calibration
        m_calibration->calibrate();

        // update the error plot
        updateErrorPlot();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::updateErrorPlot()
{
    // clear the plot
    ui->plot_error->clearGraphs();
    ui->plot_error->clearPlottables();
    //ui->plot_error->legend->setVisible(true);

    // run over all camera poses
    for( auto camIt=m_calibration->cameras().begin(); camIt != m_calibration->cameras().end(); camIt++ )
    {
        // run over all poses of the camera
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
        {
            if( !camIt->second.poses[p].rejected )
            {
                // update more stuff
                const iris::Pose_d& pose = camIt->second.poses[p];
                auto graph = ui->plot_error->addGraph();
                QVector<double> x( pose.points2D.size() );
                QVector<double> y( pose.points2D.size() );

                // run over the points
                for( size_t i=0; i<pose.points2D.size(); i++ )
                {
                    x[i] = pose.projected2D[i](0) - pose.points2D[i](0);
                    y[i] = pose.projected2D[i](1) - pose.points2D[i](1);

                    // generate random color
                    QColor col;
                    col.setHslF( static_cast<double>(camIt->first)/static_cast<double>(m_calibration->cameras().size()),
                                 1.0,
                                 0.2 + 0.6*(static_cast<double>(p)/static_cast<double>(camIt->second.poses.size()) ) );

                    // plot the detected points
                    graph->setData(x, y);
                    graph->setPen( col );
                    graph->setLineStyle(QCPGraph::lsNone);
                    graph->setScatterStyle(QCPGraph::ssPlus);
                    graph->setScatterSize(4);
                }
            }
        }
    }

    // redraw
    ui->plot_error->replot();
}


void IrisCC::updateImage( int idx )
{
    try
    {
        // get the image
        const iris::Pose_d pose = m_calibration->pose( m_poseIndices[idx] );
        const cimg_library::CImg<uint8_t>& image = *pose.image;

        // convert image to Qt
        QImage imageQt( image.width(), image.height(), QImage::Format_RGB888 );
        for( int y=0; y<image.height(); y++ )
        {
            for( int x=0; x<image.width(); x++ )
            {
                QColor col( image(x,y,0,0),
                            image(x,y,0,1),
                            image(x,y,0,2) );
                imageQt.setPixel( x, y, col.rgb() );
            }
        }

        // init the plot
        ui->plot_image->clearGraphs();
        ui->plot_image->clearPlottables();

        // set the images
        ui->plot_image->setAxisBackground(QPixmap::fromImage(imageQt));
        ui->plot_image->setAxisBackgroundScaledMode( Qt::IgnoreAspectRatio );
        ui->plot_image->xAxis->setRange(0, imageQt.width() );
        ui->plot_image->yAxis->setRange(0, imageQt.height() );

        // draw the detected points
        if( !pose.rejected )
        {
            // get the pose
            const iris::Pose_d& pose = m_calibration->pose( m_poseIndices[idx] );

            // copy the data
            QVector<double> x( pose.points2D.size() );
            QVector<double> y( pose.points2D.size() );
            QVector<double> px( pose.points2D.size() );
            QVector<double> py( pose.points2D.size() );
            double height = static_cast<double>(imageQt.height());
            for( size_t i=0; i<pose.points2D.size(); i++ )
            {
                x[i] = pose.points2D[i](0);
                y[i] = height - pose.points2D[i](1);

                px[i] = pose.projected2D[i](0);
                py[i] = height - pose.projected2D[i](1);
            }

            // plot the detected points
            ui->plot_image->addGraph();
            ui->plot_image->graph(0)->setData(x, y);
            ui->plot_image->graph(0)->setPen( QColor( Qt::green ) );
            ui->plot_image->graph(0)->setLineStyle(QCPGraph::lsNone);
            ui->plot_image->graph(0)->setScatterStyle(QCPGraph::ssPlus);
            ui->plot_image->graph(0)->setScatterSize(4);

            // plot the detected points
            ui->plot_image->addGraph();
            ui->plot_image->graph(1)->setData(px, py);
            ui->plot_image->graph(1)->setPen( QColor( Qt::yellow ) );
            ui->plot_image->graph(1)->setLineStyle(QCPGraph::lsNone);
            ui->plot_image->graph(1)->setScatterStyle(QCPGraph::ssPlus);
            ui->plot_image->graph(1)->setScatterSize(4);
        }

        // redraw
        ui->plot_image->replot();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::critical( const std::string& message )
{
    ui->statusBar->showMessage( QString( message.c_str() ), 5000 );
    std::cerr << message << std::endl;
    QMessageBox::critical(this, "Error", QString( message.c_str() ) );
}


void IrisCC::warning( const std::string& message )
{
    ui->statusBar->showMessage( QString( message.c_str() ), 5000 );
    std::cout << message << std::endl;
}


void IrisCC::clear()
{
    // cleanup the image plot
    ui->plot_image->clearGraphs();
    ui->plot_image->clearPlottables();

    // cleanup the error plot
    ui->plot_error->clearPlottables();
    ui->plot_error->clearGraphs();

    // cleanup the lists
    ui->image_list_detected->clear();

    // clear images
    m_poseFilenames.clear();
    m_poseIndices.clear();
}


void IrisCC::check( bool complain )
{
    ui->update->setEnabled(false);

    try
    {
        // arer there any images
        if( m_calibration && m_calibration->poseCount() == 0 )
            throw std::runtime_error("IrisCC: no poses.");

        // check if there is any calibration
        if( !m_calibration )
            throw std::runtime_error("IrisCC: no Calibration selected.");

        // is there a finder
        if( !m_finder )
            throw std::runtime_error("IrisCC: no Finder selected.");
    }
    catch( std::exception& e )
    {
        if( complain )
            critical( e.what() );
        else
            warning( e.what() );

        return;
    }

    ui->update->setEnabled(true);
}


void IrisCC::on_configureFinder()
{
    try
    {
        // init stuff
        QDialog dialog(this);
        ui->configure_finder->setEnabled(false);

        // choose what to do
        switch( ui->select_finder->currentIndex() )
        {
            // nothing selected
            case 0 :
                break;

            // chessboard
            case 1 :
            {
                ui->configure_finder->setEnabled(true);
                Ui::ChessboardFinder chessboardFinderUI;
                chessboardFinderUI.setupUi( &dialog );
                dialog.exec();
                iris::ChessboardFinder* finder = new iris::ChessboardFinder();
                finder->configure( static_cast<size_t>( chessboardFinderUI.columns->value() ),
                                   static_cast<size_t>( chessboardFinderUI.rows->value() ),
                                   chessboardFinderUI.square_size->value() );
                m_finder = std::shared_ptr<iris::Finder>(finder);
                break;
            }

            // not supported finder
            default:
                throw std::runtime_error("IrisCC::on_configure_finder: Finder not supported.");
        }

        check();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::on_configureCalibration()
{
    try
    {
        // init stuff
        QDialog dialog(this);
        ui->configure_calibration->setEnabled(false);

        // choose what to do
        switch( ui->select_calibration->currentIndex() )
        {
            // nothing selected
            case 0 :
                break;

            // OpenCV
            case 1 :
            {
                ui->configure_calibration->setEnabled(true);
                Ui::OpenCVSingleCalibration form;
                form.setupUi( &dialog );
                dialog.exec();
                iris::OpenCVSingleCalibration* calib = new iris::OpenCVSingleCalibration();
                calib->configure( form.fixed_principal_point->isChecked(),
                                  form.fixed_aspect_ratio->isChecked(),
                                  form.tangential_distortion->isChecked() );
                calib->copyCameras( m_calibration );
                m_calibration = std::shared_ptr<iris::CameraCalibration>( calib );
                break;
            }

            // OpenCV Stereo
            case 2 :
            {
                ui->configure_calibration->setEnabled(true);
                Ui::OpenCVStereoCalibration form;
                form.setupUi( &dialog );
                dialog.exec();
                iris::OpenCVStereoCalibration* calib = new iris::OpenCVStereoCalibration();
                calib->configure( form.relative_to_pattern->isChecked(),
                                  form.fixed_principal_point->isChecked(),
                                  form.fixed_aspect_ratio->isChecked(),
                                  form.same_focal_length->isChecked(),
                                  form.tangential_distortion->isChecked() );
                calib->copyCameras( m_calibration );
                m_calibration = std::shared_ptr<iris::CameraCalibration>( calib );
                break;
            }

            // not supported finder
            default:
                throw std::runtime_error("IrisCC::on_configureCalibration: Calibration not supported.");
        }

        check();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::on_load()
{
    try
    {
        QStringList imagePaths = QFileDialog::getOpenFileNames(this, "Load Images", ".", "Images (*.bmp *.png *.xpm *.jpg *.tif)");

        // return fi nothing there
        if( imagePaths.size() == 0 )
            return;

        // init progress dialog
        QProgressDialog progress("Loading Images...", " ", 0, imagePaths.size(), this);
        progress.setWindowModality(Qt::WindowModal);
        progress.setCancelButton(0);
        progress.show();

        // load the images
        for( int i=0; i<imagePaths.size(); i++ )
        {
            // load the current image to RGB888
            QImage imageQt;
            imageQt.load( imagePaths[i] );
            imageQt = imageQt.convertToFormat( QImage::Format_RGB32 ).copy();

            // convert to CImg
            std::shared_ptr< cimg_library::CImg<uint8_t> > image( new cimg_library::CImg<uint8_t>( imageQt.width(), imageQt.height(), 1, 3 ) );
            for( int y=0; y<imageQt.height(); y++ )
            {
                for( int x=0; x<imageQt.width(); x++ )
                {
                    QColor col( imageQt.pixel(x,y) );
                    (*image)(x,y,0,0) = col.red();
                    (*image)(x,y,0,1) = col.green();
                    (*image)(x,y,0,2) = col.blue();
                }
            }

            // add image
            size_t id = m_calibration->addImage( image, static_cast<size_t>( ui->cameraID->value() ) );
            m_poseIndices.push_back( id );
            m_poseFilenames.push_back( QFileInfo( imagePaths[i] ).fileName() );

            // update list
            ui->image_list_detected->addItem( QFileInfo( imagePaths[i] ).fileName() );

            // update progress
            progress.setValue(i);
        }

        // tidy up progress bar
        progress.setValue(imagePaths.size());

        check();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::on_clear()
{
    QMessageBox::StandardButton response = QMessageBox::warning( this,
                                                                 QString(),
                                                                 "Clear all images?",
                                                                 QMessageBox::Yes | QMessageBox::No );
    if( QMessageBox::Yes == response )
        clear();

    check();
}


void IrisCC::on_update()
{
    update();
}


void IrisCC::on_detectedImageChanged( int idx )
{
    updateImage( idx );
}
