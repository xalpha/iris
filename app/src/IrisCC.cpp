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
#include "ui_RandomFeatureFinder.h"
#include "ui_OpenCVSingleCalibration.h"
#include "ui_OpenCVStereoCalibration.h"

#include <IrisCC.hpp>

#include <iris/ChessboardFinder.hpp>
#ifdef UCHIYAMA_FOUND
#include <iris/UchiyamaFinder.hpp>
#include "ui_UchiyamaFinder.h"
#endif
#include <iris/RandomFeatureFinder.hpp>

#include <iris/OpenCVSingleCalibration.hpp>
#include <iris/OpenCVStereoCalibration.hpp>


IrisCC::IrisCC(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::IrisCC),
    ui_ChessboardFinder( new Ui::ChessboardFinder ),
#ifdef UCHIYAMA_FOUND
    ui_UchiyamaFinder( new Ui::UchiyamaFinder ),
#endif
    ui_RandomFeatureFinder( new Ui::RandomFeatureFinder ),
    ui_OpenCVSingleCalibration( new Ui::OpenCVSingleCalibration ),
    ui_OpenCVStereoCalibration( new Ui::OpenCVStereoCalibration )
{
    ui->setupUi(this);

    // init finder and calibration
    on_configureFinder();
    on_configureCalibration();

    // connect combo boxes
    connect( ui->input, SIGNAL(currentChanged(int)), this, SLOT(on_inputChanged(int)) );
    connect( ui->capture, SIGNAL(clicked(bool)), this, SLOT(on_capture(void)) );

    connect( ui->select_finder, SIGNAL(currentIndexChanged(int)), this, SLOT(on_configureFinder(void)) );
    connect( ui->select_calibration, SIGNAL(currentIndexChanged(int)), this, SLOT(on_configureCalibration(void)) );
    connect( ui->configure_finder, SIGNAL(clicked(bool)), this, SLOT(on_configureFinder(void)) );
    connect( ui->configure_calibration, SIGNAL(clicked(bool)), this, SLOT(on_configureCalibration(void)) );

    connect( ui->load, SIGNAL(clicked(bool)), this, SLOT(on_load(void)) );
    connect( ui->clear, SIGNAL(clicked(bool)), this, SLOT(on_clear(void)) );
    connect( ui->image_list_detected, SIGNAL(currentRowChanged(int)), this, SLOT(on_detectedImageChanged(int)) );
    connect( ui->update, SIGNAL(clicked(bool)), this, SLOT(on_update(void)) );
    connect( ui->save, SIGNAL(clicked(bool)), this, SLOT(on_save(void)) );

    // init finder dialogs
    m_finderDialogs.push_back( std::shared_ptr<QDialog>() );
    m_finderDialogs.push_back( std::shared_ptr<QDialog>( new QDialog(this) ) );    ui_ChessboardFinder->setupUi( m_finderDialogs.back().get() );
#ifdef UCHIYAMA_FOUND
    m_finderDialogs.push_back( std::shared_ptr<QDialog>( new QDialog(this) ) );    ui_UchiyamaFinder->setupUi( m_finderDialogs.back().get() );
#endif
    m_finderDialogs.push_back( std::shared_ptr<QDialog>( new QDialog(this) ) );    ui_RandomFeatureFinder->setupUi( m_finderDialogs.back().get() );

    // init calibration dialogs
    m_calibrationDialogs.push_back( std::shared_ptr<QDialog>( new QDialog(this) ) );
    m_calibrationDialogs.push_back( std::shared_ptr<QDialog>( new QDialog(this) ) );   ui_OpenCVSingleCalibration->setupUi( m_calibrationDialogs.back().get() );
    m_calibrationDialogs.push_back( std::shared_ptr<QDialog>( new QDialog(this) ) );   ui_OpenCVStereoCalibration->setupUi( m_calibrationDialogs.back().get() );

    // init opengl
    ui->plot_poses->setWidget( &m_worldPoses );

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
    delete ui_ChessboardFinder;
    delete ui_RandomFeatureFinder;
#ifdef UCHIYAMA_FOUND
    delete ui_UchiyamaFinder;
#endif
    delete ui_OpenCVSingleCalibration;
    delete ui_OpenCVStereoCalibration;
}


void IrisCC::update()
{
    try
    {
        // check poses
        if( m_cs.poseCount() == 0 )
            throw std::runtime_error( "IrisCC::update: No Images" );

        // configure finder
        std::shared_ptr<iris::Finder> f;
        switch( ui->select_finder->currentIndex() )
        {
            // none
            case 0:
                throw std::runtime_error("IrisCC::update: No Finder selected.");

            // chessboard
            case 1 :
            {
                ui->configure_finder->setEnabled(true);
                iris::ChessboardFinder* finder = new iris::ChessboardFinder();
                finder->configure( static_cast<size_t>( ui_ChessboardFinder->columns->value() ),
                                   static_cast<size_t>( ui_ChessboardFinder->rows->value() ),
                                   0.001 * ui_ChessboardFinder->square_size->value() );
                finder->setScale( ui_ChessboardFinder->scale->value() );
                finder->setFastCheck( ui_ChessboardFinder->fastCheck->isChecked() );
                finder->setAdaptiveThreshold( ui_ChessboardFinder->adaptiveThreshold->isChecked() );
                finder->setNormalizeImage( ui_ChessboardFinder->normalizeImage->isChecked() );
                finder->setSubpixelCorner( ui_ChessboardFinder->subpixel_corner->isChecked() );
                f = std::shared_ptr<iris::Finder>(finder);
                break;
            }

            // uchiyama
#           ifdef UCHIYAMA_FOUND
            case 2 :
            {
                iris::UchiyamaFinder* finder = new iris::UchiyamaFinder();
                finder->configure( ui_UchiyamaFinder->points->toPlainText().toStdString() );
                finder->setScale( ui_UchiyamaFinder->scale->value() );
                f = std::shared_ptr<iris::Finder>(finder);
                break;
            }
#           endif

            // random feature descriptor
            case 3 :
            {
                iris::RandomFeatureFinder* finder = new iris::RandomFeatureFinder();
                finder->configure( ui_RandomFeatureFinder->points->toPlainText().toStdString() );
                finder->setScale( ui_RandomFeatureFinder->scale->value() );
                finder->setMaxRadiusRatio( ui_RandomFeatureFinder->max_radius_ratio->value() );
                finder->setMinRadius( ui_RandomFeatureFinder->min_radius->value() );
                f = std::shared_ptr<iris::Finder>(finder);
                break;
            }

            // not supported finder
            default:
                throw std::runtime_error("IrisCC::update: Finder not supported.");
        }

        // configure calibration
        std::shared_ptr<iris::CameraCalibration> cc;
        switch( ui->select_calibration->currentIndex() )
        {
            // none
            case 0 :
                throw std::runtime_error("IrisCC::update: No Calibration selected.");

            // OpenCV
            case 1 :
            {
                iris::OpenCVSingleCalibration* calib = new iris::OpenCVSingleCalibration();
                calib->setFixPrincipalPoint( ui_OpenCVSingleCalibration->fixed_principal_point->isChecked() );
                calib->setFixAspectRatio( ui_OpenCVSingleCalibration->fixed_aspect_ratio->isChecked() );
                calib->setTangentialDistortion( ui_OpenCVSingleCalibration->tangential_distortion->isChecked() );
                calib->setMinCorrespondences( static_cast<size_t>( ui_OpenCVSingleCalibration->minCorrespondences->value() ) );
                cc = std::shared_ptr<iris::CameraCalibration>( calib );
                break;
            }

            // OpenCV Stereo
            case 2 :
            {
                iris::OpenCVStereoCalibration* calib = new iris::OpenCVStereoCalibration();
                calib->setFixPrincipalPoint( ui_OpenCVStereoCalibration->fixed_principal_point->isChecked() );
                calib->setFixAspectRatio( ui_OpenCVStereoCalibration->fixed_aspect_ratio->isChecked() );
                calib->setTangentialDistortion( ui_OpenCVStereoCalibration->tangential_distortion->isChecked() );
                calib->setRelativeToPattern( ui_OpenCVStereoCalibration->relative_to_pattern->isChecked() );
                calib->setSameFocalLength( ui_OpenCVStereoCalibration->same_focal_length->isChecked() );
                calib->setMinCorrespondences( static_cast<size_t>( ui_OpenCVStereoCalibration->minCorrespondences->value() ) );
                cc = std::shared_ptr<iris::CameraCalibration>( calib );
                break;
            }

            // not supported finder
            default:
                throw std::runtime_error("IrisCC::update: Calibration not supported.");
        }

        // run the calibration
        cc->setFinder(f);
        cc->calibrate( m_cs );

        // update the error plot
        updateErrorPlot();

        // update current view
        if( ui->image_list_detected->currentRow() < 0 )
            ui->image_list_detected->setCurrentRow( 0 );
        else
            updateImage( ui->image_list_detected->currentRow() );
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::updateErrorPlot()
{
    // init stuff
    double error = 0;

    // clear the plot
    ui->plot_error->clearGraphs();
    ui->plot_error->clearPlottables();
    ui->plot_error->legend->setVisible( m_cs.cameras().size() > 0 );

    // run over all camera poses
    for( auto camIt=m_cs.cameras().begin(); camIt != m_cs.cameras().end(); camIt++ )
    {
        // update error
        if( camIt->second.error > error )
            error = camIt->second.error;

        // add graph for this camera
        auto graph = ui->plot_error->addGraph();
        QVector<double> x, y;

        // generate random color
        QColor col;
        col.setHslF( static_cast<double>(camIt->first)/static_cast<double>(m_cs.cameras().size()), 1.0, 0.4 );

        // run over all poses of the camera
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
        {
            if( !camIt->second.poses[p].rejected )
            {
                // update more stuff
                const iris::Pose_d& pose = camIt->second.poses[p];

                // run over the points
                for( size_t i=0; i<pose.points2D.size(); i++ )
                {
                    x.push_back( pose.projected2D[i](0) - pose.points2D[i](0) );
                    y.push_back( pose.projected2D[i](1) - pose.points2D[i](1) );
                }
            }
        }

        // plot the detected points
        graph->setData(x, y);
        graph->setPen( col );
        graph->setLineStyle(QCPGraph::lsNone);
        graph->setScatterStyle(QCPGraph::ssPlus);
        graph->setScatterSize(4);

        // add graph name
        graph->setName( "Camera \"" + QString::number(camIt->first) + "\"" );
    }

    ui->plot_error->xAxis->setRange(-error, error);
    ui->plot_error->yAxis->setRange(-error, error);

    // redraw
    ui->plot_error->replot();
}


void IrisCC::updateImage( int idx )
{
    try
    {
        // init the plot
        ui->plot_image->clearGraphs();
        ui->plot_image->clearPlottables();
        ui->plot_image->setAxisBackground( QPixmap() );
        ui->plot_image->replot();

        // check if there are any images
        if( m_cs.poseCount() == 0 )
            return;

        // check which index this is
        if( idx < 0 || idx >= m_cs.poseCount() )
            idx = 0;

        // get the image
        const iris::Pose_d pose = m_cs.pose( m_poseIndices[idx] );
        const cimg_library::CImg<uint8_t>& image = *pose.image;

        // convert image to Qt
        QImage imageQt( image.width(), image.height(), QImage::Format_RGB888 );
        for( int y=0; y<image.height(); y++ )
        {
            for( int x=0; x<image.width(); x++ )
            {
                QColor col( (255+ image(x,y,0,0)) / 2,
                            (255+ image(x,y,0,1)) / 2,
                            (255+ image(x,y,0,2)) / 2 );
                imageQt.setPixel( x, y, col.rgb() );
            }
        }

        // set the images
        ui->plot_image->setAxisBackground(QPixmap::fromImage(imageQt), true, Qt::IgnoreAspectRatio );
        ui->plot_image->xAxis->setRange(0, imageQt.width() );
        ui->plot_image->yAxis->setRange(0, imageQt.height() );

        // draw the detected points
        if( !pose.rejected )
        {
            // get the pose
            const iris::Pose_d& pose = m_cs.pose( m_poseIndices[idx] );

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
            QCPGraph* detected = ui->plot_image->addGraph();
            detected->setData(x, y);
            detected->setPen( QPen( QBrush( QColor( Qt::green ) ), 2 ) );
            detected->setLineStyle(QCPGraph::lsNone);
            detected->setScatterStyle(QCPGraph::ssPlusCircle);
            detected->setScatterSize(12);

            // plot the detected points
            QCPGraph* projected = ui->plot_image->addGraph();
            projected->setData(px, py);
            projected->setPen( QPen( QBrush( QColor( Qt::red ) ), 2 ) );
            projected->setLineStyle(QCPGraph::lsNone);
            projected->setScatterStyle(QCPGraph::ssPlus);
            projected->setScatterSize(10);
        }

        // redraw
        ui->plot_image->replot();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::addImage( std::shared_ptr< cimg_library::CImg<uint8_t> > image, const QString& name )
{
    size_t id = m_cs.add( image, name.toStdString(), static_cast<size_t>( ui->cameraID->value() ) );
    m_poseIndices.push_back( id );

    // update list
    ui->image_list_detected->addItem( name );
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
    m_poseIndices.clear();

    // clear the camera set
    m_cs.cameras().clear();

    // update charts
    updateImage(0);
    updateErrorPlot();
}


void IrisCC::on_configureFinder()
{
    try
    {
        // init stuff
        ui->configure_finder->setEnabled(false);

        // choose what to do
        switch( ui->select_finder->currentIndex() )
        {
            // none
            case 0:
                break;

            case 1 : // chessboard
#           ifdef UCHIYAMA_FOUND
            case 2 : // uchiyama
#           endif
            case 3 : // random feature descriptor
                ui->configure_finder->setEnabled(true);
                m_finderDialogs[ ui->select_finder->currentIndex() ]->exec();
                break;

            // not supported finder
            default:
                throw std::runtime_error("IrisCC::on_configure_finder: Finder not supported.");
        }
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
        ui->configure_calibration->setEnabled(false);

        // choose what to do
        switch( ui->select_calibration->currentIndex() )
        {
            // none
            case 0 :
                break;

            case 1 : // OpenCV Single
            case 2 : // OpenCV Stereo
                ui->configure_calibration->setEnabled(true);
                m_calibrationDialogs[ ui->select_calibration->currentIndex() ]->exec();
                break;

            // not supported finder
            default:
                throw std::runtime_error("IrisCC::on_configureCalibration: Calibration not supported.");
        }
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
            addImage( image, QFileInfo( imagePaths[i] ).fileName() );

            // update progress
            progress.setValue(i);
        }

        // tidy up progress bar
        progress.setValue(imagePaths.size());
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
}


void IrisCC::on_update()
{
    update();
}


void IrisCC::on_save()
{
    try
    {
        // get an output filename
        //QString filename = QFileDialog::getSaveFileName(this, "Save Calibration", "calibration", "Iris Camera Calibration XML (*.xml);; Matlab Camera Clibration Toolbox TXT (*.txt)");
        QString filename = QFileDialog::getSaveFileName(this, "Save Calibration", "calibration.xml", "Iris Camera Calibration XML (*.xml)");

//        // return fi nothing there
//        if( filename.size() == 0 )
//            return;

//        // try to save
//        QFile outFile( filename );
//        if( !outFile.open( QIODevice::WriteOnly | QIODevice::Text ) )
//            throw std::runtime_error("IrisCC::on_save: could not open file for writing.");

//        // save
//        QTextStream stream( &outFile );
////        if( QFileInfo( filename ).suffix().compare( "txt", Qt::CaseInsensitive ) )
////            stream << toMatlabTXT();
////        else
//            stream << toXML();
//        outFile.close();


        m_cs.save( filename.toStdString() );
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::on_inputChanged( int page )
{
    if( 1 == page )
        on_cameraOpen();
    else
        on_cameraClose();
}


void IrisCC::on_cameraOpen()
{
    if( !m_videoCapture.isOpened() )
    {
        m_videoCapture.open(0);
        if( !m_videoCapture.isOpened() )
        {
            ui->capture_frame->setEnabled(false);
            critical("IrisCC::on_cameraOpen: could not open camera.");
        }
        else
            ui->capture_frame->setEnabled(true);
    }
    else
        warning("IrisCC::on_cameraOpen: camera already open.");
}


void IrisCC::on_cameraClose()
{
    if( !m_videoCapture.isOpened() )
        m_videoCapture.release();
    else
        warning("IrisCC::on_cameraClose: camera not open.");
}


void IrisCC::on_capture()
{
    if( m_videoCapture.isOpened() )
    {
        cv::Mat imageCV;
        m_videoCapture >> imageCV;
        cv::cvtColor(imageCV, imageCV, cv::COLOR_BGR2RGB);

        std::shared_ptr< cimg_library::CImg<uint8_t> > image( new cimg_library::CImg<uint8_t> );
        iris::cv2cimg<uint8_t,3>( imageCV, *image );

        addImage( image, "frame" );
    }
    else
        warning("IrisCC::on_capture: camera not open.");
}


void IrisCC::on_detectedImageChanged( int idx )
{
    updateImage( idx );
}
