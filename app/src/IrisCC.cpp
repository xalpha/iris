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
#include "ui_CameraConfig.h"
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
    ui_CameraConfig( new Ui::CameraConfig ),
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
//    connect( ui->input, SIGNAL(currentChanged(int)), this, SLOT(on_inputChanged(int)) );
//    connect( ui->capture, SIGNAL(clicked(bool)), this, SLOT(on_capture(void)) );

    // camera config
    ui_CameraConfig->setupUi( &m_cameraDialog );
    connect( ui->select_camera, SIGNAL(currentIndexChanged(int)), this, SLOT(on_selectCamera(void)) );
    connect( ui->configure_camera, SIGNAL(clicked(bool)), this, SLOT(on_configureCamera(void)) );
    connect( &m_cameraDialog, SIGNAL(accepted(void)), this, SLOT( on_acceptConfigureCamera(void) ) );

    // finder
    connect( ui->select_finder, SIGNAL(currentIndexChanged(int)), this, SLOT(on_configureFinder(void)) );
    connect( ui->configure_finder, SIGNAL(clicked(bool)), this, SLOT(on_configureFinder(void)) );

    // calibration
    connect( ui->select_calibration, SIGNAL(currentIndexChanged(int)), this, SLOT(on_configureCalibration(void)) );
    connect( ui->configure_calibration, SIGNAL(clicked(bool)), this, SLOT(on_configureCalibration(void)) );

    connect( ui->load, SIGNAL(clicked(bool)), this, SLOT(on_load(void)) );
    connect( ui->clear, SIGNAL(clicked(bool)), this, SLOT(on_clear(void)) );
    connect( ui->image_list, SIGNAL(currentRowChanged(int)), this, SLOT(on_detectedImageChanged(int)) );
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
    Eigen::Matrix4d mv;
    mv << 1, 0, 0, 0,
          0, 0,-1, 0,
          0, 1, 0, 0,
          0, 0, 0, 1;
    m_worldPoses.setMV(mv);

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
    delete ui_CameraConfig;
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
                finder->setScale( ui_ChessboardFinder->scale->value() );
                finder->configure( static_cast<size_t>( ui_ChessboardFinder->columns->value() ),
                                   static_cast<size_t>( ui_ChessboardFinder->rows->value() ),
                                   0.001 * ui_ChessboardFinder->square_size->value() );
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
                finder->setScale( ui_UchiyamaFinder->scale->value() );
                finder->configure( ui_UchiyamaFinder->points->toPlainText().toStdString() );
                f = std::shared_ptr<iris::Finder>(finder);
                break;
            }
#           endif

            // random feature descriptor
            case 3 :
            {
                iris::RandomFeatureFinder* finder = new iris::RandomFeatureFinder();
                finder->setScale( ui_RandomFeatureFinder->scale->value() );
                finder->configure( ui_RandomFeatureFinder->points->toPlainText().toStdString() );
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
                calib->setIntrinsicGuess( static_cast<size_t>( ui_OpenCVSingleCalibration->intrinsic_guess->isChecked() ) );
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
                calib->setFixIntrinsic( static_cast<size_t>( ui_OpenCVStereoCalibration->fix_intrinsic->isChecked() ) );
                calib->setIntrinsicGuess( static_cast<size_t>( ui_OpenCVStereoCalibration->intrinsic_guess->isChecked() ) );
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
        updateImageList();
        updateErrorPlot();
        updatePosesPlot();
        updateImage( ui->image_list->currentRow() );
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::updateImageList()
{
    // init stuff
    int currentRow =  ui->image_list->currentRow();
    ui->image_list->clear();
    m_poseIndices.clear();

    // run over all camera poses
    for( auto camIt=m_cs.cameras().begin(); camIt != m_cs.cameras().end(); camIt++ )
    {
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
        {
            ui->image_list->addItem( QString( camIt->second.poses[p].name.c_str() ) );
            m_poseIndices.push_back( camIt->second.poses[p].id );

            if( camIt->second.poses[p].rejected )
                ui->image_list->item( ui->image_list->count() -1 )->setBackgroundColor( QColor( 255, 128, 128 ) );
        }
    }

    // set the current row
    if( currentRow < 0 )
    {
        if( ui->image_list->count() > 0 )
            ui->image_list->setCurrentRow( 0 );
        else
            ui->image_list->setCurrentRow( currentRow );
    }
    else
    {
        if( ui->image_list->count() > 0 && currentRow < ui->image_list->count() )
            ui->image_list->setCurrentRow( currentRow );
        else if( ui->image_list->count() > 0 )
            ui->image_list->setCurrentRow( 0 );
        else
            ui->image_list->setCurrentRow( -1 );
    }
}


void IrisCC::updateErrorPlot()
{
    // init stuff
    double range = 1.5;

    // clear the plot
    ui->plot_error->clearGraphs();
    ui->plot_error->clearPlottables();

    // run over all camera poses
    for( auto camIt=m_cs.cameras().begin(); camIt != m_cs.cameras().end(); camIt++ )
    {
        // update error
        if( camIt->second.error > range )
            range = camIt->second.error;

        // run over all poses of the camera
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
        {
            if( !camIt->second.poses[p].rejected )
            {
                // update more stuff
                const iris::Pose_d& pose = camIt->second.poses[p];
                auto graph = ui->plot_error->addGraph();

                // generate color
                QColor col;
                double l  = static_cast<double>(p)/static_cast<double>(camIt->second.poses.size());
                col.setHslF( 0.5843, 1.0, 0.3 + 0.4*l );

                // run over the points
                for( size_t i=0; i<pose.points2D.size(); i++ )
                    graph->addData( pose.projected2D[i](0) - pose.points2D[i](0),
                                    pose.projected2D[i](1) - pose.points2D[i](1) );

                // plot the detected points
                graph->setPen( col );
                graph->setLineStyle(QCPGraph::lsNone);
                graph->setScatterStyle(QCPGraph::ssPlus);
                graph->setScatterSize(4);
            }
        }
    }

    // draw current camera
    if(  ui->select_camera->currentIndex() >= 0 )
    {
        // run over all poses of the camera
        const iris::Camera_d& cam = m_cs.camera( getCameraId( ui->select_camera->currentIndex() ) );
        for( size_t p=0; p<cam.poses.size(); p++ )
        {
            if( !cam.poses[p].rejected )
            {
                // update more stuff
                const iris::Pose_d& pose = cam.poses[p];
                auto graph = ui->plot_error->addGraph();

                // generate color
                QColor col;
                double l  = static_cast<double>(p)/static_cast<double>(cam.poses.size());
                col.setHslF( 1.0, 1.0, 0.3 + 0.4*l );

                // run over the points
                for( size_t i=0; i<pose.points2D.size(); i++ )
                    graph->addData( pose.projected2D[i](0) - pose.points2D[i](0),
                                    pose.projected2D[i](1) - pose.points2D[i](1) );

                // plot the detected points
                graph->setPen( col );
                graph->setLineStyle(QCPGraph::lsNone);
                graph->setScatterStyle(QCPGraph::ssPlus);
                graph->setScatterSize(4);
            }
        }
    }

    // draw the current pose
    if( ui->image_list->currentRow() > 0 )
    {
        const iris::Pose_d& pose = m_cs.pose( m_poseIndices[ ui->image_list->currentRow() ] );
        if( !pose.rejected )
        {
            // points background
            auto graphBg = ui->plot_error->addGraph();
            for( size_t i=0; i<pose.points2D.size(); i++ ) graphBg->addData( pose.projected2D[i](0) - pose.points2D[i](0), pose.projected2D[i](1) - pose.points2D[i](1) );
            graphBg->setPen( QPen( QBrush( QColor( Qt::black ) ), 3 ) );
            graphBg->setLineStyle(QCPGraph::lsNone);
            graphBg->setScatterStyle(QCPGraph::ssPlus);
            graphBg->setScatterSize(6);

            // points
            auto graph = ui->plot_error->addGraph();
            for( size_t i=0; i<pose.points2D.size(); i++ ) graph->addData( pose.projected2D[i](0) - pose.points2D[i](0), pose.projected2D[i](1) - pose.points2D[i](1) );
            graph->setPen( QPen( QBrush( QColor( 115, 210, 22 ) ), 1.5 ) );
            graph->setLineStyle(QCPGraph::lsNone);
            graph->setScatterStyle(QCPGraph::ssPlus);
            graph->setScatterSize(5);
        }
    }

    // update the range
    ui->plot_error->xAxis->setRange(-range, range);
    ui->plot_error->yAxis->setRange(-range, range);

    // update the tick size
    double ts = 1.0;
    while((ts * 10.0) < range)
        ts *= 10.0;
    ui->plot_error->xAxis->setTickStep( ts*0.5 );
    ui->plot_error->yAxis->setTickStep( ts*0.5 );

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
            double height = static_cast<double>(imageQt.height());
            for( size_t i=0; i<pose.points2D.size(); i++ )
            {
                // get the color hue
                double hue = static_cast<double>(3*pose.pointIndices[i])/static_cast<double>(4*pose.pointsMax);

                // plot the background of detected points
                QCPGraph* detectedBg = ui->plot_image->addGraph();
                detectedBg->addData( pose.points2D[i](0), height - pose.points2D[i](1) );
                QColor detectedBgCol;
                detectedBgCol.setHslF( hue, 0.8, 0.8 );
                detectedBg->setPen( QPen( QBrush( detectedBgCol ), 3 ) );
                detectedBg->setLineStyle(QCPGraph::lsNone);
                detectedBg->setScatterStyle(QCPGraph::ssPlus);
                detectedBg->setScatterSize(12);

                // plot the detected points
                QCPGraph* detected = ui->plot_image->addGraph();
                detected->addData( pose.points2D[i](0), height - pose.points2D[i](1) );
                QColor detectedCol;
                detectedCol.setHslF( hue, 0.8, 0.3 );
                detected->setPen( QPen( QBrush( detectedCol ), 1 ) );
                detected->setLineStyle(QCPGraph::lsNone);
                detected->setScatterStyle(QCPGraph::ssPlus);
                detected->setScatterSize(12);

                // plot the reprojected points
                QCPGraph* projected = ui->plot_image->addGraph();
                projected->addData(pose.projected2D[i](0), height - pose.projected2D[i](1));
                QColor projectedCol;
                projectedCol.setHslF( hue, 0.8, 0.6 );
                projected->setPen( QPen( QBrush( projectedCol ), 1.5 ) );
                projected->setLineStyle(QCPGraph::lsNone);
                projected->setScatterStyle(QCPGraph::ssCircle);
                projected->setScatterSize(8);
            }
        }

        // redraw
        ui->plot_image->replot();
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


void IrisCC::updatePosesPlot()
{
    // init stuff
    std::vector<Eigen::Matrix4d> RTs;
    std::vector<Eigen::Vector3d> points3D;
    std::vector<Eigen::Vector4d> colors;
    m_worldPoses.clear();

    // run over all camera poses
    for( auto camIt=m_cs.cameras().begin(); camIt != m_cs.cameras().end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            if( !camIt->second.poses[p].rejected )
            {
                // get the transformation of the pose
                Eigen::Affine3d trans(camIt->second.poses[p].transformation);
                trans = trans.inverse();
                Eigen::Matrix4d rt = trans.matrix();
                RTs.push_back( rt );

                // get the points in this pose
                for( size_t k=0; k<camIt->second.poses[p].points3D.size(); k++ )
                {
                    double hue = static_cast<double>(3*camIt->second.poses[p].pointIndices[k])/static_cast<double>(4*camIt->second.poses[p].pointsMax);
                    double alpha = static_cast<double>(1) / static_cast<double>(camIt->second.poses.size());
                    QColor col;
                    col.setHslF( hue, 0.8, 0.4 );

                    points3D.push_back( camIt->second.poses[p].points3D[k] );
                    colors.push_back( Eigen::Vector4d( col.redF(), col.greenF(), col.blueF(), alpha ) );
                }
            }

    // update widget
    if( RTs.size() > 0 )
    {
        ui->plot_poses->update();
        m_worldPoses.setLineWidth( 1 );
        m_worldPoses( RTs, nox::plot<double>::CS );
        m_worldPoses.setPointSize( 5 );
        m_worldPoses( points3D, colors );
        ui->plot_poses->update();
    }
}


void IrisCC::updatePosesPlotCurrent()
{
    m_worldPoses.clear( 1 );

    // draw the current pose
    if( ui->image_list->currentRow() > 0 )
    {
        const iris::Pose_d& pose = m_cs.pose( m_poseIndices[ ui->image_list->currentRow() ] );
        if( !pose.rejected )
        {
            m_worldPoses.setLineWidth( 3 );
            Eigen::Affine3d trans( pose.transformation);
            trans = trans.inverse();
            m_worldPoses( trans.matrix(), nox::plot<double>::CS | nox::plot<double>::Pos, 1 );
            ui->plot_poses->update();
        }
    }

}


void IrisCC::updateCameraList()
{
    // init stuff
    int currentRow =  ui->select_camera->currentIndex();
    ui->select_camera->clear();

    // run over all cameras and add their names
    for( auto camIt=m_cs.cameras().begin(); camIt != m_cs.cameras().end(); camIt++ )
        ui->select_camera->addItem( QString("Camera: \"") + QString::number(camIt->first) + QString("\"") );

    // set the current row
    if( currentRow < 0 )
    {
        if( ui->select_camera->count() > 0 )
            ui->select_camera->setCurrentIndex( 0 );
        else
            ui->select_camera->setCurrentIndex( currentRow );
    }
    else
    {
        if( ui->select_camera->count() > 0 && currentRow < ui->select_camera->count() )
            ui->select_camera->setCurrentIndex( currentRow );
        else if( ui->select_camera->count() > 0 )
            ui->select_camera->setCurrentIndex( 0 );
        else
            ui->select_camera->setCurrentIndex( -1 );
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
    m_worldPoses.clear();

    // clear images
    m_poseIndices.clear();

    // clear the camera set
    m_cs.cameras().clear();

    // update charts
    updateCameraList();
    updateImageList();
    updateImage(-1);
    updateErrorPlot();
    updatePosesPlot();
}


size_t IrisCC::getCameraId( int comboBoxIdx )
{
    // get the camera Id
    int camIdx = 0;
    size_t camId = 0;
    for( auto camIt=m_cs.cameras().begin(); camIt != m_cs.cameras().end(); camIt++ )
    {
        if( comboBoxIdx == camIdx )
        {
            camId = camIt->first;
            break;
        }
        else
            camIdx++;
    }

    return camId;
}


void IrisCC::on_selectCamera()
{
    if( ui->select_camera->currentIndex() < 0 )
        ui->configure_camera->setEnabled(false);
    else
    {
        ui->configure_camera->setEnabled(true);
        updateErrorPlot();
    }
}


void IrisCC::on_configureCamera()
{
    if( ui->select_camera->currentIndex() >= 0 )
    {
        // get the camera
        const iris::Camera_d& cam = m_cs.camera( getCameraId( ui->select_camera->currentIndex() ) );

        // update the dialog
        ui_CameraConfig->camera_box->setTitle( QString("Camera: \"") + QString::number(cam.id) + QString("\"   (") + QString::number( cam.imageSize(0) ) + "x" + QString::number( cam.imageSize(1) ) + ")" );
        ui_CameraConfig->fx->setValue( cam.intrinsic(0,0) );
        ui_CameraConfig->fy->setValue( cam.intrinsic(1,1) );
        ui_CameraConfig->cx->setValue( cam.intrinsic(0,2) );
        ui_CameraConfig->cy->setValue( cam.intrinsic(1,2) );

        // set intrinsic matrix
        std::stringstream sim;
        sim << cam.intrinsic;
        ui_CameraConfig->intrinsic_matrix->setText( QString(sim.str().c_str()) );

        // set the distortion
        ui_CameraConfig->distortion_model->setText( "OpenCV" );
        QString dist;
        for( size_t i=0; i<cam.distortion.size(); i++ )
            dist += QString::number( cam.distortion[i] ) + "\n";
        ui_CameraConfig->distortion_params->setText( dist );

        // show the dialog
        m_cameraDialog.exec();
    }
}


void IrisCC::on_acceptConfigureCamera()
{
    if( ui->select_camera->currentIndex() >= 0 && ui_CameraConfig->edit->isChecked() )
    {
        // get the camera
        iris::Camera_d& cam = m_cs.camera( getCameraId( ui->select_camera->currentIndex() ) );
        cam.intrinsic(0,0) = ui_CameraConfig->fx->value();
        cam.intrinsic(1,1) = ui_CameraConfig->fy->value();
        cam.intrinsic(0,2) = ui_CameraConfig->cx->value();
        cam.intrinsic(1,2) = ui_CameraConfig->cy->value();
    }
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
        QStringList imagePaths = QFileDialog::getOpenFileNames(this, "Load Images", ".", "Images (*.bmp *.png *.xpm *.jpg *.tif *.tiff)");

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
            m_cs.add( image,
                      QFileInfo( imagePaths[i] ).fileName().toStdString(),
                      static_cast<size_t>( ui->cameraID->value() ) );

            // update progress
            progress.setValue(i);
        }

        // tidy up progress bar
        progress.setValue(imagePaths.size());

        // update the image list
        updateImageList();
        updateCameraList();
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
        QString filename = QFileDialog::getSaveFileName(this, "Save Calibration", "calibration.xml", "Iris Camera Calibration XML (*.xml)");

        m_cs.save( filename.toStdString() );
    }
    catch( std::exception &e )
    {
        critical( e.what() );
    }
}


//void IrisCC::on_inputChanged( int page )
//{
//    if( 1 == page )
//        on_cameraOpen();
//    else
//        on_cameraClose();
//}


//void IrisCC::on_cameraOpen()
//{
//    if( !m_videoCapture.isOpened() )
//    {
//        m_videoCapture.open(0);
//        if( !m_videoCapture.isOpened() )
//        {
//            ui->capture_frame->setEnabled(false);
//            critical("IrisCC::on_cameraOpen: could not open camera.");
//        }
//        else
//            ui->capture_frame->setEnabled(true);
//    }
//    else
//        warning("IrisCC::on_cameraOpen: camera already open.");
//}


//void IrisCC::on_cameraClose()
//{
//    if( !m_videoCapture.isOpened() )
//        m_videoCapture.release();
//    else
//        warning("IrisCC::on_cameraClose: camera not open.");
//}


//void IrisCC::on_capture()
//{
//    if( m_videoCapture.isOpened() )
//    {
//        cv::Mat imageCV;
//        m_videoCapture >> imageCV;
//        cv::cvtColor(imageCV, imageCV, cv::COLOR_BGR2RGB);

//        std::shared_ptr< cimg_library::CImg<uint8_t> > image( new cimg_library::CImg<uint8_t> );
//        iris::cv2cimg<uint8_t,3>( imageCV, *image );

//        // assemble name
//        std::stringstream ss;
//        ss << "frame_" << m_cs.poseCount();
//        m_cs.add( image, ss.str() );
//    }
//    else
//        warning("IrisCC::on_capture: camera not open.");
//}


void IrisCC::on_detectedImageChanged( int idx )
{
    updateImage( idx );
    updateErrorPlot();
    updatePosesPlotCurrent();
}
