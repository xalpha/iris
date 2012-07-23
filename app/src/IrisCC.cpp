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
#include <QDomDocument>

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
    // init stuff
    double error = 0;

    // clear the plot
    ui->plot_error->clearGraphs();
    ui->plot_error->clearPlottables();
    ui->plot_error->legend->setVisible( m_calibration->cameras().size() > 0 );

    // run over all camera poses
    for( auto camIt=m_calibration->cameras().begin(); camIt != m_calibration->cameras().end(); camIt++ )
    {
        // update error
        if( camIt->second.error > error )
            error = camIt->second.error;

        // add graph for this camera
        auto graph = ui->plot_error->addGraph();
        QVector<double> x, y;

        // generate random color
        QColor col;
        col.setHslF( static_cast<double>(camIt->first)/static_cast<double>(m_calibration->cameras().size()), 1.0, 0.4 );

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

        // check if there are any images
        if( m_calibration->poseCount() == 0 )
            return;

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

        // set the images
        ui->plot_image->setAxisBackground(QPixmap::fromImage(imageQt), true, Qt::IgnoreAspectRatio );
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
    size_t id = m_calibration->addImage( image, name.toStdString(), static_cast<size_t>( ui->cameraID->value() ) );
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

    // clear the calibration
    m_calibration->clear();

    // update charts
    updateImage(0);
    updateErrorPlot();
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


QDomElement IrisCC::addDomElement( QDomDocument &doc,
                                QDomNode &node,
                                const QString &tag,
                                const QString &value )
{
    QDomElement el = doc.createElement( tag );
    node.appendChild( el );

    if( !value.isNull() )
    {
        QDomText txt = doc.createTextNode( value );
        el.appendChild( txt );
    }

    return el;
}


//QString IrisCC::toXML()
//{
//    // assemble the file tree
//    QDomDocument doc("CameraCalibration");
//    QDomProcessingInstruction instr = doc.createProcessingInstruction("xml", "version='1.0' encoding='UTF-8'");
//    doc.appendChild(instr);

//    // add root
//    QDomElement root = addDomElement(doc, doc, "CameraCalibration");

//    // run over the camers
//    QDomElement cameras = addDomElement(doc,root,"Cameras");
//    for( auto camIt=m_calibration->cameras().begin(); camIt != m_calibration->cameras().end(); camIt++ )
//    {
//        // init camera
//        QDomElement camera = addDomElement(doc,cameras,"Camera");

//        // add camera data
//        addDomElement(doc,camera,"Id", QString::number( camIt->second.id ) );
//        addDomElement(doc,camera,"ImageSize", toString( camIt->second.imageSize ) );
//        addDomElement(doc,camera,"Intrinsic", toString( camIt->second.intrinsic ) );
//        addDomElement(doc,camera,"Distortion", toString( camIt->second.distortion ) );
//        addDomElement(doc,camera,"Error", QString::number( camIt->second.error ) );

//        // run over all its poses and add them
//        QDomElement poses = addDomElement(doc,camera,"Poses");
//        for( size_t p=0; p<camIt->second.poses.size(); p++ )
//        {
//            // add the pose
//            QDomElement pose = addDomElement(doc,poses,"Pose");

//            // add pose Id and the filename
//            for( size_t pid=0; pid<m_poseIndices.size(); pid++ )
//            {
//                if( m_poseIndices[pid] == camIt->second.poses[p].id )
//                {
//                    addDomElement(doc,pose,"Id", m_poseFilenames[pid] );
//                    break;
//                }
//            }

//            // if not rejected, also add the rest
//            if( !camIt->second.poses[p].rejected )
//            {
//                addDomElement(doc,pose,"Points2D", toString( camIt->second.poses[p].points2D ) );
//                addDomElement(doc,pose,"Points3D", toString( camIt->second.poses[p].points3D ) );
//                addDomElement(doc,pose,"PointIndices", toString( camIt->second.poses[p].pointIndices ) );
//                addDomElement(doc,pose,"Transformation", toString( camIt->second.poses[p].transformation ) );
//                addDomElement(doc,pose,"ProjectedPoints", toString( camIt->second.poses[p].projected2D ) );
//            }
//        }
//    }

//    return doc.toString();
//}


//QString IrisCC::toMatlabTXT()
//{
//    return QString();
//}


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
            // chessboard
            case 0 :
            {
                ui->configure_finder->setEnabled(true);
                Ui::ChessboardFinder chessboardFinderUI;
                chessboardFinderUI.setupUi( &dialog );
                dialog.exec();
                iris::ChessboardFinder* finder = new iris::ChessboardFinder();
                finder->configure( static_cast<size_t>( chessboardFinderUI.columns->value() ),
                                   static_cast<size_t>( chessboardFinderUI.rows->value() ),
                                   0.001 * chessboardFinderUI.square_size->value() );
                finder->setFastCheck( chessboardFinderUI.fastCheck->isChecked() );
                finder->setAdaptiveThreshold( chessboardFinderUI.adaptiveThreshold->isChecked() );
                finder->setNormalizeImage( chessboardFinderUI.normalizeImage->isChecked() );
                finder->setSubpixelCorner( chessboardFinderUI.subpixel_corner->isChecked() );
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
            // OpenCV
            case 0 :
            {
                ui->configure_calibration->setEnabled(true);
                Ui::OpenCVSingleCalibration form;
                form.setupUi( &dialog );
                dialog.exec();
                iris::OpenCVSingleCalibration* calib = new iris::OpenCVSingleCalibration();
                calib->setFixPrincipalPoint( form.fixed_principal_point->isChecked() );
                calib->setFixAspectRatio( form.fixed_aspect_ratio->isChecked() );
                calib->setTangentialDistortion( form.tangential_distortion->isChecked() );
                calib->setMinCorrespondences( static_cast<size_t>( form.minCorrespondences->value() ) );
                calib->copyCameras( m_calibration );
                m_calibration = std::shared_ptr<iris::CameraCalibration>( calib );
                break;
            }

            // OpenCV Stereo
            case 1 :
            {
                ui->configure_calibration->setEnabled(true);
                Ui::OpenCVStereoCalibration form;
                form.setupUi( &dialog );
                dialog.exec();
                iris::OpenCVStereoCalibration* calib = new iris::OpenCVStereoCalibration();
                calib->setFixPrincipalPoint( form.fixed_principal_point->isChecked() );
                calib->setFixAspectRatio( form.fixed_aspect_ratio->isChecked() );
                calib->setTangentialDistortion( form.tangential_distortion->isChecked() );
                calib->setRelativeToPattern( form.relative_to_pattern->isChecked() );
                calib->setSameFocalLength( form.same_focal_length->isChecked() );
                calib->setMinCorrespondences( static_cast<size_t>( form.minCorrespondences->value() ) );
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
            addImage( image, QFileInfo( imagePaths[i] ).fileName() );

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


        m_calibration->save( filename.toStdString() );
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
