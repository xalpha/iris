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
