/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/////////////////////////////////////////////////////////////////////////////////////
//                                                                                 //
// Copyright (c) 2012, Amin Abouee and Alexandru Duliu and Yuji Oyamada            //
// All rights reserved.                                                            //
//                                                                                 //
// Redistribution and use in source and binary forms, with or without              //
// modification, are permitted provided that the following conditions are met:     //
//                                                                                 //
// 1. Redistributions of source code must retain the above copyright notice, this  //
//    list of conditions and the following disclaimer.                             //
// 2. Redistributions in binary form must reproduce the above copyright notice,    //
//    this list of conditions and the following disclaimer in the documentation    //
//    and/or other materials provided with the distribution.                       //
//                                                                                 //
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND //
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   //
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          //
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR //
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  //
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    //
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND     //
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      //
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   //
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    //
//                                                                                 //
////////////////////////////////////////////////////////////////////////////////////

#include <limits>
#include <multiCalibrate.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

///
/// \file    multiCalibrate.cpp
///
/// \version 0.1.0
///
/// \brief   function for to calibrate multiple cameras
///
/// \details This function builds upon OpenCV's implementation of for
///          camera calibration, extending it to to also support multiple cameras
///
/// \author  Amin Abouee
/// \author  Alexandru Duliu
/// \author  Yuji Oyamada
/// \date    Nov 16, 2012
///

double multiCalibrate( cv::InputArrayOfArrays objectPoints,
                       cv::InputArrayOfArrays imagePoints,
                       cv::Size imageSize,
                       cv::InputOutputArray cameraMatrix,
                       cv::InputOutputArray distCoeffs,
                       cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs,
                       int flags )
{
    /*
    CVAPI(double) cvStereoCalibrate( const CvMat* object_points, const CvMat* image_points1,
                                   const CvMat* image_points2, const CvMat* npoints,
                                   CvMat* camera_matrix1, CvMat* dist_coeffs1,
                                   CvMat* camera_matrix2, CvMat* dist_coeffs2,
                                   CvSize image_size, CvMat* R, CvMat* T,
                                   CvMat* E CV_DEFAULT(0), CvMat* F CV_DEFAULT(0),
                                   CvTermCriteria term_crit CV_DEFAULT(cvTermCriteria(
                                       CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,1e-6)),
                                   int flags CV_DEFAULT(CV_CALIB_FIX_INTRINSIC));
    */

    //**
    const int CameraNum = imagePoints.size().height;
    const int NINTRINSIC = 12;

    cv::Ptr <cv::Mat> err, J_LR, Je, Ji, RT0; //imagePoints[2], objectPoints,npoints,  ;
    CvLevMarq solver;

    double reprojErr[CameraNum];
    double A[CameraNum][9];
    double dk[CameraNum][8];

    //Mat bigCube(3, sz, CV_8U, Scalar::all(0));
//    int szCameraMat[] = {CameraNum, 3, 3};
//    int szDistortionMat[] = {CameraNum, 1, 8};
    cv::Mat K[CameraNum];
    cv::Mat Dist[CameraNum];

    bool recomputeIntrinsics = false;
    double aspectRatio[CameraNum];
    cv::Mat om_LR, T_LR;
    cv::Mat R_LR (3, 3, CV_64F);
    int nparams;

    //int i, k, p, ni = 0, ofs, nimages, pointsTotal, maxPoints = 0;

//    CV_Assert( CV_IS_MAT(_imagePoints1) && CV_IS_MAT(_imagePoints2) &&
//               CV_IS_MAT(_objectPoints) && CV_IS_MAT(_npoints) &&
//               CV_IS_MAT(matR) && CV_IS_MAT(matT) );

//    CV_Assert( CV_ARE_TYPES_EQ(_imagePoints1, _imagePoints2) &&
//               CV_ARE_DEPTHS_EQ(_imagePoints1, _objectPoints) );

//    CV_Assert( (_npoints->cols == 1 || _npoints->rows == 1) &&
//               CV_MAT_TYPE(_npoints->type) == CV_32SC1 );

//    nimages = _npoints->cols + _npoints->rows - 1;
//    npoints = cvCreateMat( _npoints->rows, _npoints->cols, _npoints->type );
//    cvCopy( _npoints, npoints );

//    for( i = 0, pointsTotal = 0; i < nimages; i++ )
//    {
//        maxPoints = MAX(maxPoints, npoints->data.i[i]);
//        pointsTotal += npoints->data.i[i];
//    }

//    objectPoints = cvCreateMat( _objectPoints->rows, _objectPoints->cols,
//                                CV_64FC(CV_MAT_CN(_objectPoints->type)));
//    cvConvert( _objectPoints, objectPoints );
//    cvReshape( objectPoints, objectPoints, 3, 1 );

    for( int k = 0; k < CameraNum; k++ )
    {
        cv::Mat cvVectorPoints2D_K = imagePoints.getMat();
        cv::Mat cvVectorPoints3D = objectPoints.getMat();
        std::vector<cv::Mat> rotationVectors;
        std::vector<cv::Mat> translationVectors;
        K[k]= cv::Mat(3,3,CV_64F,cv::Scalar::all(0));
        Dist[k]= cv::Mat(1,8,CV_64F,cv::Scalar::all(0));

        double error = cv::calibrateCamera( cvVectorPoints3D,
                                            cvVectorPoints2D_K,
                                            imageSize,
                                            K[k],
                                            Dist[k],
                                            rotationVectors,
                                            translationVectors,
                                            flags );
//        const CvMat* points = k == 0 ? _imagePoints1 : _imagePoints2;
//        const CvMat* cameraMatrix = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
//        const CvMat* distCoeffs = k == 0 ? _distCoeffs1 : _distCoeffs2;

//        int cn = CV_MAT_CN(_imagePoints1->type);
//        CV_Assert( (CV_MAT_DEPTH(_imagePoints1->type) == CV_32F ||
//                    CV_MAT_DEPTH(_imagePoints1->type) == CV_64F) &&
//                   ((_imagePoints1->rows == pointsTotal && _imagePoints1->cols*cn == 2) ||
//                    (_imagePoints1->rows == 1 && _imagePoints1->cols == pointsTotal && cn == 2)) );

//        K[k] = cvMat(3,3,CV_64F,A[k]);
//        Dist[k] = cvMat(1,8,CV_64F,dk[k]);

//        imagePoints[k] = cvCreateMat( points->rows, points->cols, CV_64FC(CV_MAT_CN(points->type)));
//        cvConvert( points, imagePoints[k] );
//        cvReshape( imagePoints[k], imagePoints[k], 2, 1 );

//        if( flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS|
//                     CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_FIX_FOCAL_LENGTH) )
//            cvConvert( cameraMatrix, &K[k] );

//        if( flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS|
//                     CV_CALIB_FIX_K1|CV_CALIB_FIX_K2|CV_CALIB_FIX_K3|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5|CV_CALIB_FIX_K6) )
//        {
//            CvMat tdist = cvMat( distCoeffs->rows, distCoeffs->cols,
//                                 CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), Dist[k].data.db );
//            cvConvert( distCoeffs, &tdist );
//        }

//        if( !(flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS)))
//        {
//            cvCalibrateCamera2( objectPoints, imagePoints[k],
//                                npoints, imageSize, &K[k], &Dist[k], 0, 0, flags );
//        }
    }
    //**



    return std::numeric_limits<double>::max();
}
