////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of iris, a lightweight C++ camera calibration library    //
//                                                                            //
// Copyright (C) 2012 Alexandru Duliu                                         //
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
#include <list>
#include <memory>
#include <string>
#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

// opencv
using std::ptrdiff_t;
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/flann/flann.hpp>

// cimg
#define cimg_display 0
#include <CImg.h>

namespace iris
{

/////
// Pose
///
template <typename T>
class Pose
{
public:
    Pose()
    {
        id = -1;
        transformation = Eigen::Matrix<T,4,4>::Identity();
        rejected = true;
    }


    Pose( const Pose& pose )
    {
        *this = pose;
    }


    void operator =( const Pose& pose )
    {
        id = pose.id;
        name = pose.name;
        image = pose.image;
        points2D = pose.points2D;
        points3D = pose.points3D;
        pointIndices = pose.pointIndices;
        transformation = pose.transformation;
        projected2D = pose.projected2D;
        rejected = pose.rejected;
    }


public:
    // id
    size_t id;
    std::string name;
    // image
    std::shared_ptr< cimg_library::CImg<uint8_t> > image;
    // correspondences
    std::vector< Eigen::Matrix<T,2,1> > points2D;
    std::vector< Eigen::Matrix<T,3,1> > points3D;
    std::vector<size_t> pointIndices;
    size_t pointsMax;
    // calibration results
    Eigen::Matrix<T,4,4> transformation;
    std::vector< Eigen::Matrix<T,2,1> > projected2D;
    bool rejected;
};
typedef Pose<double> Pose_d;


/////
// Camera
///
template <typename T>
class Camera
{
public:
    Camera()
    {
        intrinsic = Eigen::Matrix<T,3,3>::Identity();
        error = 0;
    }


    Camera( const Camera& cam )
    {
        *this = cam;
    }


    void operator =( const Camera& cam )
    {
        id = cam.id;
        poses = cam.poses;
        imageSize = cam.imageSize;
        sensorSize = cam.sensorSize;
        intrinsic = cam.intrinsic;
        distortion = cam.distortion;
        error = cam.error;
    }


public:
    size_t id;
    std::vector< Pose<T> > poses;
    Eigen::Vector2i imageSize;
    Eigen::Matrix<T,2,1> sensorSize;
    Eigen::Matrix<T,3,3> intrinsic;
    std::vector<T> distortion;
    T error;
};
typedef Camera<double> Camera_d;


/////
// OpenCV Points
///
template <typename To, typename Te>
inline std::vector<cv::Point_<To> > eigen2cv( const std::vector<Eigen::Matrix<Te,2,1> >& points2D )
{
    // init stuff
    std::vector<cv::Point_<To> > cvPoints2D;

    // convert the points
    for( size_t j=0; j<points2D.size(); j++ )
        cvPoints2D.push_back( cv::Point_<To>( static_cast<To>( points2D[j](0) ),
                                              static_cast<To>( points2D[j](1) ) ) );

    // return
    return cvPoints2D;
}


template <typename To, typename Te>
inline std::vector<cv::Point3_<To> > eigen2cv( const std::vector<Eigen::Matrix<Te,3,1> >& points3D )
{
    // init stuff
    std::vector<cv::Point3_<To> > cvPoints3D;

    // convert the points
    for( size_t j=0; j<points3D.size(); j++ )
        cvPoints3D.push_back( cv::Point3_<To>( static_cast<To>( points3D[j](0) ),
                                               static_cast<To>( points3D[j](1) ),
                                               static_cast<To>( points3D[j](2) ) ) );

    // return
    return cvPoints3D;
}


/////
// OpenCV Matrices
///
template <typename To, typename Ti>
inline cv::Mat_<To> vector2cv( const std::vector< std::vector<Ti> >& dat )
{
    // init stuff
    cv::Mat_<float> result( dat.size(), dat[0].size() );

        for( size_t i=0; i<dat.size(); i++ )
            for( size_t j=0; j<dat[i].size(); j++ )
                result( i, j ) = static_cast<To>(dat[i][j]);

    // return
    return result;
}


/////
// OpenCV Transformation
///
template <typename T, int Rows, int Cols>
void cv2eigen( const cv::Mat& rot, const cv::Mat& transl, Eigen::Matrix<T,Rows,Cols>& mat )
{
    // init stuff
    Eigen::Transform<T,3,Eigen::Affine> trans;
    trans.setIdentity();

    try
    {
        // get the rotation
        cv::Mat rotationCV( 3, 3, rot.type() );
        cv::Rodrigues( rot, rotationCV );
        Eigen::Matrix<T,3,3> rotation;
        cv::cv2eigen( rotationCV, rotation );

        // get the translation
        Eigen::Matrix<T,3,1> translation;
        cv::cv2eigen( transl, translation );

        // convert to a Eigen transformation
        trans.translate( translation );
        trans.rotate( rotation );
    }
    catch( std::exception& e )
    {
        throw std::runtime_error("iris::cv2eigen: type of openCV rotation and translation matrices don't match that of the output matrix.");
    }

    // return
    //mat = trans.matrix().cast<T>;
    mat = trans.matrix();
}


template <typename T, int Rows, int Cols>
inline void eigen2cv( const Eigen::Matrix<T, Rows, Cols>& trans, cv::Mat& rot, cv::Mat& transl )
{
    // generate the affine trans
    Eigen::Transform<T, 3, Eigen::Affine> affine( trans );

    // decompose the affine transformation
    Eigen::Matrix<T, 3, 3> rotation = affine.rotation();
    Eigen::Matrix<T, 3, 1> translation = affine.translation();

    // convert rotation
    cv::Mat rotationCV( 3, 3, rot.type() );
    cv::eigen2cv( rotation, rotationCV );
    cv::Rodrigues( rotationCV, rot );

    // convert translation
    cv::eigen2cv( translation, transl );
}


/////
// from Eigen to String (with love)
///
template <typename T>
inline std::string toString( const T val )
{
    std::stringstream ss;
    ss << val;

    return ss.str();
}


template <typename T, int Rows, int Cols>
inline std::string toString( const Eigen::Matrix<T,Rows,Cols>& mat )
{
    std::stringstream ss;
    for( int i=0; i<Rows*Cols; i++ )
        ss << mat.data()[i] << " ";

    return ss.str();
}


template <typename T>
inline std::string toString( const std::vector<T>& vec )
{
    std::stringstream ss;
    for( size_t i=0; i<vec.size(); i++ )
        ss << vec[i] << " ";

    return ss.str();
}


template <typename T, int Rows, int Cols>
inline std::string toString( const std::vector< Eigen::Matrix<T,Rows,Cols> >& vec )
{
    std::stringstream ss;
    for( size_t i=0; i<vec.size(); i++ )
        ss << toString( vec[i] ) << "; ";

    return ss.str();
}


/////
// from String to Eigen (also with love)
///
template <typename T>
inline void str2scalar( std::string str, T& result )
{
    std::stringstream ss;
    ss << str;
    ss.seekg( 0, std::ios::beg );
    ss >> result;
}


template <typename T>
inline void str2vector( const std::string& str, std::vector<T>& result )
{
    std::stringstream ss;
    ss << str;
    ss.seekg( 0, std::ios::beg );
    while( !ss.eof() )
    {
        T tmp;
        ss >> tmp;
        result.push_back( tmp );
    }
    result.pop_back();
}


template <typename T, int Rows, int Cols>
inline void str2eigen( std::string str, Eigen::Matrix<T,Rows,Cols>& result )
{
    std::stringstream ss;
    ss << str;
    ss.seekg( 0, std::ios::beg );
    for( int i=0; i<Rows*Cols && str.size() > Rows*Cols; i++ )
        ss >> result.data()[i];
}


template <typename T, int Rows, int Cols>
inline void str2eigenVector( std::string str, std::vector< Eigen::Matrix<T,Rows,Cols> >& result )
{
    size_t start=0;
    size_t finish = str.find_first_of( ';' );
    while( finish != std::string::npos )
    {
        // get the matrix
        Eigen::Matrix<T,Rows,Cols> mat;
        str2eigen( str.substr( start, finish - start ), mat );
        result.push_back( mat );

        // advance indices
        start=finish + 1;
        finish = str.find_first_of( ';', start );
    }
}



/////
// OpenCV Image to CImg
///
template <typename T, int Ch>
inline void cimg2cv( const cimg_library::CImg<T>& src, cv::Mat& dst )
{
    // some runtime checks
    if( Ch != src.spectrum() )
        throw std::runtime_error( "iris::cimg2cv: channel count of source image does not match template parameter." );

    // init result
    typedef cv::Vec<T,Ch> Pix;
    cv::Mat_<Pix> result( src.height(), src.width() ); // don't forget cv::Mat works on rows and columns and not width and height ;)

    // convert
    for( int y=0; y<src.height(); y++ )
        for( int x=0; x<src.width(); x++ )
            for( int c=0; c<Ch; c++ )
                result(y,x)[c] = src( x, y, 0, c );

    // pass the result
    dst = result;
}


template <typename T>
inline void cimg2cv( const cimg_library::CImg<T>& src, cv::Mat& dst )
{
    switch( src.spectrum() )
    {
        case 1 : cimg2cv<T, 1>(src, dst); break;
        case 2 : cimg2cv<T, 2>(src, dst); break;
        case 3 : cimg2cv<T, 3>(src, dst); break;
        case 4 : cimg2cv<T, 4>(src, dst); break;
        default:
            throw std::runtime_error( "iris::cimg2cv: unsupported number of channels." );
    }
}


template <typename T, int Ch>
inline void cv2cimg( const cv::Mat& src, cimg_library::CImg<T>& dst )
{
    // init result
    typedef cv::Vec<T,Ch> Pix;
    cimg_library::CImg<uint8_t> result( src.cols, src.rows, 1, Ch );

    // convert
    for( int y=0; y<src.rows; y++ )
        for( int x=0; x<src.cols; x++ )
            for( int c=0; c<Ch; c++ )
                result( x, y, 0, c ) = src.at<Pix>(y,x)[c];

    // pass the result
    dst = result;
}


template <typename T>
inline void cv2cimg( const cv::Mat& src, cimg_library::CImg<T>& dst )
{
    switch( src.channels() )
    {
        case 1 : cv2cimg<T, 1>(src, dst); break;
        case 2 : cv2cimg<T, 2>(src, dst); break;
        case 3 : cv2cimg<T, 3>(src, dst); break;
        case 4 : cv2cimg<T, 4>(src, dst); break;
        default:
            throw std::runtime_error( "iris::cv2cimg: unsupported number of channels (" + toString( src.channels() ) + ")." );
    }
}


/////
// Eigen Cross Matrix
///
template <typename T>
inline Eigen::Matrix<T,3,3> crossMatrix( const Eigen::Matrix<T,3,1>& v )
{
    Eigen::Matrix<T,3,3> result;
    result <<   0, -v(2),  v(1),
             v(2),     0, -v(0),
            -v(1),  v(0),     0;
    return result;
}


/////
// Eigen Vector Product (or whatever this stupidity is called)
///
template <typename T>
inline Eigen::Matrix<T,3,3> vectorProduct( const Eigen::Matrix<T,3,1>& a, const Eigen::Matrix<T,3,1>& b )
{
    Eigen::Matrix<T,3,3> result;
    result(0,0) = a(0)*b(0);
    result(0,1) = a(0)*b(1);
    result(0,2) = a(0)*b(2);
    result(1,0) = a(1)*b(0);
    result(1,1) = a(1)*b(1);
    result(1,2) = a(1)*b(2);
    result(2,0) = a(2)*b(0);
    result(2,1) = a(2)*b(1);
    result(2,2) = a(2)*b(2);
    return result;
}


/////
// Undistort pose
///
template <typename Timg, typename Tmat>
inline void undistort( const Camera<Tmat>& camera, const Pose<Tmat> &pose, cimg_library::CImg<Timg> &image )
{
    // get the pose
    cv::Mat_<uint8_t> imageCV;
    cv::Mat_<uint8_t> imageCVout;
    cimg2cv( *pose.image, imageCV );
    cv::Mat_<Tmat> intrinsic;
    cv::eigen2cv( camera.intrinsic, intrinsic );
    cv::Mat_<Tmat> distCoeff( camera.distortion );

    // undistort the image
    cv::undistort( imageCV, imageCVout, intrinsic, distCoeff );

    // save result
    cv2cimg( imageCVout, image );
}


/////
// Commandline Progress bar
///
template <typename T>
class progress
{
public:
    progress( const std::string& message, T steps ) :
        m_message(message),
        m_step(0),
        m_steps(steps)
    {}


    ~progress(){ reset(); }


    void step( T step )
    {
        m_step = (step) < m_steps ? step : m_steps;
        update();
    }


    void next_step()
    {
        step( m_step + static_cast<T>(1) );
    }


    void reset()
    {
        m_step = 0;
    }


    void finish()
    {
        m_step = m_steps;
        update();
        std::cout << std::endl;
    }


    void update()
    {
        // assemble the progressbar
        size_t pc = (100.0*static_cast<size_t>(m_step)) / static_cast<size_t>(m_steps);
        std::string pb = " [";
        for( size_t i=0; i<20; i++ )
            pb += ( (i*100)/20 <= pc ) ? "#" : " ";
        pb += "] " + toString( pc ) + " (" + toString(m_step) + "/" + toString(m_steps) +")";


        std::cout << '\r' << m_message << pb;
        std::cout.flush();
    }

protected:
    std::string m_message;
    T m_step;
    T m_steps;
};



/////
// limit an image to a certain pixel count
///
template<typename T>
inline cimg_library::CImg<T> pixelLimit( const cimg_library::CImg<T>& image, const size_t maxPix=2000000, const size_t slack=1000000 )
{
    // init stuff
    cimg_library::CImg<T> result = image;
    int pixelCount = image.width()*image.height();
    size_t times = 0;

    // determine the best scale factor
    while( (pixelCount > (maxPix + slack)) &&
           ( abs( pixelCount - maxPix ) > slack ) )
    {
        result.resize_halfXY();
        pixelCount = result.width()*result.height();
        times++;
    }

    return result;
}


/////
// angle
///
template<typename T>
inline T angle( T x, T y )
{
    T result = atan2( y, x );
    return (result < static_cast<T>(0)) ? 6.28318530718 + result : result;
}


/////
// Angle comparisson operator
//
// thanks to Ben Voigt and Tom Sirgedas for putting me on the right track
// http://stackoverflow.com/questions/7774241/sort-points-by-angle-from-given-axis
///

template<typename T>
class counter_clockwise_comparisson
{
    typedef Eigen::Matrix<T,2,1> Point;

public:
    counter_clockwise_comparisson() : m_origin( 0, 0 ){}
    counter_clockwise_comparisson(const Point& origin) : m_origin( origin ){}

    bool operator()(const Point& a, const Point& b) const
    {
        // init stuff
        Point dirA = a - m_origin;
        Point dirB = b - m_origin;

        // compute the angles
        T angleA = iris::angle( dirA(0), dirA(1) );
        T angleB = iris::angle( dirB(0), dirB(1) );

        // return the order
        return angleA < angleB;
    }

private:
    Point m_origin;
};



template<typename T>
class clockwise_comparisson
{
    typedef Eigen::Matrix<T,2,1> Point;

public:
    clockwise_comparisson() : m_origin( 0, 0 ){}
    clockwise_comparisson(const Point& origin) : m_origin( origin ){}

    bool operator()(const Point& a, const Point& b) const
    {
        // init stuff
        Point dirA = a - m_origin;
        Point dirB = b - m_origin;

        // compute the angles
        T angleA = iris::angle( dirA(0), dirA(1) );
        T angleB = iris::angle( dirB(0), dirB(1) );

        // return the order
        return angleA > angleB;
    }

private:
    Point m_origin;
};



template <typename T>
inline size_t count_bits( const T number )
{
    // init stuff
    size_t c; // c accumulates the total bits set in number
    T n = number;

    // count
    for( c=0; n; c++)
        n &= n - 1; // clear the least significant bit set

    // return
    return c;
}


template <typename T>
inline std::vector< std::vector<T> > possible_combinations( T n, T k )
{
    // compute the maximum number of N combinations
    std::vector< std::vector<T> > result;
    std::vector<bool> v(n);
    std::fill(v.begin() + k, v.end(), true);

    do
    {
        std::vector<T> c;
        for(T i = 0; i < n; ++i)
            if (!v[i])
                c.push_back(i);
        result.push_back(c);
    }
    while (std::next_permutation(v.begin(), v.end()));

    // return
    return result;
}


template <typename T>
inline std::vector< std::vector<T> > shift_combinations( const std::vector<T>& vec )
{
    // init stuff
    std::vector< std::vector<T> > result;

    for( size_t i=0; i<vec.size(); i++ )
    {
        std::vector<T> line;
        for( size_t j=0; j<vec.size(); j++ )
            line.push_back( vec[ (i+j) % vec.size() ] );
        result.push_back( line );
    }

    return result;
}


template <typename T>
inline T area( const Eigen::Matrix<T,2,1>& A,
               const Eigen::Matrix<T,2,1>& B,
               const Eigen::Matrix<T,2,1>& C )
{
    Eigen::Matrix<T,2,1> a = B-A;
    Eigen::Matrix<T,2,1> b = C-A;

    return 0.5 * (a(0)*b(1) - a(1)*b(0));
}


template <typename T>
inline T crossRatio( const Eigen::Matrix<T,2,1>& A,
                     const Eigen::Matrix<T,2,1>& B,
                     const Eigen::Matrix<T,2,1>& C,
                     const Eigen::Matrix<T,2,1>& D,
                     const Eigen::Matrix<T,2,1>& E )
{
    // compute the cross ratio of five coplanar points
    // area(A,B,C)*area(A,D,E) / area(A,B,D)*area(A,C,E)
    T result = area( A, B, D ) * area( A, C, E );

    if( fabs( result ) < std::numeric_limits<T>::epsilon() )
        return std::numeric_limits<T>::max();
    else
        return ( area( A, B, C ) * area( A, D, E ) ) / result;
}



template <typename T>
inline T affineInvariant( const Eigen::Matrix<T,2,1>& A,
                          const Eigen::Matrix<T,2,1>& B,
                          const Eigen::Matrix<T,2,1>& C,
                          const Eigen::Matrix<T,2,1>& D )
{
    // compute the ratio of triangle areas
    // area(A,C,D) / area(A,B,C)
    T result = area( A, B, C );

    if( fabs( result ) < std::numeric_limits<T>::epsilon() )
        return std::numeric_limits<T>::max();
    else
        return area( A, C, D ) / result;
}


/////
// Sum
//
template<typename T>
inline T sum( const std::vector<T> &x )
{
    // init stuff
    T s = 0;
    for( size_t i=0; i<x.size(); i++ )
        s += x[i];

    return s;
}


/////
// Mean
///
template<typename T>
inline T mean( const std::vector<T> &x )
{
    // init stuff
    T sum = 0;
    T fac = 1.0 / static_cast<T>(x.size());
    for( size_t i=0; i<x.size(); i++ )
        sum += fac * x[i];

    return sum;
}


/////
// Covariance
///
template<typename T>
inline T cov( const std::vector<T> &x, const std::vector<T> &y, bool is_subset=false )
{
    // init stuff
    T mX = mean(x);
    T mY = mean(y);
    T c = 0;
    T fac = static_cast<T>( x.size() - (is_subset ? 1 : 0 ) );

    // add everything up
    for( size_t i=0; i<x.size(); i++ )
        c += (x[i] - mX) * (y[i] - mY) * fac;

    return c;
}


/////
// Variance
///
template<typename T>
inline T var( const std::vector<T> &x, bool is_subset=false )
{
    return cov(x,x,is_subset);
}


/////
// Standard Deviation
///
template<typename T>
inline T std_dev( const std::vector<T> &x, bool is_subset=false )
{
    return sqrt( var(x, is_subset) );
}


/////
// n choose k number of combinations
// Knuth's "The Art of Computer Programming, 3rd Edition, Volume 2: Seminumerical Algorithms"
///
template <typename T>
inline T n_choose_k(T n, T k)
{
    if (k > n)
        return 0;

    T r = 1;
    for (T d = 1; d <= k; ++d)
    {
        r *= n--;
        r /= d;
    }
    return r;
}


/////
// Generate Random points with within range and min distance
///
template <typename T>
inline std::vector<Eigen::Matrix<T,2,1> > generate_points( const size_t count,
                                                           const T minDist,
                                                           const Eigen::Matrix<T,2,1>& minP,
                                                           const Eigen::Matrix<T,2,1>& maxP )
{
    // init stuff
    std::vector<Eigen::Matrix<T,2,1> > points;
    points.reserve( count );
    Eigen::Matrix<T,2,1> scale = static_cast<T>(0.5) * (maxP - minP) - Eigen::Matrix<T,2,1>::Constant(minDist);
    Eigen::Matrix<T,2,1> trans = scale + minP + Eigen::Matrix<T,2,1>::Constant(minDist);

    // fill with points
    for( size_t i=0; i<count; i++ )
    {
        while( true )
        {
            // get new point
            Eigen::Matrix<T,2,1> p = Eigen::Matrix<T,2,1>::Random();
            p(0) *= scale(0);
            p(1) *= scale(1);
            p += trans;

            // compute distance to closest point
            bool collision = false;
            for( size_t n=0; n<points.size() && !collision; n++ )
                collision = ( points[n] - p ).norm() < minDist;

            // check for collision
            if( !collision )
            {
                points.push_back( p );
                break;
            }
        }
    }

    // return result
    return points;
}
template <typename T>
inline std::vector<Eigen::Matrix<T,2,1> > generate_points( const size_t count, const T minDist )
{
    Eigen::Matrix<T,2,1> minP(0,0), maxP(1,1);
    return generate_points( count, minDist, minP, maxP );
}


/////
// Project Points
///
template <typename T, int Dim>
inline Eigen::Matrix<T,Dim,1> project_point( const Eigen::Matrix<T,Dim+1,Dim+1>& P,
                                             const Eigen::Matrix<T,Dim,1>& point )
{
    Eigen::Matrix<T,Dim+1,1> projected = P * point.homogeneous();
    return projected.hnormalized();
}

template <typename T, int Dim>
inline std::vector<Eigen::Matrix<T,Dim,1> > project_points( const Eigen::Matrix<T,Dim+1,Dim+1>& P,
                                                            const std::vector<Eigen::Matrix<T,Dim,1> >& points )
{
    std::vector< Eigen::Matrix<T,Dim,1> > pp( points.size() );
    for( size_t i=0; i<pp.size(); i++ )
        pp[i] = project_point<T,Dim>( P, points[i] );
    return pp;
}


} // end namespace iris


