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

using std::ptrdiff_t;

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/flann/flann.hpp>

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
    std::vector< Eigen::Matrix<T,2,1> > detected2D;
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
        intrinsic = cam.intrinsic;
        distortion = cam.distortion;
        error = cam.error;
    }


public:
    size_t id;
    std::vector< Pose<T> > poses;
    Eigen::Vector2i imageSize;
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
    switch( src.depth() )
    {
        case 1 : cv2cimg<T, 1>(src, dst); break;
        case 2 : cv2cimg<T, 2>(src, dst); break;
        case 3 : cv2cimg<T, 3>(src, dst); break;
        case 4 : cv2cimg<T, 4>(src, dst); break;
        default:
            throw std::runtime_error( "iris::cv2cimg: unsupported number of channels." );
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

    std::cout << "iris::pixelLimit: width: " << image.width() << ", height: " << image.height() << ", MP: " << static_cast<double>(pixelCount)/1000000.0 << "(" << times << "x)" << std::endl;

    return result;
}


/////
// angle comparisson operator
///

// thanks to Ben Voigt and Tom Sirgedas for putting me on the right track
// http://stackoverflow.com/questions/7774241/sort-points-by-angle-from-given-axis

template<typename T>
class counter_clockwise_comparisson
{
    typedef Eigen::Matrix<T,2,1> Point;

public:
    counter_clockwise_comparisson(const Point& origin, const Point& direction)
    {
        m_origin = origin;
        m_direction = direction;
    }

    bool operator()(const Point& a, const Point& b) const
    {
        // init stuff
        Point dirA = a - m_origin;
        Point dirB = b - m_origin;

        // check A
        T detB = det(m_direction, dirB);
        T dotB = dot(m_direction, dirB);
        if( (0==detB) && (0==dotB) )
            return false;

        // check B
        T detA = det(m_direction, dirA);
        T dotA = dot(m_direction, dirA);
        if( (0==detA) && (0==dotA) )
            return true;

        // a&b on the same side
        if (detA * detB > 0)
            return det(dirA, dirB) > 0;
        else
            return detA > 0;
    }

private:
    // determinant of 2x2 matrix [a b]
    double det( const Point& a, const Point& b) const { return a(0)*b(1) - a(1)*b(0); }
    double dot( const Point& a, const Point& b) const { return a(0)*b(0) + a(1)*b(1); }

private:
    Point m_origin;
    Point m_direction;
};


/////
// count bits in an integer
///
template <typename T>
inline size_t count_bits( const T number )
{
    // init stuff
    size_t c; // c accumulates the total bits set in v
    size_t n = static_cast<size_t>(number);

    // count
    for( c=0; n; c++)
        n &= n - 1; // clear the least significant bit set

    // return
    return c;
}


/////
// compute all possible combinations of K out of N elements
///
template<size_t N, size_t K>
inline std::vector< std::vector<size_t> > possible_combinations()
{
    // compute the maximum number of N combinations
    uint64_t maxC = 1;
    maxC = maxC << N;
    std::vector<uint64_t> bits;
    std::vector< std::vector<size_t> > result;

    // run over all possibilities
    for( uint64_t i=0; i<maxC; i++ )
        if( count_bits(i) == K )
            bits.push_back( i );

    // run over all recovered combinations and extract the indices
    for( size_t i=0; i<bits.size(); i++ )
    {
        // update stuff
        uint64_t x = static_cast<uint64_t>(bits[i]);
        std::vector<uint64_t> indices;

        // run over all bits and store their positions
        for (uint64_t z = 0; z < 64; z++)
            if( (x>>z) & 0x1 )
                indices.push_back(z);

        // add the indices to the result
        result.push_back( indices );
    }

    // return
    return result;
}


/////
// generate all shift permutations of the vector
///
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


/////
// Area of a triangle
///
template<typename T>
inline T area( const Eigen::Matrix<T,2,1>& A,
               const Eigen::Matrix<T,2,1>& B,
               const Eigen::Matrix<T,2,1>& C )
{
    Eigen::Matrix<T,2,1> a = B-A;
    Eigen::Matrix<T,2,1> b = C-A;

    return static_cast<T>(0.5) * (a(0)*b(1) - a(1)*b(0));
}


template <typename T>
inline double crossRatio( const Eigen::Matrix<T,2,1>& A,
                          const Eigen::Matrix<T,2,1>& B,
                          const Eigen::Matrix<T,2,1>& C,
                          const Eigen::Matrix<T,2,1>& D,
                          const Eigen::Matrix<T,2,1>& E )
{
    // compute the cross ratio of five coplanar points
    // area(A,B,C)*area(A,D,E) / area(A,B,D)*area(A,C,E)

    double result = area( A, B, D ) * area( A, C, E );

    if( fabs( result ) < std::numeric_limits<double>::epsilon() )
        return std::numeric_limits<double>::max();
    else
        return ( area( A, B, C ) * area( A, D, E ) ) / result;
}


template <typename T>
inline double affineInvariant( const Eigen::Matrix<T,2,1>& A,
                               const Eigen::Matrix<T,2,1>& B,
                               const Eigen::Matrix<T,2,1>& C,
                               const Eigen::Matrix<T,2,1>& D )
{
    // compute the ratio of triangle areas
    // area(A,C,D) / area(A,B,C)

    double result = area( A, B, C );

    if( fabs( result ) < std::numeric_limits<double>::epsilon() )
        return std::numeric_limits<double>::max();
    else
        return area( A, C, D ) / result;
}

template<size_t K>
inline double descriptor( const std::vector<Eigen::Vector2d>& n );

template<>
inline double descriptor<5>( const std::vector<Eigen::Vector2d>& points )
{
    return crossRatio( points[0], points[1], points[2], points[3], points[4] );
}


template<>
inline double descriptor<4>( const std::vector<Eigen::Vector2d>& points )
{
    return affineInvariant( points[0], points[1], points[2], points[3] );
}


/////
// Find duplicates
///
template<typename T>
inline bool duplicates( const std::vector<T>& vec )
{
    for( size_t i=0; i<vec.size(); i++ )
        for( size_t j=0; j<vec.size(); j++ )
            if( i != j && vec[i] == vec[j] )
                return true;

    return false;
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




} // end namespace iris


