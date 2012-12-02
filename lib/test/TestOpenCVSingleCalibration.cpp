#include <iostream>

#include <iris/util.hpp>

// Finders
#include <iris/ChessboardFinder.hpp>

// CameraCalibrations
#include <iris/OpenCVSingleCalibration.hpp>
#include <iris/OpenCVStereoCalibration.hpp>

void showPose( const iris::Pose_d& pose )
{
    // get the image
    cv::Mat img;
    iris::cimg2cv( *pose.image, img );
    cv::cvtColor( img, img, CV_BGR2GRAY );
    cv::cvtColor( img, img, CV_GRAY2RGB );

    // get the name
    std::stringstream ss;
    ss << pose.id;
    std::string name = ss.str();

    // render the points
    for( size_t i=0; i<pose.points2D.size(); i++ )
        cv::circle( img, cv::Point( pose.points2D[i](0), pose.points2D[i](1) ), 5, cv::Scalar(0,255,0), -1, 8, 0 );
    for( size_t i=0; i<pose.projected2D.size(); i++ )
        cv::circle( img, cv::Point( pose.projected2D[i](0), pose.projected2D[i](1) ), 3, cv::Scalar(0,0,255), -1, 8, 0 );

    // show the image
    cv::namedWindow( name, 0 );
    cv::imshow( name, img );
    cvResizeWindow( name.c_str(), 1024, 768 );
}


void testCalibration( int argc, char** argv )
{
    // init stuff
    iris::ChessboardFinder* finder = new iris::ChessboardFinder();
    iris::OpenCVSingleCalibration* cc = new iris::OpenCVSingleCalibration();
    std::shared_ptr<iris::Finder> finderPtr(finder);
    std::shared_ptr<iris::CameraCalibration> ccPtr(cc);
    iris::CameraSet_d cs;

    // configure finder
    //finder->configure( "235.9	148.7	39	71.8	24	43.9	104.8	66.7	193.1	64.3	190	136.7	189.6	154.4	247.6	125.2	230.7	98.6	53	35.7	18.4	137.8	250.3	144.7	206.9	70.9	45.2	16.3	115.9	164.7	63.2	43.2	224.2	164.4	174.3	150.1	108.9	112.9	149.4	29.2	35.6	115.4	80	103.3	15.5	157.5	118.1	139.7	66.3	82.3	208.8	95.9	52.9	139.2	89.1	35.6	153.4	68.1	39.1	34.4	61.8	153.9	98.5	123	116.4	16.2	218.8	38.4	131.9	79.2	98.4	95.3	207.4	139.9	134.5	116.6	150.3	106.1	87.3	79.8	156.3	93.9	160.4	49.9	234.8	14.7	78.8	62	196	120.1	23.4	66.9	196.6	37.9	93.3	139.6	163.8	18.8	238.7	33.5	253.7	114.1	239.4	60.4	227.8	48.3	130.9	50.7	87.9	114.1	83.6	130.7	33	142.6	144.8	44.9	73	136.8	115.9	33.3	177.4	65.2	174.5	79.5	26.6	90.1	223.1	132	216.3	25.3	174.2	102.3	34.7	50.8	52.4	58	167.6	130.2	210.1	164.1	195.6	52.2	113.2	100.9	146.6	136.8	201.1	106.9	180.8	91.2	137	157.3	62.9	166.7	99.5	22.7	118.4	81.8	102.9	152.9	193.3	92.6	48.7	92	256.3	83.8	13	113	147.1	81.7	69.5	32.7	184	48.3	12	125.5	134	92	121.7	112.1	238.7	135	60.9	111.8	219.7	61.5	49.8	159.4	98.9	54.6	24.3	22.4	130.6	135	251.7	27.6	133.3	16.8	236.8	74.4	77.8	18	252.3	39.9	223	79.8	42.8	103.8	185.3	28.3	134.8	29.1	76.9	163.6	191.7	79	244.2	100.1	242.4	166.7	181.3	16.2	176.8	162	148.4	124.4	146.7	14.9	243.1	84.9	151.3	148.9	127.7	147.1	170.2	116.7	172	27.9	101	38	90.7	160.2	155.3	162.4	57.5	22.8	203	13.4	28.4	104.4	232.4	119	34.8	161.9	116.3	45.6	66.5	57.1	187.5	105.3	127.1	63.1	51.6	125.9	60.5	69.9	211.8	52.2	254.8	55.3	12.4	31.4	140.9	70.3	27.7	125.4	17.3	99.1	204.4	26.9	89.7	14.1	79.5	49.1	220.8	145.9	257.9	17	91	64.4	61.2	94.5	256.8	163.7	256.3	102.3	53.1	79.5	74.3	122.2	" );
    finder->configure( 12, 11, 0.01 );

    // configure cc
    cc->setFinder( finderPtr );

    // load & add images
    for( size_t i=1; i<argc; i++ )
    {
        std::string path( argv[i] );
        std::shared_ptr<cimg_library::CImg<uint8_t> > image( new cimg_library::CImg<uint8_t> );
        image->load( path.c_str() );

        cs.add( image, path );
    }

    // perform the calibration
    cc->calibrate( cs );

    // run over all poses
    const std::map< size_t, iris::Camera_d >& cameras = cs.cameras();
    for( auto camIt = cameras.begin(); camIt != cameras.end(); camIt++ )
        for( size_t p=0; p<camIt->second.poses.size(); p++ )
            showPose( camIt->second.poses[p] );

    // save results
    cs.save( "test.xml" );

    while( true )
    {
        //Handle pause/unpause and ESC
        int c = cv::waitKey(15);
        if(c == 'q')
            break;
    }
}


void testLoadXML( const std::string& filename )
{
    iris::CameraSet_d cs;
    cs.load( filename );
    cs.save( "test.xml" );
}


int main(int argc, char** argv)
{
    // check
    if( argc == 1 )
    {
        std::cerr << "IrisTest: No Input." << std::endl;
        return -1;
    }

    // do the work
    try
    {
        if( argc == 2 )
        {
            // try to identify the file
            std::string filename( argv[1] );
            size_t extIdx = filename.find_last_of( '.' ) + 1;
            std::string extension = filename.substr( extIdx, filename.size() -1 );

            if( extension.compare( "xml" ) == 0 ||
                extension.compare( "Xml" ) == 0 ||
                extension.compare( "XML" ) == 0 )
                testLoadXML( filename );
            else
                testCalibration( argc, argv );
        }
        else
            testCalibration( argc, argv );
    }
    catch( std::exception& e )
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
