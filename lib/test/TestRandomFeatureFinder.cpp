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

#include <assert.h>
#include <iostream>
#include <stdexcept>

#include <iris/RandomFeatureFinder.hpp>


void test_rff( const std::string& path )
{
    // init finder
    iris::RandomFeatureFinder finder;
    finder.setScale( 0.0001 );
    finder.configure( "2359 1487 390 718 240 439 1048 667 1931 643 1900 1367 1896 1544 2476 1252 2307 986 530 357 184 1378 2503 1447 2069 709 452 163 1159 1647 632 432 2242 1644 1743 1501 1089 1129 1494 292 356 1154 800 1033 155 1575 1181 1397 663 823 2088 959 529 1392 891 356 1534 681 391 344 618 1539 985 1230 1164 162 2188 384 1319 792 984 953 2074 1399 1345 1166 1503 1061 873 798 1563 939 1604 499 2348 147 788 620 1960 1201 234 669 1966 379 933 1396 1638 188 2387 335 2537 1141 2394 604 2278 483 1309 507 879 1141 836 1307 330 1426 1448 449 730 1368 1159 333 1774 652 1745 795 266 901 2231 1320 2163 253 1742 1023 347 508 524 580 1676 1302 2101 1641 1956 522 1132 1009 1466 1368 2011 1069 1808 912 1370 1573 629 1667 995 227 1184 818 1029 1529 1933 926 487 920 2563 838 130 1130 1471 817 695 327 1840 483 120 1255 1340 920 1217 1121 2387 1350 609 1118 2197 615 498 1594 989 546 243 224 1306 1350 2517 276 1333 168 2368 744 778 180 2523 399 2230 798 428 1038 1853 283 1348 291 769 1636 1917 790 2442 1001 2424 1667 1813 162 1768 1620 1484 1244 1467 149 2431 849 1513 1489 1277 1471 1702 1167 1720 279 1010 380 907 1602 1553 1624 575 228 2030 134 284 1044 2324 1190 348 1619 1163 456 665 571 1875 1053 1271 631 516 1259 605 699 2118 522 2548 553 124 314 1409 703 277 1254 173 991 2044 269 897 141 795 491 2208 1459 2579 170 910 644 612 945 2568 1637 2563 1023 531 795 743 1222" );
    finder.setMaxRadiusRatio( 3.0 );
    finder.setMinRadius( 5.0 );

    // load the image
    iris::Pose_d pose;
    pose.name = path;
    pose.image = std::shared_ptr< cimg_library::CImg<uint8_t> >( new cimg_library::CImg<uint8_t>(path.c_str()) );

    // run the finder
    finder.find( pose );
}


int main(int argc, char** argv)
{
    try
    {        
        test_rff( "/home/duliu/Pictures/Webcam/cam5_uchiya/2012-07-19-111715.jpg" );
    }
    catch( std::exception &e )
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}







