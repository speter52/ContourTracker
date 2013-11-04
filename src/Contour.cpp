// =====================================================================================
//
//       Filename:  Contour.cpp
//
//    Description:  Class for storing contour info.
//
//        Version:  1.0
//        Created:  08/01/2013 02:50:00 PM
//       Revision:  none
//       Compiler:  gcc
//
//         Author:  Martin Miller (), miller7@illinois.edu
//   Organization:  Aerospace Robotics and Control Lab
//
// =====================================================================================

#include "Contour.hpp"
#include <iostream>
//Takes in a vector of 4 points and figures out the top left and bottom right point
Contour::Contour( vector<Point> cntr )
{
    contour = cntr;
    prevdel = vector<Point>( ARC_PREVDEL_SIZE, Point(0, 0) );
	nomatch = 0;
}

