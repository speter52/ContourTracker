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
	vector<Point> temp;
	for(  size_t i=0; i<cntr.size( ); i++ ){
		Point point( cntr[i]);
		contour.push_back( point );
		temp.push_back( point );
	}
	Point top1; Point top2; 
	top1 = cntr[0];
	int topCounter = 0;
	for(  size_t i=1;i<4;i++ ){
		if( top1.y>temp[i].y ){
			top1 = temp[i];
			topCounter = i;
		}	
	}
	temp.erase( temp.begin( )+topCounter );
	top2 = cntr[0];
	for(  size_t i=1;i<3;i++ ){
		if( top2.y>temp[i].y ){
			top2 = temp[i];
		}
	}
	TL = top1;
	if( top1.x>top2.x ) TL = top2;
	BR = temp[0];
	if( BR.x<temp[1].x ) BR=temp[1];

	nomatch = 0;

}

