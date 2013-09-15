#ifndef  ContourTracker_INC
#define  ContourTracker_INC
#include "Contour.hpp"
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <fstream>
using namespace std;
using namespace cv;

void objectToContours( vector<Contour> *contours,  vector<vector<Point> > *vectors );
void getImageList( string filename,  vector<string>* il );
double angleBetween( Point point1,  Point vertex,  Point point2 );
void findRectangles( Mat image,  vector<Contour>  *contours );
#endif   // ----- #ifndef ContourTracker_INC  ----- 
