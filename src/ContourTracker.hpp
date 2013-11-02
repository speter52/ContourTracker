#ifndef  ContourTracker_INC
#define  ContourTracker_INC
#include "Contour.hpp"
#include <cv.h>
#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <vector>
#include <opencv/highgui.h>
#include <iostream>
#include <string>
#include <fstream>
using namespace std;
using namespace cv;

void init ( int argc, char **argv, vector<string>& images, VideoWriter& vidout );
void help ( char **argv );
void objectToContours( vector<Contour> *contours,  vector<vector<Point> > *vectors );
void getImageList( string filename,  vector<string>* il );
void getContours( Mat image,  vector<Contour>  *contours );
double centroidTest ( Moments& trackedMom, Moments& newMom );
double huMomentsTest ( Moments& trackedMom, Moments& newMom );
#endif   // ----- #ifndef ContourTracker_INC  ----- 
