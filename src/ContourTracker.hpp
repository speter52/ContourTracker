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

#define tab "\t"            /*  */

/* Internal parameters */
#define ARC_NUM_COLORS 128            /* Number of random colors to generate. */
#define ARC_MIN_AREA 500            /* minimum area of a good contour */
#define ARC_LAMBDA 0.60
/* Command line flags */
#define ARC_HELP_FLAG "-h"            /*  */
#define ARC_IMAGELIST_NAME "-l"            /* image list */
#define ARC_VIDOUT "-o"            /* set output filename */
#define ARC_VERBOSE_FLAG "-v"            /*  */
#define ARC_DISTANCE "-d"
#define ARC_MAHALANOBOIS "-m"
#define ARC_AREA_DIFF "-a"
#define ARC_MISCOUNT "-mc"

#define ARC_VERBOSE 1

#define ARC_DEFAULT_VIDOUT "out.avi"

#define ARC_DEFAULT_DISTANCE 50            /* Max distance between two matching contour centers */
#define ARC_DEFAULT_MAHALANOBOIS 0.60            /* Max mahalanobois distance */
#define ARC_DEFAULT_AREA 2000            /* Maximum area difference */
#define ARC_DEFAULT_MISCOUNT 40          /* Max frames without match */

/* Energy mimization params */
#define ARC_ENERGY_D 5            /* size of contour increment in pixels */
#define ARC_SOBEL_MULT 1        /* sobel mean multiplier */
Mat maskImage ( Mat image, vector<Point>& snake, Scalar c );

void matchContours ( Mat image, Contour& con, 
        vector<Contour>& newContours );
void displayContours ( Mat image, vector<Contour> tracked,
        VideoWriter vidout, vector<Scalar> colors );
void init ( int argc, char **argv, vector<string>& images,
        VideoWriter& vidout, vector<Scalar>& colors );
void help ( char **argv );
void objectToContours( vector<Contour> *contours,
        vector<vector<Point> > *vectors );
void getImageList( string filename,  vector<string>* il );
void getContours( Mat image,  vector<Contour>  *contours );
double centroidTest ( Moments& trackedMom, Moments& newMom );
double huMomentsTest ( Moments& trackedMom, Moments& newMom );
void flow( Mat prev_image, Mat image, vector<Contour>& tracked );
void minimize_energy ( Mat image, Point center );
void getUserPoints(Mat image, vector<Point> & points);
void getPointsFromMouse(int event, int x, int y, int flags, void* points);
        

#endif   // ----- #ifndef ContourTracker_INC  ----- 
