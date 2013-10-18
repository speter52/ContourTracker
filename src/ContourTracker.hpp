#ifndef  ContourTracker_INC
#define  ContourTracker_INC
#include "Contour.hpp"
#include <cv.h>
#include <vector>
#include <opencv/highgui.h>
#include <iostream>
#include <string>
#include <fstream>

#define tab "\t"            /*  */

/* Command line flags */
#define ARC_HELP_FLAG "-h"            /*  */
#define ARC_IMAGELIST_NAME "-l"            /* image list */
#define ARC_NUM_CONTOURS "-n"            /* Number of contours to track */
#define ARC_VERBOSE_FLAG "-v"            /*  */

#define ARC_VERBOSE 1
#define ARC_NUM_CONTOURS_DEFAULT 1            /*  */

#define ARC_CANNY_SIMPLE 0            /* find edges using canny */
#define ARC_CANNY_ITER 1            /* find edges using iterative canny */
#define ARC_ENERGY_MIN 2            /* find edges energy minimization (not implemented */

#define ARC_MIN_AREA 2500            /*  */
#define ARC_RECT_MARGIN 10            /* margin around bounding rect for mask when matching contours */
#define ARC_ALPHA 0.70
#define ARC_MIN_SCORE 0.45            /*  */
using namespace std;
using namespace cv;

bool init( int argc, char **argv, vector<string>& image_list,
        VideoWriter& vidout, vector<Scalar>& colors, int* num_contours );
void help ( char** argv );
void get_image_list(string filename, vector<string>& il);
int find_contours( Mat& image, vector<vector<Point> >& contours, int num_contours, int algorithm );
void flow();
void match_contours();
void display_contours( Mat& image, vector<vector<Point> >& contours, VideoWriter& v, vector<Scalar>& colors );

#endif   // ----- #ifndef ContourTracker_INC  ----- 
