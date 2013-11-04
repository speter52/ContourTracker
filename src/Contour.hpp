#ifndef  Contour_INC
#define  Contour_INC
#include </usr/include/opencv2/opencv.hpp>
#include <cv.h>
#include <vector>
using namespace std;
using namespace cv;
#define ARC_PREVDEL_SIZE 1024            /*  */
class Contour{
	public: 
		vector<Point> contour;
        vector<Point> prevdel;
		int nomatch;
		Contour( vector<Point> cntr );
};

#endif   // ----- #ifndef Contour_INC  ----- 
