#ifndef  Contour_INC
#define  Contour_INC
#include <cv.h>
#include <vector>
using namespace std;
using namespace cv;
class Contour{
	public: 
		vector<Point> contour;
		Point TL;
		Point BR;
		int nomatch;
		Contour( vector<Point> cntr );
};

#endif   // ----- #ifndef Contour_INC  ----- 
