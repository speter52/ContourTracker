//Simon Peter
//Class to find contours for VISP library
//speter3@illinois.edu

#ifndef FUNCTION_CREATOR_H
#define FUNCTION_CREATOR_H

#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

class ARC_FindContours
{
	public:

		ARC_FindContours();
		ARC_FindContours(int minArea, int maxArea, bool runApproxPolyDP, bool runConvexHull);
		ARC_FindContours(int minArea, int maxArea, int lowerAngle, int upperAngle, bool runApproxPolyDP, bool runConvexHull);
		void set_min_area(int minArea);
		void set_max_area(int maxArea);
		void set_lower_angle(int lowerAngle);
		void set_upper_angle(int upperAngle);
		void set_approx_poly(bool runApproxPolyDP);
		void set_convex_hull(bool runConvexHull);
		bool get_contours(Mat image, vector<vector<Point> > & contours);
		bool get_quads(Mat image, vector<vector<Point> > & contours);

	private:

		int minContourArea;
		int maxContourArea;
		int lowerAngleThreshold;
		int upperAngleThreshold;
		int numOfSides;
		bool runPoly;
		bool runHull;

		double dot(Point A, Point B);		
		double findMagnitude(Point vector);
		double angleBetween(Point point1, Point vertex, Point point2);
		bool angleTest(vector<Point> contour);
		bool applyThresholds(vector<Point> & contour);
		void filterContours(vector<vector<Point> > & curContours, vector<vector<Point> > & newContours);
		void getCanny(Mat & image);
};
#endif		
