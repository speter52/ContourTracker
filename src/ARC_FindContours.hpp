//Simon Peter
//Class to find contours for VISP library
//speter3@illinois.edu

#ifndef FUNCTION_CREATOR_H
#define FUNCTION_CREATOR_H

#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <iostream>
#include <string>

class ARC_FindContours
{
	public:

		ARC_FindContours();
		ARC_FindContours(int minArea, int maxArea, bool runApproxPolyDP, bool runConvexHull);
		ARC_FindContours(int minArea, int maxArea, int lowerAngle, int upperAngle,
                bool runApproxPolyDP, bool runConvexHull);
		void set_min_area(int minArea);
		void set_max_area(int maxArea);
		void set_lower_angle(int lowerAngle);
		void set_upper_angle(int upperAngle);
		void set_approx_poly(bool runApproxPolyDP);
		void set_convex_hull(bool runConvexHull);
		bool get_contours(cv::Mat image, 
                std::vector<std::vector<cv::Point> >& curContours,
                std::vector<std::vector<cv::Point> >& newContours );
		bool get_quads(cv::Mat image, 
                std::vector<std::vector<cv::Point> >& curContours,
                std::vector<std::vector<cv::Point> >& newContours );

	private:

		int minContourArea;
		int maxContourArea;
		int lowerAngleThreshold;
		int upperAngleThreshold;
		int numOfSides;
		bool runPoly;
		bool runHull;

		double dot(cv::Point A, cv::Point B);		
		double findMagnitude(cv::Point vector);
		double angleBetween(cv::Point point1, cv::Point vertex, cv::Point point2);
		bool angleTest(std::vector<cv::Point> contour);
		bool applyThresholds(std::vector<cv::Point>& contour);
		void filterContours(std::vector<std::vector<cv::Point> >& curContours,
                std::vector<std::vector<cv::Point> >& newContours);
		void getCanny(cv::Mat& image);
};
#endif		
