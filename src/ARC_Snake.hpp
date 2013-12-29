/*
 * =====================================================================================
 *
 *       Filename:  ARC_Snake.cpp
 *
 *    Description:  Snake Class. Replaces Contour class. Stores a snake,
 *    measures energy. Easy access to incrementing the point of inflection of
 *    the snake. Easy access to expanding and contracting the snake.
 *
 *        Version:  1.0
 *        Created:  11/26/2013 06:35:17 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), 
 *   Organization:  Aerospace Robotics and Control Lab
 *
 * =====================================================================================
 */

#ifndef  ARC_Snake_INC
#define  ARC_Snake_INC

#include <iostream>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>

using std::vector;
using std::cerr;
using std::cout;
using std::endl;
using namespace cv;

/*
 * =====================================================================================
 *        Class:  ARC_Snake
 *  Description:  Class for storing a snake contour and measuring its energy.
 * =====================================================================================
 */
class ARC_Snake
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ARC_Snake ( vector<Point>& con);                             /* constructor */
        ARC_Snake ( Point center );             /* constructor */
        ARC_Snake ( const ARC_Snake & a );             /* constructor */
        

        /* ====================  ACCESSORS     ======================================= */
        bool next_point();
        bool prev_point();
        Point get_point()
        {
            return *it;
        }
        void get_contour( vector<vector<Point> >& cons );

        /* ====================  MUTATORS      ======================================= */
        void set_color( Mat image );
        void set_canny( Mat image ) { image_canny = image; }
        void expand( int L );
        void contract( int L);
        int interpolate(int L);
        void set_point( Point p );
        void polygonize();

        /* ====================  OPERATORS     ======================================= */
        double energy( Mat img, vector<Point>& prev );
        double area();
        double elasticity();
        Point center();

		/* ==== METHODS === */
		double getMahalanobis(vector<Point> contour);


    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */
        Point get_orthvec();
        Point get_next_point();
        Point get_prev_point();
        void init_contour( Point center, vector<Point>& snake, int L, int N );
        double diff_color ( Mat image, Scalar c );
        Mat maskImage ( Mat image, Scalar c );
        double measureCanny ( );
        bool intersections_test( Point p0, Point p1, Point p2, Point p3, Point p4 );
        double ccw( Point a, Point b, Point c );

        /* ====================  DATA MEMBERS  ======================================= */
        Mat image_canny;
        vector<Point> contour;
        vector<Point>::iterator it;
        Scalar color;

}; /* ----------  end of template class Snake  ---------- */
#endif   /* ----- #ifndef ARC_Snake_INC  ----- */
