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

#include <iostream>
#include <cv.h>
#include <opencv2/core/core.hpp>

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
template < class T >
class ARC_Snake
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ARC_Snake ( vector<Point>& con);                             /* constructor */
        ARC_Snake ( Point center );             /* constructor */

        /* ====================  ACCESSORS     ======================================= */
        bool next_point();
        Point get_next_point();
        Point get_prev_point();
        Point expand();
        Point contract();

        /* ====================  MUTATORS      ======================================= */

        /* ====================  OPERATORS     ======================================= */
        double energy( Mat img );
        int area()
        {
            return contourArea(contour);
        }


    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */
        void init_contour( Point center, vector<Point>& snake, int L, int N );

        /* ====================  DATA MEMBERS  ======================================= */
        vector<Point> contour;
        vector<Point>::iterator it;

}; /* ----------  end of template class Snake  ---------- */
