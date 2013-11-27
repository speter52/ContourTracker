/*
 * =====================================================================================
 *
 *       Filename:  ARC_Snake.cpp
 *
 *    Description:  code for ARC_Snale
 *
 *        Version:  1.0
 *        Created:  11/26/2013 06:52:30 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), 
 *   Organization:  Aerospace Robotics and Control Lab
 *
 * =====================================================================================
 */


#include "ARC_Snake.hpp"

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake
 * Description:  constructor for existing contour
 *--------------------------------------------------------------------------------------
 */
ARC_Snake::ARC_Snake ( vector<Point> con )
{
    contour=con;
    it = contour.begin();
}  /* -----  end of method ARC_Snake::ARC_Snake  (constructor)  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake
 * Description:  constructor for point only
 *--------------------------------------------------------------------------------------
 */
ARC_Snake::ARC_Snake ( Point center )
{
    int L, N;
    L = 8;
    N = 4;
    init_contour( center, contour, L, N );
    it = contour.begin();
}  /* -----  end of method ARC_Snake::ARC_Snake  (constructor)  ----- */


    void
ARC_Snake::init_contour ( Point center, vector<Point>& snake, int L, int N )
{
    Mat mags, angles, x, y;
    double dw;

    dw = 2* M_PI/N; 
    mags = L * Mat::ones(N,1, CV_32F);

    angles = Mat::zeros(N,1, CV_32F);
    for( int i=0; i<N; ++i ) angles.at<float>(i,0) = dw*i;
    polarToCart( mags, angles, x, y, false );
    for( int i=0; i<N; ++i )
    {
        Point newpt(x.at<float>(i,0), y.at<float>(i,0));
        snake.push_back( center + newpt );
    }
    
    return;
}		/* -----  end of method ARC_Snake::init_contour  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: next_point
 * Description:  Moves iterator to next point. Returns false when iterator
 * loops back to beginning.
 *--------------------------------------------------------------------------------------
 */
    bool
ARC_Snake::next_point ( )
{
    ++it;
    if( it!=contour.end() ) return true;
    it=contour.begin();
    return false;
}		/* -----  end of method ARC_Snake::next_point  ----- */



/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: get_next_point
 * Description:  Returns next point.
 *--------------------------------------------------------------------------------------
 */
    Point
ARC_Snake::get_next_point ( )
{
    Point n;
    n = *(
    return ;
}		/* -----  end of method ARC_Snake::get_next_point  ----- */


int main()
{
	using namespace std;
	return 0;
}

