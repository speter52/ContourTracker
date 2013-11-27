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
ARC_Snake::ARC_Snake ( vector<Point>& con )
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
 *      Method:  ARC_Snake :: prev_point
 * Description:  Moves iterator to previous point. Returns false when iterator
 * loops back to end.
 *--------------------------------------------------------------------------------------
 */
    bool
ARC_Snake::prev_point ( )
{
    if( it!=contour.begin() )
    {
        --it;
        return true;
    }
    it=contour.end()-1;
    return false;
}		/* -----  end of method ARC_Snake::prev_point  ----- */


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
    if( it==contour.end()-1 )
    {
        n = *(contour.begin());
    }
    else
    {
        n = *(it+1);
    }
    return n;
}		/* -----  end of method ARC_Snake::get_next_point  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: get_prev_point
 * Description:  Returns the point before the current point.
 *--------------------------------------------------------------------------------------
 */
    Point
ARC_Snake::get_prev_point ( )
{
    Point p;
    if( it==contour.begin() )
    {
        p = *(contour.end()-1);
    }
    else
    {
        p = *(it-1);
    }
    return p;
}		/* -----  end of method ARC_Snake::get_prev_point  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: get_contour
 * Description:  Returns contour as a vector<vector<Point> >
 *--------------------------------------------------------------------------------------
 */
    void
ARC_Snake::get_contour( vector<vector<Point> >& cons )
{
    cons.push_back( contour );
    return;
}


    Point
ARC_Snake::get_orthvec ( )
{
    Vec2f n, p, diff, orthvec;

    // Rotation matrix
    Matx22f rot90(0, -1, 
                  1, 0);

    n = (Mat) get_next_point();
    p = (Mat) get_prev_point();
    diff = p-n;
    orthvec = rot90 * diff;
    orthvec = orthvec/norm(orthvec);

    return (Point) orthvec ;
}		/* -----  end of method ARC_Snake::get_orthvec  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: expand
 * Description:  Expands the current point by L.
 *--------------------------------------------------------------------------------------
 */
    void
ARC_Snake::expand ( int L )
{
    Point ov;
    ov = get_orthvec();
    *it+=L*ov;

    return ;
}		/* -----  end of method ARC_Snake::expand  ----- */


    void
ARC_Snake::contract ( int L )
{
    Point ov;
    ov = get_orthvec();
    *it-=L*ov;
    return ;
}		/* -----  end of method ARC_Snake::contract  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: interpolate
 * Description:  Doubles the number of points in the contour, by adding points
 * at the midpoint between adjacent points. Checks if the contour is large
 * enough before performing the interpolation. If the contour is too small, -1
 * is returned. Otherwise the number of points in returned.
 * Also, will not create a  new point if the distance between existing adjacent
 * points is too small.
 *--------------------------------------------------------------------------------------
 */
    int
ARC_Snake::interpolate ( )
{
    double diff;
    vector<Point> contour2N;
    if( 2*contour.size()<(size_t)(sqrt(contourArea(contour))/4) )
    {
        it = contour.begin();
        do
        {
            Vec2f cur, next, mid;
            cur = (Mat) get_point();
            next=(Mat) get_next_point();
            mid = 0.5*(cur+next);
            diff = norm( cur-next );
            contour2N.push_back( (Point) cur );
            if( diff> 20 ) contour2N.push_back( (Point) mid );
        }
        while( next_point() );

        contour = contour2N;
        // Reset iterator
        it=contour.begin();
        return (int) contour.size();
    }
    else
    {
        return -1 ;
    }
}		/* -----  end of method ARC_Snake::interpolate  ----- */

