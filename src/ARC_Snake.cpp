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
#include "ARC_External.hpp"

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
    L = 16;
    N = 4;
    init_contour( center, contour, L, N );
    it = contour.begin();
}  /* -----  end of method ARC_Snake::ARC_Snake  (constructor)  ----- */

/* Copy constructor */
ARC_Snake::ARC_Snake ( const ARC_Snake & a )
{
    contour = a.contour;
    it=contour.begin();
} 

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
 *      Method:  ARC_Snake :: ccw
 * Description:  Returns >0 for CCW, 0 for collinear, <0 for CW
 *--------------------------------------------------------------------------------------
 */
    double
ARC_Snake::ccw ( Point a, Point b, Point c )
{
    Matx33f m;
    m = Matx33f( a.x, a.y, 1,
                 b.x, b.y, 1,
                 c.x, c.y, 1);
    return determinant(m);
}		/* -----  end of method ARC_Snake::ccw  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: intersections_test
 * Description:  Determines if line segments intersect by method described here:
 * http://algs4.cs.princeton.edu/91primitives/
 * Returns true if there is an intersection, false otherwise.
 * TODO: Bug, returns true if collinear.
 *--------------------------------------------------------------------------------------
 */
    bool
ARC_Snake::intersections_test ( Point p0, Point p1, Point p2, Point p3, Point p4 )
{
    // Check 12X34
    if( ccw( p1, p2, p3 ) * ccw( p1, p2, p4 ) > 0 ) return false;
    // Check 01X23
    if( ccw( p0, p1, p2 ) * ccw( p0, p1, p3 ) > 0 ) return false;
    return true;
    // TODO edge cases:
    // The test above ignores situations where three or more endpoints are
    // collinear. This includes the case if one or both of the line segments
    // degenerate to a single point. See figure XYZ for some possible cases.
    // Usesful subroutine is: given 3 collinear points a, b, and c, is b
    // between a and c? Need to handle vertical line-segment case separately.
    // Also need to worry about segments that degenerate to a single point.
    // Surprisingly difficult to get it right.
}		/* -----  end of method ARC_Snake::intersections_test  ----- */


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
    Point p0, p1, p2, p3, p4; // Previous and next
    Point ov;

    // Get the 2 previous points, the current point, and the next two points.
    prev_point();
    p0=get_prev_point();
    next_point();
    p1=get_prev_point();
    p2=get_point();
    p3=get_next_point();
    next_point();
    p4=get_next_point();
    prev_point();

    // Move the current point orthoganally.
    ov = get_orthvec();
    p2+=L*ov;
    // Check if there are any intersections.
    if( !intersections_test( p0, p1, p2, p3, p4 ) )
        *it=p2;

    return ;
}		/* -----  end of method ARC_Snake::expand  ----- */


    void
ARC_Snake::contract ( int L )
{
    Point p0, p1, p2, p3, p4; // Previous and next
    Point ov;

    // Get the 2 previous points, the current point, and the next two points.
    prev_point();
    p0=get_prev_point();
    next_point();
    p1=get_prev_point();
    p2=get_point();
    p3=get_next_point();
    next_point();
    p4=get_next_point();
    prev_point();

    // Move the current point orthoganally.
    ov = get_orthvec();
    p2-=L*ov;
    if( !intersections_test( p0, p1, p2, p3, p4 ) )
        *it=p2;
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
ARC_Snake::interpolate (int L )
{
    double diff;
    vector<Point> contour2N;
    //if( contour.size()<(size_t)(sqrt(contourArea(contour))/4) )
    if( 1 )
    {
        it = contour.begin();
        do
        {
            Vec2f cur, next, mid;
            cur = (Mat) get_point();
            next=(Mat) get_next_point();
            mid = 0.5*(cur+next);
            diff = norm( cur-next );
            if( diff> 2*L ) contour2N.push_back( (Point) cur ); 
            if( diff> 4*L ) contour2N.push_back( (Point) mid ); 
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


    double
ARC_Snake::energy ( Mat image )
{
    double elas;
    //double c;
    double dc;
    dc =diff_color( image, color );
    elas = elasticity();
    //c = measureCanny( image );
    //cout << "Canny: " << c << endl;
    //cout << "Area: " << area() << endl;
    //cout << "Elasticity: " << 0.001 * elas << endl;
    //cout << "DC: " << dc/pow(contourArea(contour),2) << endl;
    //return dc/pow(contourArea(contour),2);
    return 0.00001*dc + area() + 0.001*elas;
}		/* -----  end of method ARC_Snake::energy  ----- */

    double
ARC_Snake::area ( )
{
    double alpha;
    double beta;
    double area;
    alpha=-1e-5;
    beta=111e-7;
    beta=0;
    area=contourArea(contour);
    return 100*exp(alpha*area) + exp(beta*(area-30000));
}		/* -----  end of method ARC_Snake::area  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: diff_color
 * Description:  Simple energy test to seek areas similar to a target color.
 *--------------------------------------------------------------------------------------
 */
    double
ARC_Snake::diff_color ( Mat image, Scalar c )
{
    double d;
    Scalar sumS;
    Mat masked_contour;
    masked_contour = maskImage( image, c );
    //imshow("Contour Tracking", masked_contour );
    //waitKey(0);
    // subtract color from each element.
    subtract( c, masked_contour, masked_contour);
    
    sumS = sum(masked_contour);

    //if( verbosity==ARC_VERBOSE ) cout << "sum: " << sumS << endl;
    d = sumS[0] + sumS[1] + sumS[2];
    //imshow("Contour Tracking", masked_contour );
    //waitKey(0);

    // sum all elements.
    return d;
}		/* -----  end of function diff_color  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: maskImage
 * Description:  Masks an image with a contour.
 *--------------------------------------------------------------------------------------
 */
    Mat
ARC_Snake::maskImage ( Mat image, Scalar c )
{
    Mat mask, masked_contour ;
    vector<vector<Point> > contours;

    contours.push_back( contour );
    // mask the image with the contour.
    mask= Mat::zeros(image.size(), CV_8UC1 );
    drawContours(mask, contours, -1, Scalar(255,255,255), CV_FILLED );
    masked_contour = Mat( image.size(), CV_8UC3 );
    if( c!=Scalar(-1,-1,-1,-1) )
    {
        masked_contour.setTo(c);
    }
    image.copyTo(masked_contour, mask);
    return masked_contour;
}		/* -----  end of function maskImage  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: measureCanny
 * Description:  measures amount of canny edges.
 *--------------------------------------------------------------------------------------
 */
    double
ARC_Snake::measureCanny ( Mat image )
{
    Mat masked_contour, masked_bw;
    Mat full_hann, hann, windowed;
    Mat canny_mat, canny_mat_x, canny_mat_y;
    Mat abs_canny_mat_x, abs_canny_mat_y;
    Rect snake_rect, image_rect;
    Scalar canny_mean;
    vector<vector<Point> > contours;

    // Make a hanning window
    image_rect = Rect(Point(0,0), image.size());
    snake_rect = boundingRect( contour ) & image_rect;
    full_hann = Mat(image.size(), CV_32F, Scalar(0,0,0,0) );
    hann = full_hann( snake_rect );
    createHanningWindow( hann, snake_rect.size(), CV_32F );
    // Mask our contour
    masked_contour = maskImage( image, Scalar(0, 0, 0, 0) );
    cvtColor( masked_contour, masked_bw, CV_BGR2GRAY );

    // Window the mask
    multiply( masked_bw, full_hann, windowed, 1, CV_8U );
    Sobel( windowed, canny_mat_x, CV_16S, 1, 0, 3 );
    Sobel( windowed, canny_mat_y, CV_16S, 0, 1, 3 );

    convertScaleAbs( canny_mat_x, abs_canny_mat_x );
    convertScaleAbs( canny_mat_y, abs_canny_mat_y );
    canny_mat = 0.5 * abs_canny_mat_x + 0.5 * abs_canny_mat_y;
    Canny( windowed, canny_mat, 300, 400, 3 );
    //imshow("snake", canny_mat);
    //waitKey(0);
    contours.push_back(contour);
    drawContours(canny_mat, contours, 0, Scalar(0,0,0), 3 );
    canny_mean = mean(canny_mat, masked_bw);
    //if( verbosity==ARC_VERBOSE ) cout << "canny mean: " << canny_mean << endl;

    /*
    imshow("Contour Tracking", sobel_mat );
    waitKey(0);
    */
    return canny_mean[0];
}		/* -----  end of function measureSobel  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: elasticity
 * Description:  Calculate how much "external pressure" is on each point. For
 * each point in the contour, we calculate the difference between its distance
 * to the center of the contour and the average distance of its two nearest
 * neighbors to the center. This difference is squared and sum of the squares
 * is returned.
 *--------------------------------------------------------------------------------------
 */
    double
ARC_Snake::elasticity ( )
{
    vector<Point>::iterator it_saved;
    double E;
    Point cen;
    Point p, c, n; // Previous, current, next
    double dp, dc, dn; // Euclidean distance to center
    double mean_dist;

    // Save internal iterator so we can use functions that rely on them.
    it_saved = it; 
    it = contour.begin();
    E=0;
    // find center of contour
    cen = center();
    // Loop over each point
    while( it!=contour.end() )
    {
        p = get_prev_point();
        c = get_point();
        n = get_next_point();
        dp = norm(p-cen);
        dc = norm(c-cen);
        dn = norm(n-cen);
        mean_dist = 0.5 * (dp+dn);
        E += pow(abs(dc - mean_dist), 2);
        ++it;
    }
    it = it_saved;

    return E;
}		/* -----  end of method ARC_Snake::elasticity  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: center
 * Description:  Finds center of mass of the contour.
 *--------------------------------------------------------------------------------------
 */
    Point
ARC_Snake::center ( )
{
    Moments trackedMom;
    Point center;

    trackedMom = moments( contour, false );
    center = Point(trackedMom.m10/trackedMom.m00,trackedMom.m01/trackedMom.m00);
    return center;
}		/* -----  end of method ARC_Snake::center  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_Snake
 *      Method:  ARC_Snake :: set_color
 * Description:  Sets the color to the average inside the contour.
 *--------------------------------------------------------------------------------------
 */
    void
ARC_Snake::set_color ( Mat image )
{
    Mat mask;
    Scalar black;
    vector<vector<Point> > contours;
    black = Scalar( 0,0,0,0 );
    mask = Mat(image.size(), CV_8UC1, Scalar::all(0));
    contours.push_back(contour);
    drawContours(mask, contours, -1, Scalar(255,255,255), CV_FILLED );
    color = mean(image, mask);
    return ;
}		/* -----  end of method ARC_Snake::set_color  ----- */

    void
ARC_Snake::set_point ( Point p )
{
    *it=p;
}		/* -----  end of method ARC_External::set_point  ----- */

    void
ARC_Snake::polygonize ( )
{
    double eps;
    vector<Point> approx;
    eps=4;
    approxPolyDP( contour, approx, eps, true );
    contour=approx;
    return ;
}		/* -----  end of method ARC_External::polygonize  ----- */

