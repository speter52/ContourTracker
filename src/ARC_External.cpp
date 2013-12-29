/*
 * =====================================================================================
 *
 *       Filename:  ARC_External.cpp
 *
 *    Description:  Calculate external energies.
 *
 *        Version:  1.0
 *        Created:  12/26/2013 10:08:00 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), witsquash@lavabit.com
 *   Organization:  
 *
 * =====================================================================================
 */
#include "ARC_External.hpp"

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_External
 *      Method:  ARC_External
 * Description:  constructor
 *--------------------------------------------------------------------------------------
 */
ARC_External::ARC_External ( Mat img )
{
    image = img;
    wline=0.01;
    wedge=0.01;
    wterm=0.01;
}  /* -----  end of method ARC_External::ARC_External  (constructor)  ----- */


    void
ARC_External::set_image ( Mat img )
{
    image = img;
    return ;
}		/* -----  end of method ARC_External::set_image  ----- */

    void
ARC_External::set_wline ( double w )
{
    wline = w;
    return ;
}		/* -----  end of method ARC_External::set_wline  ----- */

    void
ARC_External::set_wedge ( double w )
{
    wedge = w;
    return ;
}		/* -----  end of method ARC_External::set_wedge  ----- */

    void
ARC_External::set_wterm ( double w )
{
    wterm = w;
    return ;
}		/* -----  end of method ARC_External::set_wterm  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_External
 *      Method:  ARC_External :: line_energy
 * Description:  Calculates change in image intensity.
 * TODO: does this have to be done in grayscale?
 *--------------------------------------------------------------------------------------
 */
    void
ARC_External::line_energy ( Mat& gradx, Mat& grady )
{
    Mat gray, reverse;
    cvtColor( image, gray, CV_BGR2GRAY );
    // Convolve with gradient
    Sobel( gray, gradx, CV_16S, 1, 0, 3 );
    Sobel( gray, grady, CV_16S, 0, 1, 3 );
    // Reverse

    return;
}		/* -----  end of method ARC_External::line_energy  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_External
 *      Method:  ARC_External :: edge_energy
 * Description:  Semi-global edge detector.
 *--------------------------------------------------------------------------------------
 */
    void
ARC_External::edge_energy ( Mat& gradxx, Mat& gradxy, Mat& gradyy )
{
    Mat gray, smooth;
    Mat reverse(image.size(), CV_16SC1 );
    cvtColor( image, gray, CV_BGR2GRAY );
    // Smoothing
    GaussianBlur( gray, smooth, Size(7, 7), 0, 0 );
    // Convolve with gradient
    Sobel( smooth, gradxx, CV_16S, 2, 0, 3 );
    Sobel( smooth, gradxy, CV_16S, 1, 1, 3 );
    Sobel( smooth, gradyy, CV_16S, 0, 2, 3 );
    // Reverse
    gradxx=-1*gradxx;
    gradxy=-1*gradxy;
    gradyy=-1*gradyy;
    return ;
}		/* -----  end of method ARC_External::edge_energy  ----- */


    void
ARC_External::energy ( Mat& x, Mat& y )
{
    Mat gradx, grady, gradxx, gradxy, gradyy;
    line_energy( gradx, grady );
    edge_energy( gradxx, gradxy, gradyy );
    add(gradxx, gradx, x);
    add(gradyy, grady, y);
    return ;
}		/* -----  end of method ARC_External::energy  ----- */

