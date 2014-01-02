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
    init_matrices();
    wline=0.01;
    wedge=0.01;
    wterm=0.01;
}  /* -----  end of method ARC_External::ARC_External  (constructor)  ----- */


    void
ARC_External::set_image ( Mat img )
{
    image = img;
    init_matrices();
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

    void
ARC_External::init_matrices ( )
{
    intensity();
    edges();
    //term();
    return ;
}		/* -----  end of method ARC_External::init_matrices  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_External
 *      Method:  ARC_External :: line_energy
 * Description:  Calculates change in image intensity.
 * TODO: does this have to be done in grayscale?
 *--------------------------------------------------------------------------------------
 */
    void
ARC_External::intensity ( )
{
    Mat gray, reverse;
    cvtColor( image, gray, CV_BGR2GRAY );
    // Convolve with gradient
    Sobel( gray, gradx, CV_16S, 1, 0, 3 );
    Sobel( gray, grady, CV_16S, 0, 1, 3 );

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
ARC_External::edges( )
{
    Mat gray, smooth;
    Mat reverse(image.size(), CV_16SC1 );
    cvtColor( image, gray, CV_BGR2GRAY );
    // Smoothing
    GaussianBlur( gray, smooth, Size(5, 5), 0, 0 );
    // Convolve with gradient
    Sobel( smooth, gradxx, CV_16S, 2, 0, 3);
    Sobel( smooth, gradxy, CV_16S, 1, 1, 3);
    Sobel( smooth, gradyy, CV_16S, 0, 2, 3);
    return ;
}		/* -----  end of method ARC_External::edge_energy  ----- */

    void
ARC_External::term ( )
{
    Mat gray, C;
    Point n;

    cvtColor( image, gray, CV_BGR2GRAY );
    // Smoothing
    GaussianBlur( gray, C, Size(9, 9), 0, 0 );
    // Get x and y derivatives
    Sobel( C, cgradx, CV_16S, 1, 0, 3 );
    Sobel( C, cgrady, CV_16S, 0, 1, 3 );
    return ;
}		/* -----  end of method ARC_External::term  ----- */


    Point
ARC_External::energy ( Point s )
{
    Point external_energy;
    Matx21f line_vec, edge_vec, sum_vec;
    short dx, dy, dxx, dxy, dyy;
    dx= gradx.at<short>(s);
    dy= grady.at<short>(s);
    dxx=gradxx.at<short>(s);
    dyy=gradyy.at<short>(s);
    dxy=gradxy.at<short>(s);
    //cdx=cgradx.at<short>(s);
    //cdy=cgrady.at<short>(s);

    line_vec=Matx21f(dx, dy);
    Matx22f H( dxx, dxy, 
               dxy, dyy);
    edge_vec=H*line_vec;
    sum_vec=wline*line_vec -wedge*edge_vec;
    external_energy=Point((int)sum_vec(0,0), (int)sum_vec(1,0));

    
    return external_energy;
}		/* -----  end of method ARC_External::energy  ----- */

    void
ARC_External::show_intensity ( )
{
    cout<< gradx << endl;
    /*
    imshow("snake", abs(gradx));
    waitKey(0);
    imshow("snake", abs(grady));
    waitKey(0);
    */
    return ;
}		/* -----  end of method ARC_External::show_intensity  ----- */

