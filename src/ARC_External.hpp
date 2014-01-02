#ifndef  ARC_EXTERNAL_INC
#define  ARC_EXTERNAL_INC

#include <iostream>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include </usr/include/opencv/highgui.h>

using std::vector;
using std::cerr;
using std::cout;
using std::endl;
using namespace cv;


/*
 * =====================================================================================
 *        Class:  ARC_External
 *  Description:  calculate external energies.
 * =====================================================================================
 */
class ARC_External
{
    public:
        /* ====================  LIFECYCLE     ======================================= */
        ARC_External ( Mat img );                             /* constructor */

        /* ====================  ACCESSORS     ======================================= */
        void intensity();
        void edges();
        void term();
        void show_intensity();
        Point energy (Point s);

        /* ====================  MUTATORS      ======================================= */
        void set_wline( double w );
        void set_wedge( double w );
        void set_wterm( double w );
        void set_image( Mat img );

        /* ====================  OPERATORS     ======================================= */
        void init_matrices();


    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        Mat image;
        double wline, wedge, wterm;
        Mat gradx, grady, gradxx, gradxy, gradyy, cgradx, cgrady;

}; /* -----  end of class ARC_External  ----- */

#endif   /* ----- #ifndef ARC_EXTERNAL_INC  ----- */
