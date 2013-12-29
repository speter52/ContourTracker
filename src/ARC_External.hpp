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
        void line_energy(Mat& gradx, Mat& grady);
        void edge_energy(Mat& gradxx, Mat& gradxy, Mat& gradyy);
        void term_energy();
        void energy (Mat& x, Mat& y);

        /* ====================  MUTATORS      ======================================= */
        void set_wline( double w );
        void set_wedge( double w );
        void set_wterm( double w );
        void set_image( Mat img );

        /* ====================  OPERATORS     ======================================= */


    protected:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */

    private:
        /* ====================  METHODS       ======================================= */

        /* ====================  DATA MEMBERS  ======================================= */
        Mat image;
        double wline, wedge, wterm;

}; /* -----  end of class ARC_External  ----- */

#endif   /* ----- #ifndef ARC_EXTERNAL_INC  ----- */
