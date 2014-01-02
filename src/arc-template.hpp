#ifndef  arc_template_INC
#define  arc_template_INC
#include <cv.h>
#include </usr/include/opencv2/opencv.hpp>
#include </usr/include/opencv/highgui.h>
#include <vector>
#include <opencv/highgui.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp/vpTemplateTrackerWarpHomography.h>
#define ARC_MIN_CONTOURS 3            /* Will add more contours if number tracked falls below */
#define ARC_WIDTH 640            /*  */
#define ARC_HEIGHT 480            /*  */
void getImageList( std::string filename,  std::vector<std::string>* il );
unsigned next_index ( );
#endif   /* ----- #ifndef arc-template_INC  ----- */
