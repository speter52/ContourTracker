/*
 * =====================================================================================
 *
 *       Filename:  ContourTracker.cpp
 *
 *    Description:  Segments an image, tracks contours using contour matching
 *    and Farneback flows to estimate motion.
 *
 *        Version:  1.0
 *        Created:  10/17/2013 01:59:45 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Martin Miller (), miller7@illinois.edu
 *   Organization:  ARCL
 *
 * =====================================================================================
 */
#include "ContourTracker.hpp"

using namespace std;
using namespace cv;

int verbosity;


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  help
 *  Description:  
 * =====================================================================================
 */
    void
help ( char** argv )
{
    cout << "Usage: " << argv[0] << " <-l image list> [options]" << endl
         << endl
         << "OPTIONS" << endl
         << "-h" << tab << "Display this help." << endl
         << "-l <filename>" << tab << "Path to the image list." << endl
         << "-n <number>" << tab << "Number of contours to track." << endl
         << "-v" << tab << "Verbose output." << endl;
    exit( EXIT_FAILURE );
    return ;
}		/* -----  end of function help  ----- */
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  init
 *  Description:  Initializes image list, video, etc.
 * =====================================================================================
 */
bool init( int argc, char **argv, vector<string>& image_list,
        VideoWriter& vidout, int* num_contours, vector<Scalar>& colors )
{
    RNG rng;
    string listname;
    Mat first_frame;
    string vid_filename = "out.avi";
    *num_contours = ARC_NUM_CONTOURS_DEFAULT;

    if( argc==1 ) return false;
    for ( int i = 1; i < argc; i += 1 ) 
    {
        if( !strcmp(argv[i], ARC_HELP_FLAG) ) 
        {
            return false;
        }
        if( !strcmp(argv[i], ARC_IMAGELIST_NAME) ) 
        {
            listname = argv[++i];
            continue;
        }
        if( !strcmp(argv[i], ARC_NUM_CONTOURS) ) 
        {
            *num_contours = atoi(argv[++i]);
            continue;
        }
        if( !strcmp(argv[i], ARC_VERBOSE_FLAG) ) verbosity=ARC_VERBOSE; 
    }
    get_image_list( listname, image_list );

    // Init Video
    first_frame=imread( image_list[0], CV_LOAD_IMAGE_COLOR );
    vidout.open( vid_filename, CV_FOURCC('F','M','P','4'), 20.0, first_frame.size(), true );
    if( !vidout.isOpened() )
    {
        cerr << "Could not open video file: " << vid_filename << endl;
        exit( EXIT_FAILURE );
    }

    for( int i=0; i<(*num_contours); ++i )
    {
        Scalar c( rng.uniform(0, 255),
                  rng.uniform(0, 255),
                  rng.uniform(0, 255) );
        colors.push_back(c);
    }
    return true;
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  find_contours
 *  Description:  Finds N contours using chosen algorithm.
 * =====================================================================================
 */
int find_contours( Mat& image, vector<vector<Point> >& contours, int num_contours, int algorithm=0 )
{
    int num_good_con;
    Mat gray, canny;
    vector<vector<Point> > found_contours, hulls, poly;
    vector<Vec4i> hierarchy;

    // Convert to grayscale
    cvtColor( image, gray, CV_BGR2GRAY );

    /* Find some contours. */
    if( algorithm==ARC_CANNY_SIMPLE )
    {
        Canny( gray, canny, 100, 200, 3 );
        dilate( canny, canny, Mat(), Point( -1, -1 ) );
        findContours( canny, found_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
    }
    else
    {
        cerr << "Algorithm unsupported." << endl;
        exit( EXIT_FAILURE );
    }

    /* Filter the contours so that we get N desired. */
    num_good_con = 0;
    for( vector<vector<Point> >::iterator con=found_contours.begin();
            con!=found_contours.end(); ++con )
    {
        if( contourArea( *con )>ARC_MIN_AREA )
        {
            contours.push_back( *con );
            ++num_good_con;
        }
        if( num_good_con>=num_contours ) break;
    }
    return num_good_con;
}

void flow( Mat& prev_image, Mat& image, vector<vector<Point> >& contours )
{
    Mat gray, prev_gray, flow;

    cvtColor( image, gray, CV_BGR2GRAY );
    cvtColor( prev_image, prev_gray, CV_BGR2GRAY );

    calcOpticalFlowFarneback( prev_gray, gray, flow, 0.25, 5, 30, 5, 5, 1.2, 0 );

    for( vector<vector<Point> >::iterator con=contours.begin();
            con!=contours.end(); ++con )
    {
        int i=0;
        for( vector<Point>::iterator pt=con->begin();
                pt!=con->end(); ++pt, ++i )
        {
            Point2f delk;
            Point del;

            delk = flow.at<Point2f>(*pt);
            del = delk;                         /* converts Point2f to Point */

            *pt += del;
        }
    }


}

void match_contours( Mat& image, vector<Point>& contour )
{
    Rect image_rect, crect;
    Mat gray, canny, masked_gray;
    vector<Vec4i> hierarchy;
    vector<Point> contour_copy;
    vector<vector<Point> > found_contours;
    int c_area;
    double min_score;
    Size rect_margin;

    rect_margin = Size( ARC_RECT_MARGIN, ARC_RECT_MARGIN );
    cvtColor( image, gray, CV_BGR2GRAY );

    image_rect = Rect( Point(0,0), image.size() );

    crect = Rect( boundingRect( contour ) );
    crect += rect_margin;
    crect -= (Point) rect_margin*0.5;
    crect &= image_rect;

    masked_gray = Mat(gray,crect);
    Canny(masked_gray, canny, 100, 200, 3);
    //dilate( edges, edges, Mat(), Point(-1, -1) );
    findContours( canny, found_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, crect.tl() );

    contour_copy = contour;
    c_area = contourArea( contour );
    min_score = ARC_MIN_SCORE;

    for( vector<vector<Point> >::iterator fc=found_contours.begin();
            fc!=found_contours.end(); ++fc )
    {
        int fc_area ;
        double match_score, area_dif, score;

        fc_area = contourArea( *fc );

        area_dif = abs( fc_area-c_area )/(1+max( c_area, fc_area ));
        match_score = matchShapes( contour_copy, *fc, CV_CONTOURS_MATCH_I1, 0 );
        score = (1-ARC_ALPHA)*(1-ARC_ALPHA)*match_score*match_score+ARC_ALPHA*ARC_ALPHA*area_dif*area_dif;
        cout << "Score: " << score << endl;
        if( score<min_score && score!=0 )
        {
            contour = *fc;
            min_score = score;
        }
    }
}

void display_contours( Mat& image, vector<vector<Point> >& contours, VideoWriter& v, vector<Scalar>& colors )
{
    for( size_t i=0; i<contours.size(); ++i )
    {
        drawContours( image, contours, i, colors[i], 2, 8, noArray( ), 0, Point( ));
    }
    imshow( "ARC", image );
    v << image;
    waitKey( 3 );
}

void get_image_list(string filename, vector<string>& il)
{
 
    string    ifs_file_name = filename;         /* input  file name */
    ifstream  ifs;                              /* create ifstream object */

    ifs.open ( ifs_file_name.c_str() );         /* open ifstream */
    if (!ifs) {
        cerr << "\nERROR : failed to open input  file " << ifs_file_name << endl;
        exit (EXIT_FAILURE);
    }
    string line;
    while( getline(ifs, line, '\n') )
    {
        il.push_back(line);
    }
    ifs.close ();                                 /* close ifstream */
}

int main( int argc,  char **argv )
{
    VideoWriter vidout;
    vector<string> image_list;
    vector<vector<Point> > contours;
    int num_contours;
    int num_found_contours;
    Mat first_frame, image, prev_image;
    vector<Scalar> colors;

    namedWindow("ARC", CV_WINDOW_AUTOSIZE );

    if( !init( argc, argv, image_list, vidout, &num_contours, colors ) ) help( argv );

    if( verbosity==ARC_VERBOSE )
    {
        cout << "Num frames: " << image_list.size() << endl;
        cout << "Num contours: " << num_contours << endl;
    }

    /* Perform initial segmentation and extract desired contours. */
    first_frame = imread( image_list[0], CV_LOAD_IMAGE_UNCHANGED );
    num_found_contours = find_contours( first_frame, contours, num_contours );                 
    if( verbosity==ARC_VERBOSE )
    {
        /* Describe found contours. */
        cout << "Found " << num_found_contours << " contours." << endl;
        for( int i=0; i<num_found_contours; ++i )
        {
            drawContours( first_frame, contours, i, colors[i], 2, 8, noArray( ), 0, Point( ));
        }
        imshow( "ARC", first_frame );
        waitKey( 0 );
    }

    prev_image = first_frame;

    for( vector<string>::iterator im=image_list.begin();
            im!=image_list.end(); ++im )
    {
        Mat image_copy;
        image = imread( *im, CV_LOAD_IMAGE_UNCHANGED );
        image_copy = image.clone();

        flow( prev_image, image, contours );                                 /* Estimate position of contour using flow. */
        for( vector<vector<Point> >::iterator con=contours.begin();
                con!=contours.end(); ++con )
        {
            match_contours( image, *con );                   /* Match current contour to other contours in region. */
            if( verbosity==ARC_VERBOSE )
            {
                /* Describe result of contour matching. */
            }
        }
        display_contours( image_copy, contours, vidout, colors );                     /* Draw contours, write to video, etc. */
        prev_image = image;
    }
    return 0;
}
