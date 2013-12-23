//Simon Peter
//Contour Tracker
//7-10-14
#include "ContourTracker.hpp"
#include "ARC_Snake.hpp"

//#define ARC_DEBUG            /*  */
using namespace std;
using namespace cv;

/* 
 * ===  FUNCTION  ========================================================================
 *         Name: getUserPoints 
 *  Description:  Asks user to select points and stores it in the vector that's passed in.
 * =======================================================================================
 */
void getUserPoints(Mat image, vector<Point> & points)
{
    cout << "Select center of contour with mouse. Press any key when finished." << endl;
    namedWindow("Select Points - Then Press Any Key",CV_WINDOW_AUTOSIZE);
    setMouseCallback("Select Points - Then Press Any Key", getPointsFromMouse, &points);
    imshow("Select Points - Then Press Any Key",image);
    waitKey(0);
    destroyWindow("Select Points - Then Press Any Key");
}

/* 
 * ===  FUNCTION  ========================================================================
 *         Name: onMouse 
 *  Description:  Mouse callback function that retrieves user selected points.
 * =======================================================================================
 */
void getPointsFromMouse(int event, int x, int y, int flags, void* points)
{
    if(event==EVENT_LBUTTONDOWN)
    {
        cout<<"Point added";
        vector<Point>* userpts = (vector<Point>*)(points);        
        userpts->push_back(Point(x,y));
    }
}

#ifndef ARC_DEBUG
// GLOBALS
int areaThreshold = ARC_DEFAULT_AREA; //used 2000 and .13 for match
double distanceThreshold = ARC_DEFAULT_DISTANCE;
double mahalanobisThreshold = ARC_DEFAULT_MAHALANOBOIS;
int miscountThreshold = ARC_DEFAULT_MISCOUNT; //TODO: decide on a threshold to track between frames
int verbosity;


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  measureSobel
 *  Description:  
 * =====================================================================================
 */
    double
measureSobel ( Mat image, vector<Point>& snake )
{
    Mat masked_contour, masked_bw;
    Mat full_hann, hann, windowed;
    Mat sobel_mat, sobel_mat_x, sobel_mat_y;
    Mat abs_sobel_mat_x, abs_sobel_mat_y;
    Rect snake_rect, image_rect;
    Scalar sobel_mean;
    vector<vector<Point> > contours;

    // Make a hanning window
    image_rect = Rect(Point(0,0), image.size());
    snake_rect = boundingRect( snake ) & image_rect;
    full_hann = Mat(image.size(), CV_32F, Scalar(0,0,0,0) );
    hann = full_hann( snake_rect );
    createHanningWindow( hann, snake_rect.size(), CV_32F );
    // Mask our contour
    masked_contour = maskImage( image, snake, Scalar(0, 0, 0, 0) );
    cvtColor( masked_contour, masked_bw, CV_BGR2GRAY );

    // Window the mask
    multiply( masked_bw, full_hann, windowed, 1, CV_8U );
    Sobel( windowed, sobel_mat_x, CV_16S, 1, 0, 3 );
    Sobel( windowed, sobel_mat_y, CV_16S, 0, 1, 3 );

    convertScaleAbs( sobel_mat_x, abs_sobel_mat_x );
    convertScaleAbs( sobel_mat_y, abs_sobel_mat_y );
    sobel_mat = 0.5 * abs_sobel_mat_x + 0.5 * abs_sobel_mat_y;
    Canny( windowed, sobel_mat, 100, 200, 3 );
    contours.push_back(snake);
    drawContours(sobel_mat, contours, 0, Scalar(0,0,0), 3 );
    sobel_mean = mean(sobel_mat, masked_bw);
    if( verbosity==ARC_VERBOSE ) cout << "Sobel mean: " << sobel_mean << endl;

    /*
    imshow("Contour Tracking", sobel_mat );
    waitKey(0);
    */
    return sobel_mean[0];
}		/* -----  end of function measureSobel  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  maskImage
 *  Description:  
 * =====================================================================================
 */
    Mat
maskImage ( Mat image, vector<Point>& snake, Scalar c )
{
    Mat mask, masked_contour ;
    vector<vector<Point> > contours;

    contours.push_back( snake );
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
 * ===  FUNCTION  ======================================================================
 *         Name:  setup_contour
 *  Description:  Creates an N point contour of radius L around the center.
 * =====================================================================================
 */
    void
setup_contour ( vector<Point>& snake, Point center, int N, int L )
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
}		/* -----  end of function setup_contour  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  minimize_energy
 *  Description:  Seeks a contour that minimizes energy. Seeks to maximize area
 *  of green contours.
 *  =====================================================================================
 */
    void
minimize_energy ( Mat image, Point center, vector<Point>& contour )
{
    Mat image_copy;
    Scalar green;
    Point dc;
    //Moments trackedMom;
    bool converged;
    int area;
    double E;
    int N, L;
    int iter;
    //size_t maxN;
    vector<Point> snake;

    converged=false;
    E=10000;
    //maxN=32;
    N=4; /* initial number of points in contour */
    L=4; /* magnitude of increment in pixels */
    green = Scalar(0, 255, 0); /* target color */

    //trackedMom = moments( contour, false );

    // setup initial N point contour
    setup_contour(snake, center, N, 4*L);

    iter=0;
    while( converged==false && iter<3 )
    {
        ++iter;
        converged=true;
        vector<vector<Point> > contours;
        vector<Point> snake_copy;
        snake_copy = snake;
        Matx22f rot90(0, -1, 
                      1, 0);
        vector<Point>::iterator pt_cpy=snake_copy.begin();
        for( vector<Point>::iterator pt=snake.begin();
                pt!=snake.end(); ++pt, ++pt_cpy )
        {
            //Moments newMom;
            //double mahalanobois;

            double Enew;
            double sobel;

            Point old = *pt;

            // move point normal to contour
            Vec2f prev, next, diff, orthvec;
            if( pt==snake.begin() )
            {
                prev = (Mat)*( snake_copy.end()-1 );
            }
            else
            {
                prev = (Mat)*(pt_cpy-1);
            }
            if( pt==snake.end()-1 )
            {
                next = (Mat)*( snake_copy.begin() );
            }
            else
            {
                next = (Mat)*(pt_cpy+1);
            }

            diff = prev - next;
            orthvec = rot90 * diff;
            // norm the vector then multiply it by target magnitude.
            orthvec = L * orthvec/norm(orthvec);

            // Update point
            *pt=*pt+(Point) orthvec;
            // measure energy
            area = contourArea( snake );
            sobel = measureSobel( image, snake );

            //newMom = moments( snake, false );
            //mahalanobois = huMomentsTest( trackedMom, newMom );
            /*
            double mahalmult = 2;
            double sobelmult = 1;
            double areamult = 4096;
            cout << "Mahal dist: " << mahalmult*mahalanobois << endl;
            cout << "Sobel: " << sobelmult*sobel << endl;
            cout << "area: " << area << endl;
            */


            Enew = pow(sobel,1)/pow(area,1);
            //Enew = areamult*(1.0/area) + sobelmult*sobel + mahalmult*mahalanobois;
            cout << "Enew: " << Enew << " Eold: " << E << endl;
            if( Enew>E )
            {
                *pt = old;
            }
            else
            {
                E = Enew;
                converged=false;
            }
            contours.push_back(snake);
        }
        // interpolate new points up to maxN

        cout << "Sz: " << 2*snake.size() << endl;
        if( 2*snake.size()<(size_t)(sqrt(contourArea(snake))/4) )
        {
            vector<Point> snake2N ( 2*snake.size() );
            int i=0;
            for( vector<Point>::iterator pt=snake.begin();
                    pt!=snake.end(); ++i, ++pt )
            {
                Vec2f cur, next, mid;
                cur = (Mat) *pt;
                if( pt==snake.end() -1 )
                {
                    next=(Mat) *snake.begin();
                }
                else
                {
                    next=(Mat) *(pt+1);
                }
                mid = 0.5*(cur+next);
                snake2N[i] = *pt;
                snake2N[++i] = (Point) mid;
            }
            snake = snake2N;
        }
    }
    //image_copy = image.clone();
    //drawContours( image_copy, contours, 0, Scalar(128,0,255), 2, 8, noArray( ), 0, Point( ));
    //imshow("Contour Tracking", image_copy);
    //waitKey(0);
    contour = snake;
    return ;
}		/* -----  end of function minimize_energy  ----- */

//Converts vector<Contour> to vector<vector<Point> > which can then be passed
//into functions like drawContours
void objectToContours( vector<Contour> *contours,  vector<vector<Point> >
        *vectors )
{
	for( size_t	i=0; i<contours->size( ); ++i )
    {
		vectors->push_back( (*contours)[i].contour );
	}
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getImageList
 *  Description:  Reads in list of image filenames.
 * =====================================================================================
 */
void getImageList( string filename,  vector<string>* il )
{

    string    ifs_file_name = filename;         /* input  file name */
    ifstream  ifs;                              /* create ifstream object */

    ifs.open (  ifs_file_name.c_str( ) );         /* open ifstream */
    if ( !ifs ) {
        cerr << "\nERROR : failed to open input  file " << ifs_file_name << endl;
        exit ( EXIT_FAILURE );
    }
    string line;
    while(  getline( ifs,  line,  '\n') )
    {
        il->push_back( line );
    }
    ifs.close ( );                                 /* close ifstream */
}


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  displayContours
 *  Description:  draws contours, writes to video, etc
 * =====================================================================================
 */
    void
displayContours ( Mat image, vector<ARC_Snake> snakes, VideoWriter vidout, vector<Scalar> colors )
{
    //Draw the tracked contours for that frame
    for(  size_t i=0; i<snakes.size( ); i++ )
    {
        Point cp;
        vector<vector<Point> >  temp;
        snakes[i].get_contour( temp );
        cp = snakes[i].get_point();

        //if( i<1 )	
        {
            drawContours( image, temp, 0, colors[i], 1, 8, noArray( ), 0, Point( ));
            circle( image, cp, 2, Scalar(255, 0, 0), 3 );
        }
    }

    namedWindow( "Contour Tracking", CV_WINDOW_AUTOSIZE );
    imshow( "Contour Tracking", image );
    waitKey( 25 );
    vidout << image;
    return;
}		/* -----  end of function displayContours  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  matchContours
 *  Description:  Attempts to replace a contour with a newer, matching contour.
 * =====================================================================================
 */
    void
matchContours ( Mat image, Contour& con, vector<Contour>& newContours )
{
    for( vector<Contour>::iterator newcon=newContours.begin();
     newcon!=newContours.end(); ++newcon )
    {
        Moments trackedMom;
        Moments newMom;
        double distance, mahalanobis;

        Mat image3 = image.clone();
        Rect newRect= boundingRect(newcon->contour);
        Mat newImage = image3(newRect);

        trackedMom = moments(con.contour,false);
        newMom = moments(newcon->contour,false);

        //Centroid Test
        distance = centroidTest( trackedMom, newMom );

        //Hu Moments Test - TODO: Test still needs to be implemented	
        mahalanobis = huMomentsTest( trackedMom, newMom );
    
        //Shapes test - Uses moments to compare the actual shape of two contours
        //TODO: Get matchShapes working
        double matchReturn = matchShapes( con.contour, newcon->contour, 
                CV_CONTOURS_MATCH_I1, 0 ); 
        if( verbosity==ARC_VERBOSE )
        {
            //cout << "matchShapes return: " << m << " " << n << " " << matchReturn << endl;
            cout << "matchShapes return: " << matchReturn << endl;
        }
        
        //Area test - Compares the areas between two contours
        int trackedArea = contourArea( con.contour );
        int newArea = contourArea( newcon->contour );
        int areaDifference = abs( trackedArea-newArea );
        if( verbosity==ARC_VERBOSE )
        {
            cout << "Area difference: " << areaDifference << endl; 
            cout << "images created\n";
        }

        // If the contour passes all the tests,  it is added to the
        // tracked vector and taken out of the newContours vector
        if( mahalanobis<mahalanobisThreshold
            //matchReturn<=matchThreshold 
//                   && isNear==true 
               && areaDifference<=areaThreshold 
//                   && compare<compareThreshold
                && distance<distanceThreshold
          )
        {
            con.contour=newcon->contour;
            con.nomatch = 0; //set nomatch to 0,  since it was found in that frame
            if( verbosity==ARC_VERBOSE ) cout << "Match found\n";
            newContours.erase( newcon );
            if( verbosity==ARC_VERBOSE ) cout << "New contours modified size: " << newContours.size( )<<endl;
            //after found,  it can stop trying to match that
            //particular tracked contour and move on to match the
            //next tracked contour
            break; 
        }
    }
    return;
}		/* -----  end of function matchContours  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getContours
 *  Description:  Finds good contours in given frame, adds them to the list of contours.
 * =====================================================================================
 */
void getContours( Mat image,  vector<ARC_Snake>  *snakes )
{
    Mat imageGray, cannyImage;
    vector<vector<Point> > foundContours;
    vector<Vec4i> hierarchy;

    cvtColor( image,  imageGray, CV_BGR2GRAY );
    //blur( imageGray, imageGray,  Size( 3, 3 ));
    Canny( imageGray, cannyImage, 100, 200, 3 );

	dilate( cannyImage, cannyImage, Mat(), Point(-1,-1) );
    //Finds initial contours
    findContours( cannyImage, foundContours, hierarchy, CV_RETR_TREE, 
            CV_CHAIN_APPROX_SIMPLE,  Point( 0, 0 ));

    for( vector<vector<Point> >::iterator con=foundContours.begin();
            con!=foundContours.end(); ++con )
    {
        if( contourArea(*con)>ARC_MIN_AREA )
        {
            Moments trackedMom;
            Point center;

            trackedMom = moments( *con, false );
            center = Point(trackedMom.m10/trackedMom.m00,trackedMom.m01/trackedMom.m00);
            snakes->push_back( ARC_Snake(center) );
        }
    }
}

void flow( Mat prev_image, Mat image, vector<Contour>& tracked )
{
    Mat gray, prev_gray, flow;

    cvtColor( image, gray, CV_BGR2GRAY );
    cvtColor( prev_image, prev_gray, CV_BGR2GRAY );

    calcOpticalFlowFarneback( prev_gray, gray, flow, 0.25, 5, 30, 5, 5, 1.2, 0 );

    int i=0;
    for( vector<Contour>::iterator tr=tracked.begin();
            tr!=tracked.end(); ++i, ++tr )
    {
        vector<Point> con, pd;
        con = tr->contour;
        pd = tr->prevdel;

        int j=0;
        for( vector<Point>::iterator pt=con.begin();
                pt!=con.end(); ++pt, ++j )
        {
            Point2f delk = flow.at<Point2f>(*pt);
            if( abs(delk.x)>640 || abs(delk.y)>480 ) break;
            Point del = delk;                         // converts Point2f to Point
            if( verbosity==ARC_VERBOSE )
            {
                cout << "Delk: " << delk << endl;
                cout << "Del: " << del << endl;
                cout << pd[j] << endl;
            }

            if( pd[j]==Point(0, 0) )
            {
                pd[j] = del;
            }
            else
            {
                Point del2 = ARC_LAMBDA*pd[j] + (1-ARC_LAMBDA)*del;
                pd[j] = del2;
            }
            *pt += pd[j];
        }
        tr->prevdel = pd;
        tr->contour = con;
    }
}

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  help
 *  Description:  Prints usage information.
 * =====================================================================================
 */
    void
help ( char **argv )
{
    cout << "Usage: " << argv[0] << " <-l image list> [options]" << endl
         << endl
         << "OPTIONS" << endl
         << "-h" << tab << "Display this help." << endl
         << "-l <filename>" << tab << "Path to the image list." << endl
         << "-o <filename>" << tab << "Output video name." << endl
         << "-d <double>" << tab << "Maximum distance between like contours." << endl
         << "-a <integer>" << tab << "Maximum area difference between like contours." << endl
         << "-m <double>" << tab << "Maximum mahalanobois difference between like contours." << endl
         << "-mc <integer>" << tab << "Maximum untracked frames before contour is lost." << endl
         << "-v" << tab << "Verbose output." << endl;
    return;
}		/* -----  end of function help  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  init
 *  Description:  Initializes video, parses arguments, etc
 * =====================================================================================
 */
    void
init ( int argc, char **argv, vector<string>& images, VideoWriter& vidout, vector<Scalar>& colors )
{
    RNG rng;
    string listname;
    Mat first_frame;
	String videoName =  ARC_DEFAULT_VIDOUT;

    // Parse input arguments
    if( argc==1 ) 
    {
        help(argv);
        exit( EXIT_FAILURE );
    }
    for ( int i = 1; i < argc; i += 1 ) 
    {
        if( !strcmp(argv[i], ARC_HELP_FLAG) ) 
        {
            help( argv );
            exit( EXIT_SUCCESS );
        }
        if( !strcmp(argv[i], ARC_IMAGELIST_NAME) ) 
        {
            listname = argv[++i];
            continue;
        }
        if( !strcmp(argv[i], ARC_VIDOUT) ) 
        {
            videoName = argv[++i];
            continue;
        }
        if( !strcmp(argv[i], ARC_VERBOSE_FLAG) ) verbosity=ARC_VERBOSE; 
    }

    // Prepare images
	getImageList( listname, &images );

    // Setup video
	Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
	vidout.open( videoName, CV_FOURCC( 'F', 'M', 'P', '4'), 20.0, firstFrame.size( ), true );
    if( !vidout.isOpened() )
    {
        cerr << "Could not open video file: " << videoName << endl;
        exit( EXIT_FAILURE );
    }

    for( int i=0; i<ARC_NUM_COLORS; ++i )
    {
        Scalar c( rng.uniform(0, 255),
                  rng.uniform(0, 255),
                  rng.uniform(0, 255) );
        colors.push_back(c);
    }
    return ;
}		/* -----  end of function init  ----- */


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  huMomentsTest
 *  Description:  Calculate the mahalanobois distance.
 * =====================================================================================
 */
    double
huMomentsTest ( Moments& trackedMom, Moments& newMom )
{
    double mahalanobis;
    double trackedHu[7];
    double newHu[7];
    double xsum;
    double xmean;

    Matx<double,7,1> xminusy, xminusMean;
    Matx<double,7,7> S, Sinverse;
    Matx<double,1,1> finalProduct;

    HuMoments(trackedMom,trackedHu);//Calculates Hu Moments for tracked contour
    HuMoments(newMom,newHu);//Calculates Hu Moments for new contour to be matched
    
    //Vector that represents (x-y) where x is the tracked Hu Moments and y is the new contour Hu Moments
    for(int g=0;g<7;g++){
        xminusy(0,g) = trackedHu[g]-newHu[g];
    }
    
    //Calculates mean of x vector
    int g;
    for( xsum=0, g=0; g<7; g++ )
    {
        xsum+=trackedHu[g];
    }
    xmean = xsum/7.0;

    for(g=0; g<7; g++ )
    {
        xminusMean(g,0) = trackedHu[g]-xmean;
    }
    //Calculate the S matrix by multipying the last vector by it's transpose, then dividing by 7
    S = (xminusMean*xminusMean.t())*(1.0/7.0);
    
    invert(S,Sinverse,DECOMP_SVD);	
    finalProduct = xminusy.t()*Sinverse*xminusy;
    mahalanobis = sqrt(finalProduct(0,0));//Square root the final product to get the mahalanobis distance

    if( verbosity==ARC_VERBOSE ) cout<<"Mahalanobis Return: "<<mahalanobis<<endl;	

    return mahalanobis;
}		/* -----  end of function huMomentsTest  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  centroidTest
 *  Description:  Measures distance between centers of two contours.
 * =====================================================================================
 */
    double
centroidTest ( Moments& trackedMom, Moments& newMom )
{
    double distance;

    Point trackedCenter(trackedMom.m10/trackedMom.m00,trackedMom.m01/trackedMom.m00);
    Point newCenter(newMom.m10/newMom.m00,newMom.m01/newMom.m00);

    // L2 norm of distance between centers.
    distance = norm(newCenter-trackedCenter);
    return distance;
}		/* -----  end of function centroidTest  ----- */

int main( int argc,  char **argv )
{
    Mat prev_image;
    vector<Scalar> colors;
	vector<string> images;
	VideoWriter vidout;
    vector<Point> user_points;
	vector<ARC_Snake> snakes; //vector to store contours that are tracked between frames

    init( argc, argv, images, vidout, colors );

	//Find initial contours to track on first image.
    Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
    getUserPoints( firstFrame, user_points );
	//getContours( firstFrame, &snakes ); //finds contours and stores them in tracked
    for( vector<Point>::iterator it=user_points.begin();
            it!=user_points.end(); ++it )
    {
        snakes.push_back( ARC_Snake(*it) );
    }
    cout << "size tracked: " << snakes.size() << endl;
	for( size_t i=0; i<1; i++ )
    {
		vector<vector<Point> > temp;
		snakes[i].get_contour( temp );
		drawContours( firstFrame, temp, 0, colors[i], 1, 8, noArray( ), 0, Point( )); //draws first image
	}
    if( verbosity==ARC_VERBOSE )
    {
        cout << "Original tracked size: " << snakes.size( ) <<endl;
        namedWindow( "Contour Tracking", CV_WINDOW_AUTOSIZE );
    }
	imshow( "Contour Tracking", firstFrame );
	waitKey( 0 ); 	
	vidout << firstFrame;
    prev_image=firstFrame.clone();

	//Begin image loop
    for( vector<String>::iterator im=images.begin();
            im!=images.end(); ++im )
    {
        Mat image, image2;
        image = imread( *im, CV_LOAD_IMAGE_UNCHANGED );
        image2 = image.clone();

        if(  !image.data )
        {
            cout << "Cannot load image." << endl;
            exit(  EXIT_FAILURE );
        }
        for( vector<ARC_Snake>::iterator snake=snakes.begin();
                snake!=snakes.end(); ++snake )
        {
            double energy, expand_energy; //, contract_energy;
            bool converged;
            unsigned int iter;

            ARC_Snake snake_copy( snake->center() );
            // TODO: not sure why, but iterator broken until we do this.
            do
            {
                ;
            } while( snake_copy.next_point() );

            converged=false;
            iter=0;
            while( converged==false && iter<8 )
            {
                ++iter;
                converged =true;
                energy = snake_copy.energy(image);
                do
                {
                    snake_copy.expand(4);
                    expand_energy = snake_copy.energy( image );
                    snake_copy.contract(8);
                    //contract_energy = snake_copy.energy( image );
            //        if( contract_energy < energy && contract_energy< expand_energy )
             //       {
              //          energy = contract_energy;
               //         converged=false;
                //    }
                    //else if( expand_energy < energy )
                    if( expand_energy < energy )
                    {
                        snake_copy.expand( 8);
                        energy = expand_energy;
                        converged=false;
                    }
                    else
                    {
                        snake_copy.expand(4);
                    }
                } while( snake_copy.next_point() );
                snake_copy.interpolate();
            }
            //drawContours( image2, s, 0, colors[0], 2, 8, noArray( ), 0, Point( )); //draws contour
            //circle(image2, center, 2, colors[1], 2 );
            *snake=snake_copy;
            //break; // We only want to track one contour for now
        }
        displayContours( image2, snakes, vidout, colors );
        prev_image = image.clone();
	}
}

#else


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Description:  Debugging main
 * =====================================================================================
 */
int main ( int argc, char *argv[] )
{
    Mat user_image, image;
    Point cp;
    vector<Point> user_points;

    cout << "Usage: " << argv[0] << " " << "<image file>" << endl;
    user_image = imread(argv[1], CV_LOAD_IMAGE_UNCHANGED);
    getUserPoints( user_image, user_points );
    namedWindow("snake");
    ARC_Snake m(user_points[0]);
    cout << "ARC_Snake debug: " << endl
         << "n - next point" << endl
         << "p - previous point" << endl
         << "x - expand at point" << endl
         << "c - contract at point" << endl
         << "i - interpolate new points (if possible)" << endl;


    while( 1 )
    {
        image = user_image.clone();
        char c;
        c = (char) waitKey( 10 );
        if( c == 27 )
            break;
        switch( c )
        {
        case 'n':
            m.next_point();
            break;
        case 'p':
            m.prev_point();
            break;
        case 'x':
            m.expand(4);
            cout << "Energy: " << m.energy(image) << endl;
            break;
        case 'c':
            m.contract(4);
            cout << "Energy: " << m.energy(image) << endl;
            break;
        case 'i':
            m.interpolate();
            break;
        default:
            ;
        }
        vector<vector<Point> > conts;
        m.get_contour(conts);
        drawContours( image, conts, 0, Scalar(0,255,0) );
        cp = m.get_point();
        circle( image, cp, 2, Scalar(255, 0, 0), 3 );
        imshow( "snake", image );
    }
    return EXIT_SUCCESS;
}				/* ----------  end of function main  ---------- */
#endif
