//Simon Peter
//Contour Tracker
//7-10-14
#include "ContourTracker.hpp"

using namespace std;
using namespace cv;

// GLOBALS
int areaThreshold = ARC_DEFAULT_AREA; //used 2000 and .13 for match
double distanceThreshold = ARC_DEFAULT_DISTANCE;
double mahalanobisThreshold = ARC_DEFAULT_MAHALANOBOIS;
int miscountThreshold = ARC_DEFAULT_MISCOUNT; //TODO: decide on a threshold to track between frames
int verbosity;

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
displayContours ( Mat image, vector<Contour> tracked, VideoWriter vidout, vector<Scalar> colors )
{
    //Draw the tracked contours for that frame
    vector<vector<Point> > contoursToDraw;
    objectToContours( &tracked, &contoursToDraw ); 	
    for(  size_t i=0; i<tracked.size( ); i++ )
    {
        if( tracked[i].nomatch==0 )
        {
            if( i<4 )	drawContours( image, contoursToDraw, i, colors[i], 1, 8, noArray( ), 0, Point( ));
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
void getContours( Mat image,  vector<Contour>  *contours )
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
            contours->push_back( Contour(*con) );
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
	vector<Contour> tracked; //vector to store contours that are tracked between frames

    init( argc, argv, images, vidout, colors );

	//Find initial contours to track on first image.
    Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
	getContours( firstFrame, &tracked ); //finds contours and stores them in tracked
	for( size_t i=0; i<1; i++ )
    {
		vector<vector<Point> > temp;
		objectToContours( &tracked, &temp );
		drawContours( firstFrame, temp, i, colors[i], 1, 8, noArray( ), 0, Point( )); //draws first image
	}
    if( verbosity==ARC_VERBOSE )
    {
        cout << "Original tracked size: " << tracked.size( ) <<endl;
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
        vector<Contour> newContours;
        Mat image = imread( *im, CV_LOAD_IMAGE_UNCHANGED );
        if(  !image.data )
        {
            cout << "Cannot load image." << endl;
            exit(  EXIT_FAILURE );
        }
        Mat image2 = image.clone();

        if( tracked.size( )==0 )
        {
			waitKey( 0 );
			getContours(image,&tracked);
    	}	

        if( verbosity==ARC_VERBOSE ) cout << "Tracked contours size: " << tracked.size( )<<endl;
        getContours( image, &newContours );
        if( verbosity==ARC_VERBOSE ) cout << "New contours size: " << newContours.size( )<<endl;

        //flow( prev_image, image, tracked );
        //Every tracked contour is checked against every new contour to see if there is a match
        for( vector<Contour>::iterator con=tracked.begin();
                con!=tracked.end(); ++con )
        {
			Rect trackedRect = boundingRect(con->contour);
			Mat trackedImage = image2(trackedRect);
            con->nomatch++; // increment here, because it will go to zero if match found.
			
            matchContours( image, *con, newContours );

            if( verbosity==ARC_VERBOSE ) cout << "Going to next tracked contour\n";
        }

        if( verbosity==ARC_VERBOSE ) cout << "Going to next frame\n";

        vector<vector<Point> > contoursToDraw;

        //if a tracked contour can't be found after a certain number of
        //frames,  it is kicked out
        vector<Contour>::iterator it=tracked.begin();
        while( it!=tracked.end() )
        {
            if( it->nomatch>=miscountThreshold )
            {
                it=tracked.erase( it );
                continue;
            }
            ++it;
        }

        displayContours( image2, tracked, vidout, colors );
        prev_image = image.clone();
	}
}
