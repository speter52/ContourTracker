//Simon Peter
//Contour Tracker
//7-10-14
#include "ContourTracker.hpp"
#define spc " "

using namespace std;
using namespace cv;


static void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step,
                    double, const Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {   
            const Point2f& fxy = flow.at<Point2f>(y, x); 
            line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, Point(x,y), 2, color, -1);
        }   
}

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

//Calculates the angle between two vectors given the vertex and two outside
//points
double angleBetween( Point point1,  Point vertex,  Point point2 )
{
    Point A, B;
    double dotProduct, angle;
	A = point1 - vertex;
	B = point2 - vertex;
	dotProduct = A.ddot(B);
	angle = acos( dotProduct/( norm(A)*norm(B) ));
	return angle;
}

//Fills a vector with rectangles detected in the given image
void findRectangles( Mat image,  vector<Contour>  *contours )
{
    Mat imageGray, cannyImage;
    vector<vector<Point> > foundContours;
    vector<vector<Point> > hulls;
    vector<vector<Point> > poly;
    vector<Vec4i> hierarchy;

    cvtColor( image,  imageGray, CV_BGR2GRAY );
    //blur( imageGray, imageGray,  Size( 3, 3 ));
    Canny( imageGray, cannyImage, 100, 200, 3 );

    //Finds initial contours
    findContours( cannyImage, foundContours, hierarchy, CV_RETR_TREE, 
            CV_CHAIN_APPROX_SIMPLE,  Point( 0, 0 ));

    // Gets the hulls of each contour which estimates straight lines when
    // there is a lot of concavity
    for( size_t i=0; i<foundContours.size( ); i++ )
    {
        vector<Point> hull;
        convexHull( foundContours[i], hull, true );
        hulls.push_back( hull );
    }

    //Approximates all the contours into polygons
    for( size_t i=0; i<hulls.size( ); i++ )
    {
        vector<Point> tempPoly;
        approxPolyDP( hulls[i], tempPoly, arcLength( foundContours[i], true )*.02, true );
        poly.push_back( tempPoly ); 	
    }
    
    // Checks to see if any of the polygons have angles that are too acute
    // or obtuse and stores the result in a vector of boolean flags
    vector<bool> angles;
    for(  size_t i=0; i<poly.size( ); i++ ){
        // Note: Changed to radians, since that is native format, we can avoid
        // floating point errors.
        double lowerThreshold = M_PI/4;
        double upperThreshold = 3*M_PI/4;
        if( angleBetween( poly[i][0], poly[i][1], poly[i][2])<lowerThreshold 
                || angleBetween( poly[i][1], poly[i][2], poly[i][3])<lowerThreshold 
                || angleBetween( poly[i][2], poly[i][3], poly[i][0])<lowerThreshold 
                || angleBetween( poly[i][3], poly[i][0], poly[i][1])<lowerThreshold
                || angleBetween( poly[i][0], poly[i][1], poly[i][2])>upperThreshold
                || angleBetween( poly[i][1], poly[i][2], poly[i][3])>upperThreshold 
                || angleBetween( poly[i][2], poly[i][3], poly[i][0])>upperThreshold 
                || angleBetween( poly[i][3], poly[i][0], poly[i][1])>upperThreshold )
        {
            angles.push_back( false );
        }
        else 
        {
            angles.push_back( true );
        }
    }

    for(  size_t i=0; i<poly.size( ); i++ ){
        //If the given hull is a rectangle,  large enough to not be an
        //artifact,  and meets the angle thresholds,  it is added to the
        //return vector
        if( poly[i].size( )==4 && contourArea( poly[i])>1500 && angles[i]==true )
        {
            Contour temp ( poly[i]);
            contours->push_back( temp );
        }
    }
}


int main( int argc,  char **argv )
{
    int x = 220;
    int y = 160;
    int w = 175;
    int h = 150;
    Mat prev, gray, cflow, flow;
    if(  argc<3 )
    {
        cout << "Usage: " << argv[0] << " " << "<image list> <output video filename> [x y w h]" << endl;
        exit(  EXIT_FAILURE );
    }
    if( argc>3 )
    {
        x=atoi( argv[3] );
        y=atoi( argv[4] );
        w=atoi( argv[5] );
        h=atoi( argv[6] );
    }
    cout << "x: " << x << spc
    << "y: " << y << spc
    << "w: " << w << spc
    << "h: " << h << endl;

    Rect initial_pos( x, y, w, h );
    

	vector<string> images;
	getImageList( argv[1], &images );

	VideoWriter vidout;
	String videoName =  argv[2];
	Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
	vidout.open( videoName, CV_FOURCC( 'F', 'M', 'P', '4'), 20.0, firstFrame.size( ), true );
    cvtColor( firstFrame, gray, CV_BGR2GRAY );

    namedWindow( "Contour Tracking", CV_WINDOW_AUTOSIZE );
    imshow( "Contour Tracking", gray );
    waitKey( 0 );
	vidout << firstFrame;
    prev = gray;

	//Begin image loop
	for( size_t k=1; k<images.size( ); k++ ){
        Mat image = imread( images[k], CV_LOAD_IMAGE_UNCHANGED );
        if(  !image.data )
        {
            cout << "Cannot load image." << endl;
            exit(  EXIT_FAILURE );
        }
        cvtColor( image, gray, CV_BGR2GRAY );
        calcOpticalFlowFarneback( prev, gray, flow, 0.5, 3,
                15, 3, 5, 1.2, 0 );
        cvtColor(prev, cflow, CV_GRAY2BGR );
        drawOptFlowMap( flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
        namedWindow( "Contour Tracking", CV_WINDOW_AUTOSIZE );

        cout << flow << endl;
        imshow( "Contour Tracking", cflow );
        waitKey( 25 );
        vidout << image;
        swap( prev, gray );
	}
}
