//Simon Peter
//Contour Tracker
//7-10-14
#include "ContourTracker.hpp"

using namespace std;
using namespace cv;


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
	int x1 = point1.x-vertex.x;
	int y1 = point1.y-vertex.y;
	int x2 = point2.x-vertex.x;
	int y2 = point2.y-vertex.y;
	Point A( x1, y1 );
	Point B( x2, y2 );
	double dotProduct = A.ddot(B);
	double magnitudeA = norm( A );
	double magnitudeB = norm( B );
	double angle = acos( dotProduct/( magnitudeA*magnitudeB ));
	return angle*180/M_PI; 	

}

//Fills a vector with rectangles detected in the given image
void findRectangles( Mat image,  vector<Contour>  *contours )
{
    Mat imageGray;
    cvtColor( image,  imageGray, CV_BGR2GRAY );
    blur( imageGray, imageGray,  Size( 3, 3 ));
    Mat cannyImage;
    vector<vector<Point> > foundContours;
    vector<Vec4i> hierarchy;

    Canny( imageGray, cannyImage, 100, 200, 3 );
    //Finds initial contours
    findContours( cannyImage, foundContours, hierarchy, CV_RETR_TREE, 
            CV_CHAIN_APPROX_SIMPLE,  Point( 0, 0 ));

    // Gets the hulls of each contour which estimates straight lines when
    // there is a lot of concavity
    vector<vector<Point> > hulls;
    for( size_t i=0; i<foundContours.size( ); i++ )
    {
        vector<Point> hull;
        convexHull( foundContours[i], hull, true );
        hulls.push_back( hull );
    }

    //Approximates all the contours into polygons
    vector<vector<Point> > poly;
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
        double lowerThreshold = 45;
        double upperThreshold = 145;
        //double firstAngle = angleBetween( poly[i][0], poly[i][1], poly[i][2]);
        //double secondAngle = angleBetween( poly[i][1], poly[i][2], poly[i][3]);
        //double thirdAngle = angleBetween( poly[i][2], poly[i][3], poly[i][0]);
        //double fourthAngle = angleBetween( poly[i][3], poly[i][0], poly[i][1]);
        //cout << "First: " << firstAngle 
        //<< " Second: " << secondAngle 
        //<< " Third: " << thirdAngle 
        //<< " Fourth: " << fourthAngle << endl;
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
    if(  argc<3 )
    {
        cout << "Usage: " << argv[0] << " " << "[image list] [output video filename]" << endl;
        exit(  EXIT_FAILURE );
    }
	vector<string> images;
	getImageList( argv[1], &images );

	VideoWriter vidout;
	String videoName =  argv[2];
	Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
	vidout.open( videoName, CV_FOURCC( 'F', 'M', 'P', '4'), 20.0, firstFrame.size( ), true );

	vector<Contour> tracked; //vector to store contours that are tracked between frames
	findRectangles( firstFrame, &tracked ); //finds contours and stores them in tracked
	//Finds initial contours to track on first image
	for(  size_t i=0; i<tracked.size( ); i++ ){
		Scalar color( 0, 0, 255 );
		vector<vector<Point> > temp;
		objectToContours( &tracked, &temp );
		drawContours( firstFrame, temp, i, color, 2, 8, noArray( ), 0, Point( )); //draws first image
	}
	cout << "Original tracked size: " << tracked.size( ) <<endl;
	namedWindow( "Contour Tracking", CV_WINDOW_AUTOSIZE );
	imshow( "Contour Tracking", firstFrame );
	waitKey( 0 ); 	
	vidout << firstFrame;

	//Begin image loop
	for(  size_t k=1; k<images.size( ); k++ ){
			if( tracked.size( )==0 ) waitKey( 0 );
			Mat image = imread( images[k], CV_LOAD_IMAGE_UNCHANGED );
            if(  !image.data )
            {
                cout << "Cannot load image." << endl;
                exit(  EXIT_FAILURE );
            }
/*			if( k%60==0 || tracked.size( )==0 ){
				tracked.clear( );
				nomatch.clear( );
				findRectangles( image, &tracked );
				for(  size_t l=0; l<tracked.size( ); l++ ){
					nomatch.push_back( 0 );
				}
				cout << "60 frames reached - clearing tracked\n";
			}	*/	

            cout << "Tracked contours size: " << tracked.size( )<<endl;
			vector<Contour> newContours; //Stores the contours found in the new frame
			findRectangles( image, &newContours );
            cout << "New contours size: " << newContours.size( )<<endl;
		
			//int oldSize = tracked.size( );
/*			if( tracked.size( )<=0 ){
				cout << "Finding new contours\n";
				tracked.clear( );
				nomatch.clear( );
				findRectangles( image, &tracked );
				for(  size_t z=0; z<tracked.size( ); z++ ){
					nomatch.push_back( 0 );
				}
			}
*/
			//Every tracked contour is checked against every new contour to see if there is a match
			for( size_t m=0; m<tracked.size( ); m++ )
            {
				for( size_t n=0; n<newContours.size( ); n++ )
                {
                    /*	
                        Scalar color( 255, 0, 0 ); 	
                        Mat drawing = Mat::zeros( firstFrame.size( ), CV_8UC3 );
                        drawContours( drawing, tracked, m, color, 2, 8, noArray( ), 0, Point( ));
                        namedWindow( "Contour to track",  CV_WINDOW_AUTOSIZE );
                        imshow( "Contour to track", drawing );
                        moveWindow( "Contour to track", 700, 0 );
                    */

					//Histogram test - Creates a bounding rect around the contours and then compares their histograms
					Rect origRect ( 0, 0, image.cols, image.rows ); 	
					Mat image2 = image.clone( );
					Rect trackedContour = boundingRect( tracked[m].contour );
					Mat trackedCont = image2( trackedContour );
					Rect newContour ( newContours[n].TL.x, newContours[n].TL.y, trackedContour.width, trackedContour.height );
					newContour = origRect & newContour;
					Mat newCont = image2( newContour );

					Mat hsvTracked,  hsvNew;
					cvtColor( trackedCont, hsvTracked, CV_BGR2HSV );
					cvtColor( newCont, hsvNew, CV_BGR2HSV );

					int h_bins = 50; int s_bins = 32;
					int histSize[] = { h_bins,  s_bins };

					float h_ranges[] = { 0,  256 };
					float s_ranges[] = { 0,  180 };

					const float* ranges[] = { h_ranges,  s_ranges };

					int channels[] = { 0,  1 };

					MatND hist_track,  hist_new;
					calcHist( &hsvTracked, 1, channels, Mat( ), hist_track, 2, histSize, ranges, true, false );
					normalize( hist_track, hist_track, 0, 1, NORM_MINMAX, -1, Mat( ));
					calcHist( &hsvNew, 1, channels, Mat( ), hist_new, 2, histSize, ranges, true, false );
					normalize( hist_new, hist_new, 0, 1, NORM_MINMAX, -1, Mat( ));
					double compare = compareHist( hist_track, hist_new, 1 );
					cout << "compare: " << compare << endl;
					

					//double compareThreshold = 40; //used 40 and .2 for match	
					double matchThreshold = .08; //used .08 or .13
					int positionThreshold = 60; //used 60 and .2 for match
					//int areaThreshold = 2000; //used 2000 and .13 for match
					
					//Shapes test - Uses moments to compare the actual shape of two contours
                    //TODO: Get matchShapes working
					double matchReturn = matchShapes( tracked[m].contour, newContours[n].contour, 
                            CV_CONTOURS_MATCH_I1, 0 ); 
					cout << "matchShapes return: " << m << " " << n << " " << matchReturn << endl;
				
                    // Position test - Checks to see if the top left ( top right
                    // not included for now ) corners between two contours are
                    // significantly different	
					bool isNear = true;
					cout << "Current contour TL: " << tracked[m].TL << " New Contour TL: " << newContours[n].TL << endl;
					cout << "Current contour BR: " << tracked[m].BR << " New Contour BR: " << newContours[n].BR << endl;
					int xDiffTL = abs( tracked[m].TL.x-newContours[n].TL.x );
					int yDiffTL = abs( tracked[m].TL.y-newContours[n].TL.y );
					int xDiffBR = abs( tracked[m].BR.x-newContours[n].BR.x );
					int yDiffBR = abs( tracked[m].BR.y-newContours[n].BR.y );
					cout << "xdiffTL: " << xDiffTL << " ydiffTL: " << yDiffTL << endl;
					cout << "xdiffBR: " << xDiffBR << " ydiffBR: " << yDiffBR << endl;
					if( xDiffTL>positionThreshold 
                            || yDiffTL>positionThreshold
                            /* 
                            || xDiffBR>positionThreshold 
                            || yDiffBR>positionThreshold
                            */ 
                            ) 
                    {
                        isNear = false;
                    }
					cout << "isNear: " << isNear << endl;
					
					//Area test - Compares the areas between two contours
					int trackedArea = contourArea( tracked[m].contour );
					int newArea = contourArea( newContours[n].contour );
					int areaDifference = abs( trackedArea-newArea );
					cout << "Area difference: " << areaDifference << endl;
					

					//Displays a bounding rect around two contours
					namedWindow( "tracked cont", CV_WINDOW_AUTOSIZE );
			//		imshow( "trackedcont", trackedCont );
					namedWindow( "newCont", CV_WINDOW_AUTOSIZE );
			//		imshow( "newCont", newCont );
					moveWindow( "newCont", 0, 500 );
			//		waitKey( 0 );
					cout << "images created\n";

                    // If the contour passes all the tests,  it is added to the
                    // tracked vector and taken out of the newContours vector
					if( matchReturn<=matchThreshold 
                            /*
                            && isNear==true 
                            && areaDifference<=areaThreshold 
                            && compare<compareThreshold
                            */
                            )
                    {
                        tracked[m].contour=newContours[n].contour;
                        tracked[m].nomatch = 0; //set nomatch to 0,  since it was found in that frame
                        cout << "Match found\n";
                        newContours.erase( newContours.begin( )+n );
                        cout << "New contours modified size: " << newContours.size( )<<endl;
                        //after found,  it can stop trying to match that
                        //particular tracked contour and move on to match the
                        //next tracked contour
                        break; 
					}
					if( n==newContours.size( )-1 ) 
                    {
                        //if no match for the tracked contour is found after
                        //searching all the new contours,  increment nomatch
                        tracked[m].nomatch++; 
						cout << "No match found,  going to next new contour\n";
					}
				}
			cout << "Going to next tracked contour\n";
			}

			//Adds unmatched contours to as a new contour to track
	/*		for(  size_t l=0; l<newContours.size( ); l++ ){
				tracked.push_back( newContours[l]);
				nomatch.push_back( 0 );
			}*/

			cout << "Going to next frame\n";
			
			int miscountThreshold = 30; //TODO: decide on a threshold to track between frames
			vector<vector<Point> > contoursToDraw;
		

            //if a tracked contour can't be found after a certain number of
            //frames,  it is kicked out
            for(  size_t m=0; m<tracked.size( ); m++ )
            {
				if( tracked[m].nomatch>=miscountThreshold )
                {
					tracked.erase( tracked.begin( )+m );
				}
				//if( nomatch[m]<=1 ) contoursToDraw.push_back( tracked[m]);
			}
								
			//Draw the tracked contours for that frame
			for(  size_t i=0; i<tracked.size( ); i++ )
            {
				if( tracked[i].nomatch==0 )
                {
					Scalar red ( 0, 0, 255 );
					Scalar blue( 255, 0, 0 );
					Scalar green( 0, 255, 0 );
					Scalar black( 0, 0, 0 );
					vector<vector<Point> > contoursToDraw;
					objectToContours( &tracked, &contoursToDraw ); 	
					if( i==0 )	drawContours( image, contoursToDraw, i, red, 2, 8, noArray( ), 0, Point( ));
					if( i==1 )	drawContours( image, contoursToDraw, i, blue, 2, 8, noArray( ), 0, Point( ));
					if( i==2 )	drawContours( image, contoursToDraw, i, green, 2, 8, noArray( ), 0, Point( ));
					if( i==4 )	drawContours( image, contoursToDraw, i, black, 2, 8, noArray( ), 0, Point( ));
				}
			}

			namedWindow( "Contour Tracking", CV_WINDOW_AUTOSIZE );
			imshow( "Contour Tracking", image );
			waitKey( 25 );
			vidout << image;
	}
}
