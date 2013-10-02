#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

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

static void help()
{
    cout << "Usage: ContourTracker <image list> [x y w h]" << endl;
}
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

int main(int argc, char** argv)
{
    help();
    double alpha=0.3; // For low pass filtering.
    //int sz=2; // Size of submatrix for averaging.
    Size rect_margin(10, 10);
    int npts=102400;
    int x=220;
    int y=160;
    int w=175;
    int h=150;
    if( argc>2 )
    {
        x = atoi( argv[2] );
        y = atoi( argv[3] );
        w = atoi( argv[4] );
        h = atoi( argv[5] );
    }
    
    vector<vector<Point> > contour;
    vector<Point> pts;
    vector<Point> prevdel(npts, Point(0,0));
    pts.push_back( Point(x, y) );
    pts.push_back( Point(x+w/4, y) );
    pts.push_back( Point(x+w/2, y) );
    pts.push_back( Point(x+w*3/4, y) );
    pts.push_back( Point(x+w, y) );
    pts.push_back( Point(x+w, y+h/4) );
    pts.push_back( Point(x+w, y+h/2) );
    pts.push_back( Point(x+w, y+h*3/4) );
    pts.push_back( Point(x+w, y+h) );
    pts.push_back( Point(x+w*3/4, y+h) );
    pts.push_back( Point(x+w/2, y+h) );
    pts.push_back( Point(x+w/4, y+h) );
    pts.push_back( Point(x, y+h) );
    pts.push_back( Point(x, y+h*3/4) );
    pts.push_back( Point(x, y+h/2) );
    pts.push_back( Point(x, y+h/4) );
    contour.push_back(pts);

	vector<string> images;
	getImageList( argv[1], &images );

    Mat prevgray, gray, flow, cflow, frame;
    //double prev_area=1;
    //double prev_def=1;
    double lambda=0.6;
    namedWindow("flow", 1);
    
    VideoWriter vidout;
    String videoName = "out.avi";
    Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
    vidout.open( videoName, CV_FOURCC( 'F', 'M', 'P', '4'), 20.0, firstFrame.size(), true );

	for( size_t k=1; k<images.size( ); k++ )
    {
        frame = imread( images[k], CV_LOAD_IMAGE_UNCHANGED );
        cvtColor(frame, gray, CV_BGR2GRAY);
        //medianBlur(gray,gray,5);

        if( prevgray.data )
        {
            calcOpticalFlowFarneback(prevgray, gray, flow, 0.25, 5, 30, 5, 5, 1.2, 0);
            //medianBlur(flow, flow, 3 );
            cvtColor(prevgray, cflow, CV_GRAY2BGR);
            drawOptFlowMap(flow, cflow, 16, 1.5, CV_RGB(0, 255, 0));
            int i=0;
            for( vector<Point>::iterator it=contour[0].begin();
                it!=contour[0].end(); ++it, ++i )
            {
                //Mat sub(flow, Rect(*it, Size(1, 1))); // locate submatrix at point
                // Resize submatrix
                //sub.adjustROI( sz, sz, sz, sz );
                //Scalar m;
                //m = mean(sub);
                //cout << "m: " << m <<endl;
                Point2f delk = flow.at<Point2f>(*it);
                /* 
                Point2f delk1, delk2;
                if( it!=contour[0].begin() )
                {
                    delk2 = flow.at<Point2f>(*(it-1));
                }
                else
                {
                    delk2 = flow.at<Point2f>(*(it+(npts-1)));
                }
                if( (it+1)!=contour[0].end() )
                {
                    delk1 = flow.at<Point2f>(*(it+1));
                }
                else
                {
                    delk1 = flow.at<Point2f>(*(it-(npts-1)));
                }
                Point del = 0.25*(delk1+delk2) + 0.5*delk;
                */
                Point del = delk;
                //cout << "delf: " << delf << endl;
                //Point del( (int) delf.x, (int) delf.y );
                //Point del( (int) m[0], (int) m[1] );
                if( k!=1 )
                {
                    prevdel[i] = alpha*prevdel[i] + (1-alpha)*del;
                }
                else
                {
                    prevdel[i] = del;
                }
                
                *it+=prevdel[i];
                if( it==contour[0].begin() )
                {
                    cout << *it << endl;
                }
            }
            //mask repositioned contour
            //Mat mask = Mat::zeros( gray.size(), CV_8UC1 );
            Rect image_rect( Point(0,0), frame.size() );
            Rect crect = boundingRect( contour[0] );
            crect+=rect_margin;
            crect-=(Point) rect_margin*0.5;
            crect&=image_rect;
            //rectangle( mask, crect, 255, CV_FILLED );
            // find new contour
            Mat edges;
            Mat masked_gray(gray,crect);
            //Canny(gray, edges,  0, 200, 7);
            Canny(masked_gray, edges,  100, 200, 3);
            int dilation_size = 3;
            Mat element = getStructuringElement( MORPH_RECT,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
            dilate( edges, edges, element, Point(-1, -1) );

            //imshow("flow", edges);
            //waitKey(0);
            vector<Vec4i> cds;
            //vector<vector<Point> > hulls;
            vector<int> hull;
            //hulls.push_back( h );
            convexHull( contour[0], hull );
            convexityDefects( contour[0], hull, cds );
            double flow_defect=0;
            double flow_area;
            for( vector<Vec4i>::iterator it=cds.begin();
                    it!=cds.end(); ++it )
            {
                flow_defect+=(*it)[3]/256.0;
            }
            flow_area = contourArea( contour[0] );
            //contour[0]=hulls[0];

            double min_vary=100;

            vector<vector<Point> > foundContours;
            vector<Vec4i> hierarchy;
            findContours( edges, foundContours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, crect.tl() );
            //double min_match_score=1;
            int m=0;
            for( vector<vector<Point> >::iterator con=foundContours.begin();
                    con!=foundContours.end(); ++con, ++m )
            {
                Mat contimg = cflow.clone();
                drawContours( contimg, contour, 0, Scalar(0,0,255), 2, 8, noArray( ), 0, Point( ));
                drawContours( contimg, foundContours, m, Scalar(255,0,0), 2, 8, noArray( ), 0, Point( ));
                //drawContours( contimg, hulls, 0, Scalar(0,255,0), 2, 8, noArray( ), 0, Point( ));
                double new_vary;
                double new_area;
                double new_defect=0;
                vector<int> new_hull;
                vector<Vec4i> new_cds;
                convexHull( *con, new_hull );
                convexityDefects( *con, new_hull, new_cds );
                for( vector<Vec4i>::iterator it=new_cds.begin();
                        it!=new_cds.end(); ++it )
                {
                    new_defect+=(*it)[3]/256.0;
                }
                new_area = contourArea( *con );
                new_vary=lambda*abs(new_area-flow_area)/(1+flow_area) + (1-lambda)*abs(new_defect-flow_defect)/(1+flow_defect);
                cout << "M: " << m << endl;
                cout << "dA: " << abs(new_area-flow_area)/(1+flow_area) <<endl;
                cout << "dD: " << abs(new_defect-flow_defect)/(1+flow_defect) <<endl;
                cout << "Vary: " << new_vary <<endl;
                if( new_vary<min_vary && new_vary<0.5 )
                {
                    min_vary=new_vary;
                    contour[0]=*con;
                    //imshow("flow", contimg);
                    //waitKey(0);
                }

            }
            //contour[0]=foundContours[0];
            // sanity check
            drawContours( cflow, contour, 0, Scalar(0,0,255), 2, 8, noArray( ), 0, Point( ));
            imshow("flow", cflow);
            vidout << cflow;
        }
        else
        {
            drawContours( gray, contour, 0, Scalar(0,0,255), 2, 8, noArray( ), 0, Point( ));
            imshow("flow", gray);
            waitKey(0);
        }

        if(waitKey(30)>=0)
            break;
        std::swap(prevgray, gray);
    }
    return 0;
}
