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
    Size rect_margin(50, 50);
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
    double lambda=0.3;
    namedWindow("flow", 1);
    
    VideoWriter vidout;
    String videoName = "out.avi";
    Mat firstFrame = imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
    vidout.open( videoName, CV_FOURCC( 'F', 'M', 'P', '4'), 20.0, firstFrame.size(), true );

	for( size_t k=1; k<images.size( ); k++ )
    {
        cout << "NEW FRAME" << endl;
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
            Mat rect_mat = Mat(frame.size(), CV_8UC3, Scalar(255,255,255));
            
            rectangle(rect_mat, crect, 0, CV_FILLED);
            GaussianBlur(rect_mat, rect_mat, Size(25,25), 120, 120);
            //imshow("flow", rect_mat);
            //waitKey(0);
            //rectangle( mask, crect, 255, CV_FILLED );
            // find new contour
            Mat edges ;
            //int thresh=50;
            //int N=11;
            //Mat masked_gray(gray,crect);

            double flow_area, min_sim;
            min_sim=10;
            flow_area = contourArea( contour[0] );
            vector<Point> contour_copy = contour[0];

            Mat masked_frame;
            //addWeighted( rect_mat, .5, frame, .5, 0.0, masked_frame );
            masked_frame = rect_mat + frame;
            // TODO crop outside of the blur and ensure area strictly less
            // than WxH
            //imshow("flow", masked_frame);
            //waitKey(0);
            Mat edges0(masked_frame.size(), CV_8U );
            for( int c = 0; c < 3; c++ )
            {
                int ch[] = {c, 0};
                mixChannels(&masked_frame, 1, &edges0, 1, ch, 1);
                cout << "Mix channels" << endl;

                // try several threshold levels
                //for( int l = 0; l < N; l++ )
                {
                    // hack: use Canny instead of zero threshold level.
                    // Canny helps to catch squares with gradient shading
                    //if( l == 0 )
                    //{
                        // apply Canny. Take the upper threshold from slider
                        // and set the lower to 0 (which forces edges merging)
                        Canny(edges0, edges, 100, 200, 3);
                        // dilate canny output to remove potential
                        // holes between edge segments
                        dilate(edges, edges, Mat(), Point(-1,-1));
                    //}
                    /*  
                    else
                    {
                        // apply threshold if l!=0:
                        //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                        edges = edges0 >= (l+1)*255/N;
                    }
                    */
                    //imshow("flow", edges);
                    //waitKey(0);

                    // find contours and store them all as a list

                    vector<vector<Point> > foundContours;
                    vector<Vec4i> hierarchy;
                    findContours( edges, foundContours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //, crect.tl() );
                    //double min_match_score=1;
                    int m=0;
                    for( vector<vector<Point> >::iterator con=foundContours.begin();
                    con!=foundContours.end(); ++con, ++m )
                    {
                        Mat contimg = cflow.clone();
                        vector<Point> approx;
                        //approxPolyDP(Mat(*con), approx, arcLength(*con, true)*0.02, true);
                        //*con=approx;
                        drawContours( contimg, contour, 0, Scalar(0,0,255), 2, 8, noArray( ), 0, Point( ));
                        drawContours( contimg, foundContours, m, Scalar(255,0,0), 2, 8, noArray( ), 0, Point( ));
                        //drawContours( contimg, hulls, 0, Scalar(0,255,0), 2, 8, noArray( ), 0, Point( ));
                        double new_area, dif_area;
                        double new_match;
                        double similarity;
                        new_match = matchShapes( contour[0], *con, CV_CONTOURS_MATCH_I1, 0 );
                        new_area = contourArea( *con );
                        dif_area = abs(new_area-flow_area)/max(new_area,flow_area);
                        similarity = (lambda*new_match)*(lambda*new_match) + (1-lambda)*dif_area*(1-lambda)*dif_area;
                            //imshow("flow", contimg);
                            //waitKey(0);

                        if( similarity<min_sim && new_match<0.8 && dif_area<.25 )
                        {
                            cout << "Similarity: " << similarity << endl;
                            cout << "Match: " << new_match << endl;
                            cout << "Dif area: " << dif_area << endl;
                            //imshow("flow", contimg);
                            //waitKey(0);
                            min_sim=similarity;
                            contour_copy=*con;
                        }
                    }
                }
            }
            contour[0]=contour_copy;
            //Canny(gray, edges,  0, 200, 7);
            //Canny(masked_gray, edges,  100, 200, 3);
            //int dilation_size = 3;
            //Mat element = getStructuringElement( MORPH_RECT,
             //                          Size( 2*dilation_size + 1, 2*dilation_size+1 ),
              //                         Point( dilation_size, dilation_size ) );
            //dilate( edges, edges, element, Point(-1, -1) );

            //imshow("flow", edges);
            //waitKey(0);
            //vector<vector<Point> > hulls;
            //hulls.push_back( h );
            //contour[0]=hulls[0];

            //contour[0]=foundContours[0];
            // sanity check
            Mat mask = Mat(frame.size(), CV_8UC1, Scalar(0));
            drawContours( cflow, contour, 0, 0, 1, 8, noArray( ), 0, Point( ));
            drawContours( mask, contour, 0, Scalar(255), CV_FILLED, 8, noArray( ), 0, Point( ));
            imshow("flow", cflow);
            vidout << mask;
            int n = images[k].find_last_of('/');
            string base= "out/";
            string fn = images[k].substr(n+1);
            base.append(fn);
            imwrite(base, mask);
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
