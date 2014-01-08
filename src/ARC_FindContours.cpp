//Simon Peter
//Class to find contours for VISP library
//speter3@illinois.edu

#include "ARC_FindContours.hpp"

//Public - Constructor that uses default values for thresholds
ARC_FindContours::ARC_FindContours()
{
	int imageWidth = 640;
	int imageHeight = 480;
	minContourArea = .0015*imageWidth*imageHeight;
	maxContourArea = .22*imageWidth*imageHeight;
	lowerAngleThreshold = 30;
	upperAngleThreshold = 150;
	numOfSides = -1;
	runPoly = true;
	runHull = true;
    edge_margin = 5;
}

//Public - Constructor that initializes area thresholds but uses defaults for angle thresholds
ARC_FindContours::ARC_FindContours(int minArea, int maxArea, bool runApproxPolyDP,
        bool runConvexHull)
{
	minContourArea = minArea;
	maxContourArea = maxArea;
	lowerAngleThreshold = 30;
	upperAngleThreshold = 150;
	numOfSides = -1;
	runPoly = runApproxPolyDP;
	runHull = runConvexHull;
}
	
//Public - Constructor that initializes all thresholds
ARC_FindContours::ARC_FindContours(int minArea, int maxArea, int lowerAngle,
        int upperAngle, bool runApproxPolyDP, bool runConvexHull)
{
	minContourArea = minArea;
	maxContourArea = maxArea;
	lowerAngleThreshold = lowerAngle;
	upperAngleThreshold = upperAngle;
	numOfSides = -1;
	runPoly = runApproxPolyDP;
	runHull = runConvexHull;
}

//Public - Set minimum contour area threshold
void ARC_FindContours::set_min_area(int minArea)
{
	minContourArea = minArea;
}

//Public - Set maximum contour area threshold
void ARC_FindContours::set_max_area(int maxArea)
{
	maxContourArea = maxArea;
}

//Public - Set lower angle threshold for quadrilaterals
void ARC_FindContours::set_lower_angle(int lowerAngle)
{
	lowerAngleThreshold = lowerAngle;
}

//Public - Set upper angle threshold for quadrilaterals
void ARC_FindContours::set_upper_angle(int upperAngle)
{
	upperAngleThreshold = upperAngle;
}

//Public - Run approxPolyDB on contours
void ARC_FindContours::set_approx_poly(bool runApproxPolyDP)
{
	runPoly = runApproxPolyDP;
}

//Public - Run convexHull on contours
void ARC_FindContours::set_convex_hull(bool runConvexHull)
{
	runHull = runConvexHull;
}

//Private - Calculate dot product
double ARC_FindContours::dot(cv::Point A, cv::Point B)
{
    return A.x*B.x+A.y*B.y;
}

//Private - Calculate magnitude of a vector
double ARC_FindContours::findMagnitude(cv::Point vector)
{
    return sqrt(vector.x*vector.x + vector.y*vector.y);
}

//Private - Calculate angle between 3 points
double ARC_FindContours::angleBetween(cv::Point point1, cv::Point vertex, cv::Point point2)
{
    int x1 = point1.x-vertex.x;
    int y1 = point1.y-vertex.y;
    int x2 = point2.x-vertex.x;
    int y2 = point2.y-vertex.y;
    cv::Point A(x1,y1);
    cv::Point B(x2,y2);
    double dotProduct = dot(A,B);
    double magnitudeA = findMagnitude(A);
    double magnitudeB = findMagnitude(B);
    double angle = acos(dotProduct/(magnitudeA*magnitudeB));
    return angle*180/M_PI;
}

//Private - Test angles in a quadrilateral against thresholds
//TODO: should it be a &reference?
bool ARC_FindContours::angleTest(std::vector<cv::Point> contour)
{
	if( angleBetween( contour[0], contour[1], contour[2] )<lowerAngleThreshold ||
            angleBetween( contour[1], contour[2], contour[3] )<lowerAngleThreshold ||
            angleBetween( contour[2], contour[3], contour[0] )<lowerAngleThreshold ||
            angleBetween( contour[3], contour[0], contour[1] )<lowerAngleThreshold ||
            angleBetween( contour[0], contour[1], contour[2] )>upperAngleThreshold ||
            angleBetween( contour[1], contour[2], contour[3] )>upperAngleThreshold ||
            angleBetween( contour[2], contour[3], contour[0] )>upperAngleThreshold ||
            angleBetween( contour[3], contour[0], contour[1] )>upperAngleThreshold )
    {
        return false;
    }
	return true;
}

//Private - Tests and modifies a given contour with the thresholds and returns if it passed or not
bool ARC_FindContours::applyThresholds(std::vector<cv::Point>& contour)
{
	if( runHull ) 
	{
        std::vector<cv::Point> hull;
		convexHull(contour,hull,true);
		contour = hull;
	}
	if( runPoly )
	{
        std::vector<cv::Point> poly;
		approxPolyDP(contour, poly, arcLength( contour, true )*.01, true );
		contour = poly;
	}

	if( numOfSides>0 )
	{
		if( (int)contour.size()!=numOfSides ) return false;	
		if( numOfSides==4 )
		{
			if( angleTest(contour)==false ) return false;
		}
	}
	double area = contourArea(contour,false);
	if(area<minContourArea || area>maxContourArea) return false;
	return true;
}

//Private - Checks if the newContours pass the thresholds and don't intersect curContours
void ARC_FindContours::filterContours(std::vector<std::vector<cv::Point> >& curContours,
        std::vector<std::vector<cv::Point> >& newContours)
{
	size_t newContoursSize = newContours.size();
	for( size_t i=0; i<newContoursSize; i++)
	{
		bool isSeparate= true;
		bool meetsThresholds = applyThresholds(newContours[i]);

		//Checks if the new contours intersect or are within the passed-in current contours
		for(size_t j=0;(j<curContours.size() && meetsThresholds);j++)
		{
			for(size_t k=0;k<newContours[i].size();k++)
			{
				if(pointPolygonTest(curContours[j],newContours[i][k],false)<1)
				{
					isSeparate= false;
					break;
				}
			}
			if(!isSeparate) break;
		}

		if(!isSeparate || !meetsThresholds)
		{
			newContours.erase(newContours.begin()+i);
			newContoursSize--;
			i--;
		}
	}
}

//Private - Processes image for findContours
void ARC_FindContours::getCanny(cv::Mat& image)
{
    cv::Mat imageGray;
	cvtColor( image, imageGray, CV_BGR2GRAY );
	blur( imageGray, imageGray, cv::Size(3,3));
    cv::Mat cannyImage;
    cv::Mat image2 = imageGray.clone();
    cv::Mat empty;
    //TODO: Come up with a formula to automatically calculate canny thresholds?
    //The formula below doesn't have good results so I hardcoded in values that
    //worked before
	//double high_thres = cv::threshold( image2,empty, 0, 255, CV_THRESH_BINARY+CV_THRESH_OTSU );
	//double lower_thres = .5*high_thres;
    //Canny(imageGray,cannyImage,lower_thres,high_thres);
	Canny(imageGray,cannyImage,100,200,3);
    //Running dilate dramatically changes the contours that we get
	dilate( cannyImage, cannyImage, cv::Mat(), cv::Point(-1,-1) );
	image = cannyImage;
}

//Public - Returns detected contours from a given image
bool ARC_FindContours::get_contours(cv::Mat image,
        std::vector<std::vector<cv::Point> >& curContours,
        std::vector<std::vector<cv::Point> >& newContours )
{
    std::vector<cv::Vec4i> hierarchy;

	getCanny(image);
	findContours( image, newContours, hierarchy,
            CV_RETR_LIST,CV_CHAIN_APPROX_NONE, cv::Point(0,0) );
	filterContours(curContours,newContours);
	
	if(newContours.size()==0) return false;
	return true;
}

//Public - Returns detected quadrilaterals from a given image
bool ARC_FindContours::get_quads(cv::Mat image,
        std::vector<std::vector<cv::Point> >& curContours,
        std::vector<std::vector<cv::Point> >& newContours )
{
	//Adjusts numOfSides for approxPoly and angle tests
	numOfSides = 4;
	bool found = get_contours(image, curContours, newContours);
	numOfSides = -1;
	return found;
}


/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_FindContours
 *      Method:  ARC_FindContours :: squares
 * Description:  Follows the method in opencv/samples/cpp/squares.cpp to find
 * quadrilaterals in an image.
 *--------------------------------------------------------------------------------------
 */
    void
ARC_FindContours::squares ( cv::Mat image,
        std::vector<std::vector<cv::Point> >& curContours,
        std::vector<std::vector<cv::Point> >& newContours)
{
    int thresh;
    int N;
    cv::Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    std::vector<cv::Point> image_edge;

    thresh =50;
    N=11;

    image_edge.push_back( cv::Point(0,0) );
    image_edge.push_back( cv::Point(640,0) );
    image_edge.push_back( cv::Point(640,480) );
    image_edge.push_back( cv::Point(0,480) );
    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, cv::Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    std::vector<std::vector<cv::Point> > contours;

    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 0, thresh, 5);
                // dilate canny output to remove potential
                // holes between edge segments
                dilate(gray, gray, cv::Mat(), cv::Point(-1,-1));
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }

            // find contours and store them all as a list
            //findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
            findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_TC89_L1);
            std::vector<std::vector<cv::Point> >::iterator new_con=contours.begin();
            while( new_con!=contours.end() )
            {
                bool inside=false;
                for( std::vector<cv::Point>::iterator pt=new_con->begin();
                        pt!=new_con->end(); ++pt )
                {
                    if( abs(pointPolygonTest( image_edge, *pt, true))<edge_margin ) 
                    {
                        inside=true;
                        break;
                    }
                    for( std::vector<std::vector<cv::Point> >::iterator con=newContours.begin();
                            con!=newContours.end(); ++con )
                    {
                        if( abs(pointPolygonTest(*con, *pt, true ))<edge_margin)
                        {
                            inside=true;
                            break;
                        }
                    }
                    if( inside ) break;
                    for( std::vector<std::vector<cv::Point> >::iterator con=curContours.begin();
                            con!=curContours.end(); ++con )
                    {
                        if( abs(pointPolygonTest(*con, *pt, true ))<edge_margin)
                        {
                            inside=true;
                            break;
                        }
                    }
                }
                if( inside )
                {
                    new_con=contours.erase( new_con );
                    continue;
                }
                ++new_con;
            }

            std::vector<cv::Point> approx;
            
            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.02, true);
                
                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                if( approx.size() == 4 &&
                    fabs(contourArea(cv::Mat(approx))) > 1000 &&
                    fabs(contourArea(cv::Mat(approx))) < 640 * 480 * 0.5 &&
                    isContourConvex(cv::Mat(approx)) )
                {
                    double maxCosine = 0;

                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }

                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                    {
                        newContours.push_back(approx);
                    }
                    
                }
            }
        }
    }
    return ;
}		/* -----  end of method ARC_FindContours::squares  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  ARC_FindContours
 *      Method:  ARC_FindContours :: angle
 * Description:  finds a cosine of angle between vectors from pt0->pt1 and
 * from pt0->pt2
 *--------------------------------------------------------------------------------------
 */
    double
ARC_FindContours::angle ( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}		/* -----  end of method ARC_FindContours::angle  ----- */

