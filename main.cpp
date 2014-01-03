#include "ARC_FindContours.hpp"

int main(int argc, char** argv)
{
    Mat image = imread(argv[1],CV_LOAD_IMAGE_UNCHANGED);
    vector<vector<Point> > contours;

    //ARC_FindContours fc(0,INT_MAX,30,150,false,false); 
//    fc.get_contours(image,contours);
	ARC_FindContours fc;
	fc.get_quads(image,contours);
cout<<"Detected Contours: "<<contours.size()<<endl;
    Scalar red(0,0,255);
    Mat canvas = Mat::zeros(image.size(),CV_8UC3);

    for(size_t i=0;i<contours.size();i++)
    {   
        drawContours(canvas,contours,i,red,2,8,noArray(),0,Point());
    }  
	imwrite("out.jpg",canvas); 
    namedWindow("Contours",CV_WINDOW_AUTOSIZE);
    imshow("Contours",canvas);
    waitKey(0);
}

