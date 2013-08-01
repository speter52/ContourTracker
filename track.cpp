#include </usr/local/include/opencv/cv.h> 
#include </usr/local/include/opencv/highgui.h>
#include <stdio.h>
#include <iostream>

using namespace std;

FILE * trackFile = fopen("result.txt", "w");


int nf = 8;
int mouseFlag = 0;
CvPoint pt;

void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( event == CV_EVENT_LBUTTONDOWN ){
	mouseFlag = 0;
    }
    if( event == CV_EVENT_RBUTTONDOWN ){ // click right button four times and click left button
        pt = cvPoint(x,y);
	for ( int i = 0; i < nf; i++){
		if (mouseFlag == 2*i)
			mouseFlag = 2*i+1;
	}
    }


}

IplImage* unDistort(IplImage* pic);

int main() {
	int  n = 11850;
	char buffer[10];
	float xs;
	float ys;
	float zs;
	float xs0;
	float ys0;	
	float xsys;

	double quality_level = 0.1;
	double min_distance = 5;
	int block_size = 5;
	int use_harris = false;
	double k = 0.04;

	int numFeat = 1;
	int* numFeatures = &numFeat;
	char status[nf];
	float track_error[nf];
	CvPoint2D32f featuresA[nf];
	CvPoint2D32f featuresB[nf];
	CvPoint2D32f featuresC[nf];
	CvMat* sphere = cvCreateMat(3,1,CV_32F);

	IplImage* imgColor = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);	// 428 depends on radius in testImageWrap
	IplImage* imgEx = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
	IplImage* img = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* imgCorner = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* imgA = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* imgB = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* pyramid1 = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* pyramid2 = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* imgThresh = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* imgSmooth = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1);
	IplImage* imgBB = cvCreateImage(cvSize(40,40),IPL_DEPTH_8U,1);
		cout<<"Created all iplimages\n";
	cvNamedWindow( "window", 0 );
	cvResizeWindow( "window", 1540/2, 1540/2 );

	cvNamedWindow( "select", 0 );
	cvResizeWindow( "select", 1540/2, 1540/2 );
	cvMoveWindow( "select", 1540/2,0);

//	cvNamedWindow( "box", 0 );
//	cvResizeWindow( "box", (2513+100)/3*2, 428/3*2 );
//	cvMoveWindow( "box", 0, 650);

//	cvNamedWindow( "zero", 0 );
//	cvResizeWindow( "zero", 2513/2, 428/4 );
//	cvMoveWindow( "zero", 0, 600);

	CvPoint p,q;
	double angle;
	double hypotenuse;
	double pi = 3.141592;

	char text[100];
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,.5,.5,0,1,8);
//|CV_FONT_HERSHEY_PLAIN
	while(n != 0) {

		sprintf(buffer,"../img/img/%d.jpg", n);
//		sprintf(buffer,"Mar0420131552/img/%d.jpg", n);

		imgColor = cvLoadImage( buffer );
		cout<<"Image loaded\n";
//		cvShowImage( "original", imgColor );
		imgColor = unDistort(imgColor);
//		cvShowImage( "rectified", imgColor );
		cvCopy(imgColor,imgEx);
		cvConvertImage(imgEx,img);
		cvCopy(img,imgCorner);
//		cvLine(img, cvPoint((2513+50)/2,0), cvPoint((2513+50)/2,428), CV_RGB(0, 0, 0), 3, CV_AA, 0);
//		cvShowImage( "select", img );

//		cvThreshold( img, img, 70, 255, CV_THRESH_BINARY );
//		cvSmooth(img, img, CV_MEDIAN, 9, 0, 0, 0);
////		cvAdaptiveThreshold(img, img, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 3, 0 );
//		cvAdaptiveThreshold(img, img, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 3, 0 );

//		cvCanny( img, img, 10, 200, 3 );
//		cvConvertImage(imgThresh,img);
//		imgColor = cvCloneImage(imgThresh);

//		cvSetImageROI(img, cvRect(0,150,2513+100,428-150));
//		cvCopy(img,imgCorner);
//		cvResetImageROI(img);
		cvShowImage( "select", img );

		for ( int i = 0; i < nf; i++){
			while (mouseFlag == 2*i){
				cvConvertImage(img,imgA);
				cvSetMouseCallback( "select", on_mouse, 0 );
				cvWaitKey( 30 );
			}
			if (mouseFlag == 2*i+1){
				featuresA[i] = cvPointTo32f(pt);
				mouseFlag = 2*i+2;
			}
		}

//		cout << n%5000 << endl;
		cvConvertImage(img,imgB);

/*		int i = 0;
		cvSetImageROI(imgCorner, cvRect((int)featuresA[i].x-20,(int)featuresA[i].y-20,40,40));
		cvSetImageROI(imgBB, cvRect(0,0,40,40));
		cvCopy(imgCorner,imgBB);
		cvResetImageROI(imgCorner);
		cvResetImageROI(imgBB);

		if (n%10 == 0){
			cvGoodFeaturesToTrack(imgBB, NULL, NULL, featuresC, numFeatures, quality_level, 
				min_distance, NULL, block_size, use_harris, k);
			cout << "!!!!" << endl;
			int posAx = (int)featuresA[i].x-20;
			int posAy = (int)featuresA[i].y-20;
			featuresA[0].x = featuresC[0].x + posAx;
			featuresA[0].y = featuresC[0].y + posAy;
		}
*/
		CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );
		IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
		IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

		CvTermCriteria criteria = cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3);

		cvCalcOpticalFlowPyrLK(imgA, imgB, pyrA, pyrB, featuresA, featuresB, nf, 
			cvSize(20,20), 3, status, track_error, criteria, 1);

		cvConvertImage(img,imgA);

		for ( int i = 0; i < nf; i++){

			p.x = (int) featuresA[i].x;
			p.y = (int) featuresA[i].y;
			q.x = (int) featuresB[i].x;
			q.y = (int) featuresB[i].y;

			angle = atan2( (double) p.y - q.y, (double) p.x - q.x);
			hypotenuse = sqrt( pow(p.y - q.y, 2) + pow(p.x - q.x, 2));

			sprintf(text, "%d (%d, %d)", i, q.x, q.y);
			cvPutText(imgEx, text, cvPoint(q.x+2,q.y+2), &font, cvScalar(255,0,0));

			q.x = (int) (p.x - 1 * hypotenuse * cos(angle));
			q.y = (int) (p.y - 1 * hypotenuse * sin(angle));
			cvLine(imgEx, p, q, CV_RGB(255-i, 70, 100), 3, CV_AA, 0);
			p.x = (int) (q.x + 3 * cos(angle + pi / 4));
			p.y = (int) (q.y + 3 * sin(angle + pi/4));
			cvLine(imgEx, p, q, CV_RGB(255-i, 70, 100), 3, CV_AA, 0);
			p.x = (int) (q.x + 3 * cos(angle - pi/4));
			p.y = (int) (q.y + 3 * sin(angle - pi/4));
			cvLine(imgEx, p, q, CV_RGB(255-i, 70, 100), 3, CV_AA, 0);
		}
		for ( int i = 0; i < nf; i++){
			featuresA[i].x = featuresB[i].x;
			featuresA[i].y = featuresB[i].y;
			xs0 = cos(featuresB[i].x/(2513+50)*2*pi);
			ys0 = sin(featuresB[i].x/(2513+50)*2*pi);
			zs = sin(atan((65-featuresB[i].y)/479));
			xsys = sqrt(1-zs*zs);
			xs = xs0*xsys;
			ys = ys0*xsys;

			cvmSet( sphere,0,0,xs );
			cvmSet( sphere,1,0,ys );
			cvmSet( sphere,2,0,zs );

//			fprintf(trackFile, "%d \t %f \t %f \t %f \t %f \t %f\n", n,featuresB[i].x,featuresB[i].y,cvmGet(sphere,0,0),cvmGet(sphere,1,0),cvmGet(sphere,2,0));
			fprintf(trackFile, "%d \t %f \t %f\n", n,featuresB[i].x,featuresB[i].y);  
		}
		cvShowImage( "window", imgEx );
//		cvShowImage( "box", imgBB );
//		cvShowImage( "corner", imgCorner );

		char c = cvWaitKey( 2 );
		n++;
		cvReleaseImage( &imgColor );
		cvReleaseImage( &pyramid1 );
		cvReleaseImage( &pyramid2 );
		cvReleaseImage( &imgThresh );
		cvReleaseImage( &imgSmooth );
		cvReleaseImage( &pyrA );
		cvReleaseImage( &pyrB );
	}
	cvReleaseImage( &imgEx );
	cvReleaseImage( &img );
	cvReleaseImage( &imgA );
	cvReleaseImage( &imgB );
	cvDestroyWindow( "window" );

	return( 0 );
} 

IplImage* unDistort(IplImage* pic){
	CvMat* intrinsic = cvCreateMat(3,3,CV_64F);
	cvmSet(intrinsic,0,0,4.8259300644658794e+02);
	cvmSet(intrinsic,0,1,0);
	cvmSet(intrinsic,0,2,3.1926568965624580e+02);
	cvmSet(intrinsic,1,0,0);
	cvmSet(intrinsic,1,1,4.8084265086683672e+02);
	cvmSet(intrinsic,1,2,2.3970625670467058e+02);
	cvmSet(intrinsic,2,0,0);
	cvmSet(intrinsic,2,1,0);
	cvmSet(intrinsic,2,2,1);

	CvMat* distortion = cvCreateMat(1,4,CV_64F);
	cvmSet(distortion,0,0,-1.2912475454745747e-04);
	cvmSet(distortion,0,1,-1.2208197693939862e-01);
	cvmSet(distortion,0,2,-1.6728951622689717e-03);
	cvmSet(distortion,0,3,4.6522486953826086e-03);
		cout<<"up to 247\n";
	IplImage* mapx = cvCreateImage( cvGetSize(pic), IPL_DEPTH_32F, 1 );
		cout<<"fail after 247\n";
	IplImage* mapy = cvCreateImage( cvGetSize(pic), IPL_DEPTH_32F, 1 );

	cvInitUndistortMap(intrinsic,distortion,mapx,mapy);

	IplImage *t = cvCloneImage(pic);
	cvRemap( t, pic, mapx, mapy );
	cvReleaseImage(&mapx);
	cvReleaseImage(&mapy);
	cvReleaseImage(&t);

	return pic;
}
