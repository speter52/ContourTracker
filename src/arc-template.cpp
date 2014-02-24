/*
 * arc-template.cpp
 * Martin Miller
 * 12/31/2013
 * Uses ViSP template tracking to track contours.
 */
#include "arc-template.hpp"
#include "ARC_FindContours.hpp"

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  pad
 *  Description:  Pads a contour of a quadrilateral.
 * =====================================================================================
 */
    void
pad ( std::vector<cv::Point>& con, int N, cv::Size sz )
{
    cv::Rect br;
    cv::Vec2i center;
    cv::Vec2f unit;

    if( con.size()!=4 )
        std::cerr << "Padded contour not a quadrilateral." << std::endl;

    // Create a rect the size of the frame to prevent extending outside of frame.
    // Bound the contour by a rect, then shift and expand to pad by N.
    br = boundingRect( con );
    center = cv::Vec2i( br.tl() ) + cv::Vec2i(br.width/2, br.height/2);
    for( std::vector<cv::Point>::iterator pt=con.begin();
            pt!=con.end(); ++pt )
    {
        unit = cv::Vec2i(*pt)-center;
        unit = (1/norm(unit)) * unit;
        *pt += cv::Point(N * unit);
    }
    return;
}		/* -----  end of function pad  ----- */

/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  getImageList
 *  Description:  Reads in list of image filenames.
 * =====================================================================================
 */
void getImageList( std::string filename,  std::vector<std::string>* il )
{

    std::string    ifs_file_name = filename;         /* input  file name */
    std::ifstream  ifs;                              /* create ifstream object */

    ifs.open (  ifs_file_name.c_str( ) );         /* open ifstream */
    if ( !ifs ) {
        std::cerr << "\nERROR : failed to open input  file " << ifs_file_name << std::endl;
        exit ( EXIT_FAILURE );
    }
    std::string line;
    while(  getline( ifs,  line,  '\n') )
    {
        il->push_back( line );
    }
    ifs.close ( );                                 /* close ifstream */
}

int main(int argc, char** argv)
{
    bool mouse, video, image_out;
    std::string img_dir;
    unsigned int count;
    unsigned index=0;
    vpTemplateTrackerWarpHomography warp;
    std::string listname;
    std::vector<std::string> images;
    ARC_FindContours fc;
    std::vector<std::vector<cv::Point> > quads;
    cv::Mat firstFrame;

    std::vector<vpTemplateTrackerSSDInverseCompositional*> c;
    std::vector<unsigned> in;
    vpImage<unsigned char> I;
    vpDisplayX display;
    cv::VideoWriter vidout;

    mouse = false;
    video = false;
    image_out = false;
    for( int i=1; i<argc; ++i )
    {
        if( !strcmp(argv[i], "-m") )
        {
            mouse=true;
        }
        else if( !strcmp(argv[i], "-v") )
        {
            video=true;
        }
        else if( !strcmp(argv[i], "-i") )
        {
            img_dir = argv[++i];
            image_out = true;
        }
        else if( !strcmp(argv[i], "-h") )
        {
            std::cout << "Usage: " << argv[0] << std::endl
                << "-m\tEnable mouse clicking" << std::endl
                << "-v\tOutput video to out.avi" << std::endl
                << "-i <outdir>\tOutput images to outdir" << std::endl
                ;
            exit( EXIT_SUCCESS );
        }
    }
    listname = argv[1];
    getImageList( listname, &images );

    firstFrame = cv::imread( images[0], CV_LOAD_IMAGE_UNCHANGED );
    if( video )
    {
        vidout.open( std::string("out.avi"), CV_FOURCC('F','M','P','4'), 20.0, firstFrame.size(), true);
        if( !vidout.isOpened() )
        {
            std::cerr << "Could not open video file out.avi" << std::endl;
            exit( EXIT_FAILURE );
        }
    }

    fc.squares( firstFrame, quads, quads );
    drawContours( firstFrame, quads, -1, cv::Scalar( 0, 255, 0) );
    //cv::imshow( "test", firstFrame );
    //cv::waitKey( 10 );
    vpImageIo::read(I, images[0]);
    display.init(I, 100, 100, "Template tracker");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    if( mouse )
    {
        // Select N contours.
        std::cout << "Left click the points of the contour, "
                  << "right-click the final point." << std::endl;
        char line[5];
        do 
        {
            std::vector<vpImagePoint> points;
            vpTemplateTrackerSSDInverseCompositional* temp=new vpTemplateTrackerSSDInverseCompositional(&warp);
            temp->setSampling(2,2);
            temp->setLambda(0.001);
            temp->setIterationMax(200);
            temp->setPyramidal(2, 1);
            temp->initClick( I, true );

            // Get points
            vpTemplateTrackerZone t_zone;
            vpTemplateTrackerZPoint* zone_points;
            vpColVector p = temp->getp();
            warp.warpZone(t_zone, p);
            zone_points = t_zone.getListPtWarpes();

            std::vector<cv::Point> temp_cont;
            for( size_t i=0; i<6; ++i )
            {
                if( i==3||i==4 ) continue;
                cv::Point pt(zone_points->x, zone_points->y);
                temp_cont.push_back(pt);
            }
            quads.push_back( temp_cont );

            c.push_back( temp );
            in.push_back( index++ );

            std::cout << "Press enter to continue adding contours. "
                      << "Any other key followed by enter when done.";
            fgets(line, 4, stdin);
        } while( line[0]=='\n' );

    }
    else
    {
        for( std::vector<std::vector<cv::Point> >::iterator q=quads.begin();
                q!=quads.end(); ++q )
        {
            std::vector<vpImagePoint> points;
            pad( *q, 5, cv::Size(ARC_WIDTH, ARC_HEIGHT) );
            vpTemplateTrackerSSDInverseCompositional* temp=new vpTemplateTrackerSSDInverseCompositional(&warp);
            temp->setSampling(2,2);
            temp->setLambda(0.001);
            temp->setIterationMax(200);
            temp->setPyramidal(2, 1);
            for( std::vector<cv::Point>::iterator p=q->begin();
                    p!=q->end(); ++p )
            {
                // Note ij notation used, not xy
                points.push_back( vpImagePoint( p->y, p->x ) );
            }
            temp->initFromPoints( I, points, true );
            c.push_back( temp );
            in.push_back( index++ );
        } 
    }

    count=0;
    for( std::vector<std::string>::iterator img=images.begin();
            img!=images.end(); ++img, ++count )
    {
        cv::Mat frame;
        frame = cv::imread( *img, CV_LOAD_IMAGE_UNCHANGED );
        vpImageIo::read(I, *img);
        vpDisplay::display(I);
        // Add more contours if necessary
        vpDisplay::flush(I);
        //if( c.size()<ARC_MIN_CONTOURS )
        if( count%20==0 )
        {
            if( mouse )
            {
                while( c.size()<ARC_MIN_CONTOURS )
                {
                    std::cout << "Select contour." << std::endl;
                    vpTemplateTrackerSSDInverseCompositional* temp=new vpTemplateTrackerSSDInverseCompositional(&warp);
                    temp->setSampling(2,2);
                    temp->setLambda(0.001);
                    temp->setIterationMax(200);
                    temp->setPyramidal(2, 1);
                    temp->initClick( I, true );
                    
                    // Get points
                    vpTemplateTrackerZone t_zone;
                    vpTemplateTrackerZPoint* zone_points;
                    vpColVector p = temp->getp();
                    warp.warpZone(t_zone, p);
                    zone_points = t_zone.getListPtWarpes();

                    std::vector<cv::Point> temp_cont;
                    for( size_t i=0; i<6; ++i )
                    {
                        if( i==3||i==4 ) continue;
                        cv::Point pt(zone_points->x, zone_points->y);
                        temp_cont.push_back(pt);
                    }
                    quads.push_back( temp_cont );

                    c.push_back( temp );
                    in.push_back( index++ );
                }
            }
            else
            {
                std::vector<std::vector<cv::Point> > new_quads;
                fc.squares( frame, quads, new_quads );
                for( std::vector<std::vector<cv::Point> >::iterator q=new_quads.begin();
                        q!=new_quads.end(); ++q )
                {
                    std::vector<vpImagePoint> points;
                    vpTemplateTrackerSSDInverseCompositional* temp=new vpTemplateTrackerSSDInverseCompositional(&warp);
                    temp->setSampling(2,2);
                    temp->setLambda(0.001);
                    temp->setIterationMax(200);
                    temp->setPyramidal(2, 1);
                    pad( *q, 5, cv::Size(ARC_WIDTH, ARC_HEIGHT) );
                    for( std::vector<cv::Point>::iterator p=q->begin();
                            p!=q->end(); ++p )
                    {
                        // Note ij notation used, not xy
                        points.push_back( vpImagePoint( p->y, p->x ) );
                    }
                    temp->initFromPoints( I, points, true );
                    c.push_back( temp );
                    in.push_back( index++ );
                    quads.push_back( *q );
                }
            }
        }

        std::vector<unsigned>::iterator index=in.begin();
        std::vector<std::vector<cv::Point> >::iterator q=quads.begin();
        std::vector<vpTemplateTrackerSSDInverseCompositional*>::iterator con=c.begin();
        while( con!=c.end() )
        {
            vpTemplateTrackerZone t_zone;
            vpTemplateTrackerZPoint* zone_points;
            
            t_zone = (**con).getZoneWarp();
            try {
                (**con).track(I);
            }
            catch ( vpTrackingException &ExceptObj ) {		/* handle exception: */
                std::cerr << ExceptObj.what() << std::endl;
                con = c.erase(con);
                index = in.erase(index);
                q=quads.erase(q);
                continue;
            }
            catch (...) {		/* handle exception: unspecified */
                std::cerr << "Unspecified Error" << std::endl;
                exit(EXIT_FAILURE);
            }
            vpColVector p = (**con).getp();
            warp.warpZone(t_zone, p);
            zone_points = t_zone.getListPtWarpes();
            bool removed=false;
            for( int i=0; i<6; ++i )
            {
                bool duplicate_points=false;
                for( int j=i+1; j<6; ++j )
                {
                    if( j==3 || j==4 ) continue;
                    if( zone_points[i].x==zone_points[j].x &&
                            zone_points[i].y==zone_points[j].y )
                    {
                        duplicate_points=true;
                    }
                }
                if( i==3 || i==4 ) continue; // True for quadrilaterals
                if( duplicate_points ||
                        zone_points[i].x > ARC_WIDTH ||
                        zone_points[i].x < 0 ||
                        zone_points[i].y > ARC_HEIGHT ||
                        zone_points[i].y < 0 )
                {
                    std::cerr << "Point out of frame. Removing contour." << std::endl;
                    con = c.erase(con);
                    index = in.erase(index);
                    q=quads.erase(q);
                    removed=true;
                    break;
                }
            }
            if( removed==true ) continue;
            unsigned found = img->find_last_of("/");
            unsigned ext = img->find_last_of(".");
            std::cout << img->substr(found+1, ext-found-1) << " "
                      << *index << " " 
                      << zone_points[0].x << " " 
                      << zone_points[0].y << " "
                      << zone_points[1].x << " " 
                      << zone_points[1].y << " "
                      << zone_points[2].x << " " 
                      << zone_points[2].y << " "
                      << zone_points[5].x << " " 
                      << zone_points[5].y << std::endl;

            (*q)[0] = cv::Point( zone_points[0].x, zone_points[0].y );
            (*q)[1] = cv::Point( zone_points[1].x, zone_points[1].y );
            (*q)[2] = cv::Point( zone_points[2].x, zone_points[2].y );
            (*q)[3] = cv::Point( zone_points[5].x, zone_points[5].y );

            (**con).display(I, vpColor::red);
            // Draw onto frame (for opencv processing and display)
            ++index;
            ++con;
            ++q;
        }
        vpDisplay::flush(I);
        //vpTime::wait(40);
        if( image_out || video )
        {
            // use convexHull since we don't know the order of the points
            for(std::vector<std::vector<cv::Point> >::iterator q=quads.begin();
                    q!=quads.end(); ++q )
            {
                std::vector<cv::Point> hull;
                convexHull( *q, hull, true, true );
                *q=hull;
            }
            cv::drawContours( frame, quads, -1, cv::Scalar(0,255,0), 1, CV_AA );
            if( image_out )
            {
                std::string fn;
                unsigned bn_start, dir_last;
                dir_last = img_dir.length();
                if( img_dir[dir_last-1]!='/' ) img_dir.append("/");
                
                bn_start = img->find_last_of("/");
                fn = img_dir + img->substr(bn_start+1);
                cv::imwrite( fn, frame );
            }
            if( video )
            {
                vidout << frame;
            }

            cv::imshow("IMG", frame);
            cv::waitKey(10);
        }
    }
}
