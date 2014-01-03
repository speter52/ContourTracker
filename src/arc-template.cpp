/*
 * arc-template.cpp
 * Martin Miller
 * 12/31/2013
 * Uses ViSP template tracking to track contours.
 */
#include "arc-template.hpp"

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
    unsigned index=0;
    vpTemplateTrackerWarpHomography warp;
    std::string listname;
    std::vector<std::string> images;

    std::vector<vpTemplateTrackerSSDInverseCompositional*> c;
    std::vector<unsigned> in;
    vpImage<unsigned char> I;
    vpDisplayX display;

    listname = argv[1];
    getImageList( listname, &images );

    vpImageIo::read(I, images[0]);
    display.init(I, 100, 100, "Template tracker");
    vpDisplay::display(I);
    vpDisplay::flush(I);

    // Select N contours.
    std::cout << "Left click the points of the contour, "
              << "right-click the final point." << std::endl;
    char line[5];
    do 
    {
        vpTemplateTrackerSSDInverseCompositional* temp=new vpTemplateTrackerSSDInverseCompositional(&warp);
        temp->setSampling(2,2);
        temp->setLambda(0.001);
        temp->setIterationMax(200);
        temp->setPyramidal(2, 1);
        temp->initClick( I, true );
        c.push_back( temp );
        in.push_back( index++ );

        std::cout << "Press enter to continue adding contours. "
                  << "Any other key followed by enter when done.";
        fgets(line, 4, stdin);
    } while( line[0]=='\n' );

    for( std::vector<std::string>::iterator img=images.begin();
            img!=images.end(); ++img )
    {
        vpImageIo::read(I, *img);
        vpDisplay::display(I);
        // Add more contours if necessary
        vpDisplay::flush(I);
        while( c.size()<ARC_MIN_CONTOURS )
        {
            vpTemplateTrackerSSDInverseCompositional* temp=new vpTemplateTrackerSSDInverseCompositional(&warp);
            temp->setSampling(2,2);
            temp->setLambda(0.001);
            temp->setIterationMax(200);
            temp->setPyramidal(2, 1);
            temp->initClick( I, true );
            c.push_back( temp );
            in.push_back( index++ );
        }

        std::vector<unsigned>::iterator index=in.begin();
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
                if( i==3 || i==4 ) continue; // True for quadrilaterals
                if( zone_points[i].x > ARC_WIDTH ||
                        zone_points[i].x < 0 ||
                        zone_points[i].y > ARC_HEIGHT ||
                        zone_points[i].y < 0 )
                {
                    std::cerr << "Point out of frame. Removing contour." << std::endl;
                    con = c.erase(con);
                    index = in.erase(index);
                    removed=true;
                    break;
                }
            }
            if( removed==true ) continue;
            unsigned found = img->find_last_of("/");
            std::cout << img->substr(found+1) << " "
                      << *index << " " 
                      << zone_points[0].x << " " 
                      << zone_points[0].y << " "
                      << zone_points[1].x << " " 
                      << zone_points[1].y << " "
                      << zone_points[2].x << " " 
                      << zone_points[2].y << " "
                      << zone_points[5].x << " " 
                      << zone_points[5].y << std::endl;
            (**con).display(I, vpColor::red);
            ++index;
            ++con;
        }
        vpDisplay::flush(I);
        vpTime::wait(40);
    }
}
