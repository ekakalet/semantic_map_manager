#include "ros/ros.h"
#include <ros/package.h>
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <string>
#include <fstream>
#include <sensor_msgs/CameraInfo.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <cmath>
#include <utility>
#include <stdexcept>
#include <visualanalysis_msgs/CrowdHeatmap.h>
#include <visualanalysis_msgs/DynamicMapAnnotations.h>
#include <multidrone_msgs/GetSemanticMap.h>
#include <multidrone_msgs/PostSemanticAnnotations.h>
#include <sensor_msgs/image_encodings.h>
#include <visualanalysis_msgs/CrowdHeatmap.h>
#include <geometry_msgs/Point32.h>
#include <ctime>
#include <time.h>
#include <clipper/clipper.hpp>
#include "geodetic_conv.hpp"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <multidrone_msgs/CameraStatus.h>
#include <multidrone_msgs/DroneTelemetry.h>
#include <geometry_msgs/PoseStamped.h>
#include "pugixml.hpp"
#include <boost/thread/thread.hpp>
#include "Eigen/Dense"
#include <boost/circular_buffer.hpp>
#include <numeric>

using namespace cv;
using namespace std;
using namespace octomap;
using namespace octomath;
using namespace ClipperLib;
using namespace Eigen;
using namespace message_filters;
using namespace visualanalysis_msgs;
using namespace multidrone_msgs;


std::string sOutputFolder(getenv("SMM_OUTPUT_FOLDER"));
char* NUMBER_POL(getenv("NUMBER_POLYGONS"));

char* NUMBER_VER(getenv("NUMBER_VERTICES"));

int cnt_msg=0;
int No_callROS_Service=0;
bool interface_test = true;

bool value=false;
bool value2=false;
bool value3=false;

#define M_PI           3.14159265358979323846  /* pi */
//declaration of variables


double h,w,fx,W,F, resolution;
double roll,pitch, yaw;
tf::Quaternion quat_tf;

point3d origin, direction;
const tf::Quaternion q;
int kml_file_count=0;
int kml_clipper_count=0;
string path = sOutputFolder;


char *stopstring1, *stopstring2; 

double NUMBER_POLYGONS=strtod(NUMBER_POL, &stopstring1);
//double NUMBER_POLYGONS;
double NUMBER_VERTICES=strtod(NUMBER_VER, &stopstring2);
//double NUMBER_VERTICES;

// sensor_msgs Image message
sensor_msgs::Image msg1;
cv_bridge::CvImagePtr cv_ptr;
visualanalysis_msgs::DynamicMapAnnotations msg_crowd;
visualanalysis_msgs::CrowdHeatmap msg;
geometry_msgs::PoseStamped GEO_origin;



Paths clip= Paths(1);
Paths result=Paths(1);

ros::Time timestamp;


int metric=0;
string path_to_node;
long double time_elapsed_ms, time_elapsed_ms_total=0;

multidrone_msgs::CameraStatus camera_status;
multidrone_msgs::GimbalStatus gimbal_status;
geometry_msgs::PoseStamped drone_pose;


boost::circular_buffer<vector<point3d>>  entire_map(NUMBER_POLYGONS, std::vector<point3d>(NUMBER_VERTICES));

ofstream myTIMES,file, myfile, mypolygons, myTIMES2;

    
cv::Mat src, dst, img, dst_gray, src_gray;

ros::Publisher CrowdAnnotations_pub_merger;
double PerpendicularDistance3D_(point3d &pt,point3d &lineStart,point3d &lineEnd)
{
    double dx = lineEnd.x() - lineStart.x();
    double dy = lineEnd.y() - lineStart.y();

    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag; dy /= mag;
    }

    double pvx = pt.x() - lineStart.x();
    double pvy = pt.y() - lineStart.y();

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker3D_(vector<point3d> &pointList, double epsilon, vector<point3d> &out)
{
    if(pointList.size()<2)
        throw invalid_argument("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance3D_(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
            //cout<< " dmax="<<dmax<<endl;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<point3d> recResults1;
        vector<point3d> recResults2;
        vector<point3d> firstLine(pointList.begin(), pointList.begin()+index+1);
        vector<point3d> lastLine(pointList.begin()+index, pointList.end());
        RamerDouglasPeucker3D_(firstLine, epsilon, recResults1);
        RamerDouglasPeucker3D_(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
            throw runtime_error("Problem assembling output");
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}




                
double PerpendicularDistance3D(IntPoint &pt,IntPoint &lineStart,IntPoint &lineEnd)
{
    double dx = lineEnd.X - lineStart.X;
    double dy = lineEnd.Y - lineStart.Y;

    //Normalise
    double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
    if(mag > 0.0)
    {
        dx /= mag; dy /= mag;
    }

    double pvx = pt.X - lineStart.X;
    double pvy = pt.Y - lineStart.Y;

    //Get dot product (project pv onto normalized direction)
    double pvdot = dx * pvx + dy * pvy;

    //Scale line direction vector
    double dsx = pvdot * dx;
    double dsy = pvdot * dy;

    //Subtract this from pv
    double ax = pvx - dsx;
    double ay = pvy - dsy;

    return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker3D(vector<IntPoint> &pointList, double epsilon, vector<IntPoint> &out)
{
    if(pointList.size()<2)
        throw invalid_argument("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance3D(pointList[i], pointList[0], pointList[end]);
        if (d > dmax)
        {
            index = i;
            dmax = d;
            //cout<< " dmax="<<dmax<<endl;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<IntPoint> recResults1;
        vector<IntPoint> recResults2;
        vector<IntPoint> firstLine(pointList.begin(), pointList.begin()+index+1);
        vector<IntPoint> lastLine(pointList.begin()+index, pointList.end());
        RamerDouglasPeucker3D(firstLine, epsilon, recResults1);
        RamerDouglasPeucker3D(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end()-1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if(out.size()<2)
            throw runtime_error("Problem assembling output");
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList[0]);
        out.push_back(pointList[end]);
    }
}


std::string FormatPolygon_clipper(boost::circular_buffer<vector<point3d>>  &elements)
{
	std::string ss;
    std::ostringstream oss;
    
           for (std::size_t i = 0; i < elements.size(); ++i) {

            oss << "<Placemark>\n"
                << "<name>LinearRing" << i << "</name>\n"
                << "<description>This is a polygon of crowded location</description>\n"
                << "<styleUrl>#msn_ylw-pushpin</styleUrl>\n"

                << "<Polygon>\n"
                //"<LineString>\n"//
                << "<extrude>1</extrude>\n"
                << "<tessellate>1</tessellate>\n"
                << "<altitudeMode>clampToGround</altitudeMode>\n"
                //<< "<innerBoundaryIs>\n"
                << "<outerBoundaryIs>\n"
                << "<LinearRing>\n"
                << "<coordinates>\n";

            
                for (std::size_t j = 0; j < elements[i].size(); j++) {
                     oss << elements[i][j].x()/100000000000000000 << "," << elements[i][j].y()/100000000000000000<< "," << elements[i][j].z()/100000000000000000<< " \n";
                 }                                                                                                                                     
            oss << "</coordinates>\n"
                //  <<"</LineString>\n";
                << "</LinearRing>\n"
                //<< "</innerBoundaryIs>\n"
                << "</outerBoundaryIs>\n"
                << "</Polygon>\n"

		<< "<Style>\n"
 		<< "<LineStyle>\n"
  		<< "<color>ff0000ff</color>\n"
 		<< "</LineStyle>\n"
 		<< "<PolyStyle>\n"
  		<< "<fill>1</fill>\n"
 		<< "</PolyStyle>\n"
                << "</Style>\n"
                << "</Placemark>\n";
                
        
            }
    
    ss = oss.str();
    return ss;

}



void write_clipper_polygon_result_to_kml(Paths &polygon)
{               
                
                
		Paths solution;
		geometry_msgs::Point32 pt;
		point3d point;
		geometry_msgs::PolygonStamped poly;
		ofstream myfile;
		myfile.open ("clipper_result.txt");
                long unsigned int k=0;
                vector<point3d> coord;
		//vector<vector<point3d> > coordinates(polygon.size());
                geometry_msgs::PolygonStamped areas;
		//boost::circular_buffer<geometry_msgs::PolygonStamped>  areas(NUMBER_POLYGONS);
                mypolygons <<polygon.size() % entire_map.capacity()<<"\n"<<endl;  
                
                for (int l=0; l<polygon.size(); l++)
		{
                        
                        k= l % entire_map.capacity();
                        cout<<"result"<<l<<"="<<endl;
                        myfile <<l<<" "<<l<<"\n";
                        

                        //coordinates[l].clear();
                        entire_map[k].clear();
                        for (int i = 0; i <polygon[l].size(); i++)
                        {
							////cout<<polygon[l][i].X<<" "<<polygon[l][i].Y<<endl;
							//myfile <<polygon[l][i].X<<" "<<polygon[l][i].Y<<"\n";
							pt.x=polygon[l][i].X;///100000000000;//maybe we lost decimal digits due to conversion Int to float32
							pt.y=polygon[l][i].Y;///100000000000;
							pt.z=polygon[l][i].Z;//0
							////cout<<"Created vertex polygon "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
							myfile <<polygon[l][i].X<<" "<<polygon[l][i].Y<<" "<<polygon[l][i].Z<<"\n";
							point.x()=pt.x;//in order to be depicted to mymaps
							point.y()=pt.y;//in order to be depicted to mymaps
							point.z()=0;//pt.z;
							//coordinates[l].push_back(point);
                                                        entire_map[k].push_back(point);
                                                        //coordinates[l].push_back(point);
                                                        
                        }
                        pt.x=polygon[l][0].X;///100000000000;//maybe we lost decimal digits due to conversion Int to float32
			pt.y=polygon[l][0].Y;///100000000000;
			pt.z=polygon[l][0].Z;//0
			point.x()=pt.x;//in order to be depicted to mymaps
			point.y()=pt.y;//in order to be depicted to mymaps
			point.z()=0;//pt.z;				
                        //coord.push_back(point);
                        entire_map[k].push_back(point);
                        //coordinates[l].push_back(point);
                       // coordinates[l].push_back(coordinates[l].at(0));// push back the first element in order to create a closed polyline for the polygon
                        myfile <<polygon[l][0].X<<" "<<polygon[l][0].Y<<" "<<polygon[l][0].Z<<"\n";
			cout<<"BEFORE contour size"<<entire_map[k].size()<<endl;
			vector<point3d> pointListOut;
			/*if(coordinates[l].size()>=10)
                			{ //while 650
						//limit 250 : make simplification on the polygons so that MyMaps could display it
						//cout<<"IN_BEFORE_PROJECTION"<<endl;
						RamerDouglasPeucker3D_(coordinates[l],  2.7e+13, pointListOut);//0.008, 6, 2.7e+13,
						coordinates[l]=pointListOut;
                			}*/
                        if(entire_map[k].size()>=10)
                			{ //while 650
						//limit 250 : make simplification on the polygons so that MyMaps could display it
						cout<<"IN_BEFORE_PROJECTION"<<endl;
						RamerDouglasPeucker3D_(entire_map[k],  2.7e+13, pointListOut);//0.008, 6, 2.7e+13,
						entire_map[k]=pointListOut;
                			}
			cout<<"AFTER contour size"<<entire_map[k].size()<<endl;
               } 
                        //prepare coordinates in order to publish on the DynamicAnnotations topic from circular buffer
               for (int l=0; l<entire_map.size(); l++){
                        
                        for (int i = 0; i < entire_map[l].size(); i++)      
                        {                                                    
							pt.x=entire_map[l][i].x();///10000000000000;//<-kmlbased/100000000000;//in order to not lost decimal digits due to conversion Int to float32 the scaling must be done in the receiver 
							pt.y=entire_map[l][i].y();///10000000000000;///100000000000;
							pt.z=0;//coordinates[l][i].z()/10000000000000;///100000000000;//0
							poly.polygon.points.push_back(pt);
						
		        }
                 
                        
                cout<<"Created polygon "<<endl;
            	
                //create the msg polygonstamped_array
                areas=poly;
                poly.polygon.points.clear();
                areas.header.seq=l;
                areas.header.stamp = timestamp;
                areas.header.frame_id ='1';
                msg_crowd.areas.push_back(areas);
                
                
                cout<<l<<"Final************************"<<endl;
	    	}

    
    cout<<"created the msg"<<endl; 
    msg_crowd.header.seq=cnt_msg;
    msg_crowd.header.stamp = timestamp;
    msg_crowd.header.frame_id ='1';
              
    
    CrowdAnnotations_pub_merger.publish(msg_crowd);
    
    msg_crowd.areas.clear();
    
    myfile.close();
    cnt_msg++;
        

	cout<<"Done_created_Coordinates..."<<endl;
	std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
    handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    kml_clipper_count++;
// Open the KML file for writing:
    stringstream filename;
    filename << path <<"//Sample_clipper_result_merger.kml"; //_" << std::setw(4) << std::setfill('0') << kml_clipper_count << ".kml";
    handle.open(filename.str().c_str());
     

    

// Write to the KML file:
     handle << "<?xml version='1.0' encoding='utf-8'?>\n";
     handle << "<kml xmlns='http://www.opengis.net/kml/2.2'\n";
     handle << "xmlns:gx='http://www.google.com/kml/ext/2.2'\n";
     handle << "xmlns:kml='http://www.opengis.net/kml/2.2'\n";
     handle << "xmlns:atom='http://www.w3.org/2005/Atom'>\n";



    
     handle <<"<Document>\n";
     handle   << "<name>crowded location</name>\n";
     handle  << "<Style id='sh_ylw-pushpin'>\n";
     handle             << "<IconStyle>\n";
     handle              << "<scale>1.3</scale>\n";
     handle                          << "<Icon>\n";
     handle                         << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n";
     handle << "</Icon>\n";
     handle  << "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n";
     handle   << "</IconStyle>\n";
     handle  << "<PolyStyle>\n";
     handle  << "<color>ff00ff55</color>\n";
     handle  << "</PolyStyle>\n";
     handle  << "</Style>\n";
     handle  << "<StyleMap id='msn_ylw-pushpin'>\n";
     handle  <<  "<Pair>\n";
     handle  <<  "<key>normal</key>\n";
     handle  <<  "<styleUrl>#sn_ylw-pushpin</styleUrl>\n";
     handle                           << "</Pair>\n";
     handle                             << "<Pair>\n";
     handle                             << "<key>highlight</key>\n";
     handle                             << "<styleUrl>#sh_ylw-pushpin</styleUrl>\n";
     handle                                         << "</Pair>\n";
     handle                                          << "</StyleMap>\n";
     handle                                            << "<Style id='sn_ylw-pushpin'>\n";
     handle                                                      <<"<IconStyle>\n";
     handle                                                      << "<scale>1.1</scale>\n";
     handle                                                                  << "<Icon>\n";
     handle                                                                 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n";
     handle << "</Icon>\n";
     handle<<   "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n";
     handle                                              << "</IconStyle>\n";
     handle                                               << " <PolyStyle>\n";
     handle                                               << "<color>ff00ff55</color>\n";
     handle                                                << "</PolyStyle>\n";
     handle                                                <<"</Style>\n";
       
     handle << FormatPolygon_clipper(entire_map);
              
     handle  << "</Document>\n";

     handle << "</kml>\n";
     handle.close();
        cout << "done writing coordinates to kml..." << endl;
     
       
}

 
void PolygonsProcessingCallback(const visualanalysis_msgs::DynamicMapAnnotationsConstPtr& msg)
{       
	
        
        Paths subj= Paths(msg->areas.size());

        std::clock_t c_start, c_end;
        c_start = std::clock();

 
        ROS_INFO("Call the timestamps from message");
	cout<< "Crowd_polygon Timestamp: "<<msg->header.stamp<<endl;
        
        timestamp=msg->header.stamp;
        //msg.areas.polygon.points
        cout<<"Number of Polygons "<<msg->areas.size()<<endl;
        double NoPolygons=msg->areas.size();
        for  (int q=0; q< NoPolygons; q++)		
	{	
		cout<<"Number of Vertices "<<msg->areas[q].polygon.points.size()<<endl;
                cout<<"polygon1- "<<q<<"************************************"<<endl;
		for (int i=0; i< msg->areas[q].polygon.points.size(); i++)
                {
			
			subj[q].push_back(IntPoint(msg->areas[q].polygon.points[i].x, msg->areas[q].polygon.points[i].y, msg->areas[q].polygon.points[i].z));                      
                        cout<<"vertex "<<subj[q][i] <<endl;
                }
                
	}
        
        c_end = std::clock();
        long double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
        cout<<"Clipper format conversion..."<< time_elapsed_ms<<endl;
	myfile <<"Clipper format conversion..."<<time_elapsed_ms<<"\n";
        time_elapsed_ms_total=time_elapsed_ms_total+time_elapsed_ms;
        c_start = std::clock();
        Clipper c;
	c.AddPaths(subj, ptSubject, true);
	c.AddPaths(clip, ptClip, true);
	c.Execute(ctUnion, result, pftNonZero, pftNonZero);
        c_end = std::clock();
	time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
        cout<<"Calling Clipper..."<< time_elapsed_ms<<endl;
        myfile <<"Calling Clipper..."<<time_elapsed_ms<<"\n"; 
        time_elapsed_ms_total=time_elapsed_ms_total+time_elapsed_ms;
        //mypolygons <<result.size()<<"\n"<<endl;
        c_start = std::clock();
        for (int l=0; l< result.size(); l++)
        {
        cout<< "Number of Polygons Result: "<<result.size()<<endl;
        cout<<"BEFORE contour size Result "<<result[l].size()<<endl;
	Path pointListOut=Path(1);
        if(result[l].size()>=10)
               		{ //while 650
				//limit 250 : make simplification on the polygons so that MyMaps could display it
				cout<<"IN_BEFORE_PROJECTION"<<endl;
				RamerDouglasPeucker3D(result[l],  2.7e+13, pointListOut);//0.008, 6, 2.7e+13,
				result[l].clear();
                                result[l]=pointListOut;
               		}
	cout<<"AFTER contour size Result "<<result[l].size()<<endl;
        cout<<"Number of Polygons MSG "<<msg->areas.size()<<endl;
        //for (int i=0; i< result[l].size(); i++)
        //{              
        //       cout<<"vertex result "<<result[l][i] <<endl;
        //}
        //pointListOut.clear();
        }

       
    	write_clipper_polygon_result_to_kml(result);
        //clip.clear();
        clip=result;
        //result.clear();
       
	c_end = std::clock();

     time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
     
        cout<<"Preparation for publishing..."<< time_elapsed_ms<<endl;
        myfile <<"Preparation for publishing..."<<time_elapsed_ms<<"\n"; 
         //time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
  time_elapsed_ms_total=time_elapsed_ms_total+time_elapsed_ms;
  myTIMES2 <<time_elapsed_ms<<"\n"<<endl;  
   
  time_elapsed_ms_total=0;
     
}

		





/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////2D_simplification//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////

double PerpendicularDistance2D(const Point &pt, const Point &lineStart, const Point &lineEnd)
{
	double dx = lineEnd.x - lineStart.x;
	double dy = lineEnd.y - lineStart.y;
 
	//Normalise
	double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
	if(mag > 0.0)
	{
		dx /= mag; dy /= mag;
	}
 
	double pvx = pt.x - lineStart.x;
	double pvy = pt.y - lineStart.y;
 
	//Get dot product (project pv onto normalized direction)
	double pvdot = dx * pvx + dy * pvy;
 
	//Scale line direction vector
	double dsx = pvdot * dx;
	double dsy = pvdot * dy;
 
	//Subtract this from pv
	double ax = pvx - dsx;
	double ay = pvy - dsy;
 
	return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}
 

void RamerDouglasPeucker2D(const vector<Point> &pointList, double epsilon, vector<Point> &out)
{
	if(pointList.size()<2)
		throw invalid_argument("Not enough points to simplify");
 
	// Find the point with the maximum distance from line between start and end
	double dmax = 0.0;
	size_t index = 0;
	size_t end = pointList.size()-1;
	for(size_t i = 1; i < end; i++)
	{
		double d = PerpendicularDistance2D(pointList[i], pointList[0], pointList[end]);
		if (d > dmax)
		{
			index = i;
			dmax = d;
		}
	}
 
	// If max distance is greater than epsilon, recursively simplify
	if(dmax > epsilon)
	{
		// Recursive call
		vector<Point> recResults1;
		vector<Point> recResults2;
		vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
		vector<Point> lastLine(pointList.begin()+index, pointList.end());
		RamerDouglasPeucker2D(firstLine, epsilon, recResults1);
		RamerDouglasPeucker2D(lastLine, epsilon, recResults2);
 
		// Build the result list
		out.assign(recResults1.begin(), recResults1.end()-1);
		out.insert(out.end(), recResults2.begin(), recResults2.end());
		if(out.size()<2)
			throw runtime_error("Problem assembling output");
	} 
	else 
	{
		//Just return start and end points
		out.clear();
		out.push_back(pointList[0]);
		out.push_back(pointList[end]);
	}
}




//-----------------------------------------------------------------------------------------------------------


std::string FormatPolygonCoordinates(vector<vector<point3d> > &elements){
    std::string ss;
    std::ostringstream oss;
	//oss << sort_in_counter_clockwise( elements);
	ss = oss.str();
	return ss;

}

std::string FormatPolygon(vector<vector<point3d> > &elements)
{
    std::ostringstream ss;
    ss <<"<Document>\n"
       << "<name>crowded location</name>\n"
       //<< "<Style>\n"
       //<< "<LineStyle>\n"
       //<<"<color>#ff0000ff</color>\n"
       //<< "<width> 5 </width>\n"
       //<< "</LineStyle>\n"
       //<< "</Style>\n"

       << "<Style id='sh_ylw-pushpin'>\n"
                  << "<IconStyle>\n"
                   << "<scale>1.3</scale>\n"
                               << "<Icon>\n"
                               << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n"
    << "</Icon>\n"
       << "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n"
        << "</IconStyle>\n"
       << "<PolyStyle>\n"
       << "<color>ff00ff55</color>\n"
       << "</PolyStyle>\n"
       << "</Style>\n"
       << "<StyleMap id='msn_ylw-pushpin'>\n"
       <<  "<Pair>\n"
       <<  "<key>normal</key>\n"
       <<  "<styleUrl>#sn_ylw-pushpin</styleUrl>\n"
                                << "</Pair>\n"
                                  << "<Pair>\n"
                                  << "<key>highlight</key>\n"
                                  << "<styleUrl>#sh_ylw-pushpin</styleUrl>\n"
                                              << "</Pair>\n"
                                               << "</StyleMap>\n"
                                                 << "<Style id='sn_ylw-pushpin'>\n"
                                                           <<"<IconStyle>\n"
                                                           << "<scale>1.1</scale>\n"
                                                                       << "<Icon>\n"
                                                                      << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n"
    << "</Icon>\n"
    <<   "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n"
                                                   << "</IconStyle>\n"
                                                    << " <PolyStyle>\n"
                                                     << "<color>ff00ff55</color>\n"
                                                     << "</PolyStyle>\n"
                                                     <<"</Style>\n"
       //<< "<Placemark>\n"
       //<< "<name>LinearRing.kml</name>\n"
       //<< "<description>This is a polygon of crowded location</description>\n"
       //<< "<styleUrl>#msn_ylw-pushpin</styleUrl>\n"
       //<< "<Polygon>\n"
          //for (int i = 0; i < coordinates.size(); i++) {
           //   for (int j = 0; j < coordinates[i].size(); j++) { ;
       << FormatPolygonCoordinates(elements)<<"\n"
                   // }
          //}
      // <<"</Polygon>\n"
      // << "</Placemark>\n"
       << "</Document>\n";

    return ss.str();
}
 
void write_to_kml(vector<vector<point3d> > &coordinates){
    std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
    handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    kml_file_count++;
    // Open the KML file for writing:
    stringstream filename;
    filename << path <<"//Sample_"<< ".kml"; //<< std::setw(4) << std::setfill('0') << kml_file_count << ".kml";
    handle.open(filename.str().c_str());
     
// Write to the KML file:
    handle << "<?xml version='1.0' encoding='utf-8'?>\n";
    handle << "<kml xmlns='http://www.opengis.net/kml/2.2'\n";
    handle << "xmlns:gx='http://www.google.com/kml/ext/2.2'\n";
    handle << "xmlns:kml='http://www.opengis.net/kml/2.2'\n";
    handle << "xmlns:atom='http://www.w3.org/2005/Atom'>\n";


    handle << FormatPolygon(coordinates);



    handle << "</kml>\n";
    handle.close();

}
 // roll (x-axis rotation)
// pitch (y-axis rotation)
// yaw (z-axis rotation)

//-----------------------------------------------------------------------------------------------------------------------------------

void print_info(double NUMBER_POLYGONS, double NUMBER_VERTICES){
  cout << "NUMBER_POLYGONS: " << NUMBER_POLYGONS<<endl;
  cout << "NUMBER_VERTICES: " << NUMBER_VERTICES<<endl;
}


 
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "merger");

 
  ros::NodeHandle nh;
  myfile.open ("Times_Merger.txt");
  mypolygons.open ("Number_of_polygons.txt");
  myTIMES2.open ("Times_POL_MERGING.txt");

  nh.getParam("/NUMBER_POLYGONS", NUMBER_POLYGONS);
  nh.getParam("/NUMBER_VERTICES", NUMBER_VERTICES);

  print_info(NUMBER_POLYGONS, NUMBER_VERTICES);
  
  ros::Subscriber sub = nh.subscribe("visual_analysis/dynamic_map_annotations_not_fused", 10, PolygonsProcessingCallback);
  CrowdAnnotations_pub_merger = nh.advertise<visualanalysis_msgs::DynamicMapAnnotations>("/visual_analysis/dynamic_map_annotations", 1000);

  

 
  ros::spin();
  myfile.close();
  myTIMES2.close();
  mypolygons.close();
  return 0;
}
