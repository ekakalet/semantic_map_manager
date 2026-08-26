#include "ros/ros.h"
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
//#include <octomap_msgs/GetOctomap.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
//#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
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
#include <visualanalysis_msgs/PolygonStamped_array.h>
#include <multidrone_msgs/KMLResponse.h>
#include <sensor_msgs/image_encodings.h>
#include <visualanalysis_msgs/TimedFrame.h>
#include <geometry_msgs/Point32.h>
#include <ctime>
#include <time.h>
#include "clipper.hpp"
//#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <multidrone_msgs/CameraStatus.h>
#include <multidrone_msgs/DroneTelemetry.h>

using namespace cv;
using namespace std;
using namespace octomap;
using namespace octomath;
using namespace ClipperLib;
using namespace message_filters;
using namespace visualanalysis_msgs;
using namespace multidrone_msgs;


#define M_PI           3.14159265358979323846  /* pi */
//declaration of variables

double h,w,fx,W,F, resolution;
double roll,pitch, yaw;
double cx; //principal point in m
double cy; //principal point in m
point3d origin, direction;
const tf::Quaternion q;
int kml_file_count=0;
int kml_clipper_count=0;

string btFilename;
octomap::OcTree* octree = new octomap::OcTree("Maps/square_1.binvox.bt"); ///home/ekakalet/Documents/LandscapeMap_FLAT.binvox.bt
ColorOcTree* ctree=new ColorOcTree(octree->getResolution()); 
//octomap::OcTree* octree;
//ColorOcTree* ctree;
octomap::AbstractOcTree* tree;

int threshold_value = 0;
int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

// sensor_msgs Image message
sensor_msgs::Image msg1;
cv_bridge::CvImagePtr cv_ptr;
visualanalysis_msgs::PolygonStamped_array msg2;
visualanalysis_msgs::TimedFrame msg;
geographic_msgs::GeoPoint GEO_origin;
ros::Publisher CrowdAnnotations_pub;
Paths subj(1), clip(1), result(1);

int metric=0;

ofstream myTIMES;
    


cv::Mat src, dst, img, dst_gray, src_gray;
string window_name = (char *)"Threshold Demo";

vector<vector<point3d> > polygons_coord;

string trackbar_type = (char *)"Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
string trackbar_value = (char *)"Value";



/// Function headers
void Threshold_Demo( int, void* );
void find_contours_and_project(cv::Mat dst, OcTree* octree , ColorOcTree* ctree, double w, double h, double F, double roll, double pitch, double yaw, vector<vector<point3d> > &polygons_coord);
std::string FormatPolygon(vector<point3d> elements);
void write_to_kml(vector<point3d> coordinates);
std::string FormatPolygonCoordinates(vector<point3d> elements);
void RamerDouglasPeucker(vector<point3d> &pointList, double epsilon, vector<point3d> &out);
double PerpendicularDistance(point3d &pt,point3d &lineStart,point3d &lineEnd);
void rotation_based_in_euler_angles(double roll,double pitch,double yaw);
static void toEulerAngle(double X, double Y, double Z, double G , double& roll, double& pitch, double& yaw);
void fusion_polygon_lines(Paths poly);
void initialization_of_clipper_polygons();
void write_clipper_polygon_result_to_kml(Paths result);
std::string FormatPolygon_clipper(vector<vector<point3d> > elements);
void quaternion_to_euler_angles(const tf::Quaternion &q);

//void read_camera_settings();

//void CrowdProcessingCallback(const visualanalysis_msgs::CrowdHeatmapConstPtr& msg)
void CrowdProcessingCallback(const visualanalysis_msgs::CrowdHeatmapConstPtr& msg, const multidrone_msgs::GimbalStatusConstPtr& gimbal_msg, const multidrone_msgs::CameraStatusConstPtr& camerastatus_msg, const multidrone_msgs::DroneTelemetryConstPtr& dronetelemetry_msg)
{
	
        ROS_INFO("Create a Copy of the crowd heat map from message");
        cout<<"Gimbal Status Message: Roll: "<< gimbal_msg->roll<<" Pitch: "<<gimbal_msg->pitch<<" Yaw: "<< gimbal_msg->yaw <<endl;
	cout<< "Timeframe     Timestamp: "<<msg->header.stamp<<endl;
        cout<< "Gimbal Status Timestamp: "<<gimbal_msg->header.stamp<<endl;
        cout<< "Camera Status Timestamp: "<<camerastatus_msg->header.stamp<<endl;
        cout<< "Drone Telemetry Timestamp: "<<dronetelemetry_msg->header.stamp<<endl;

	//cv_bridge::CvImagePtr cv_ptr;
    	try
    	{
      	//ROS_INFO("Frame with timestamp: %d received", msg.meta.utctime);

	// copy sensor_msgs::Image
        msg1 = msg->heatmap;

	// convert sensor_msgs::Image to cv_bridge::CvImagePtr, pointer to image
	cv_bridge::CvImagePtr cv_ptr;

	cv_ptr = cv_bridge::toCvCopy(msg1, "mono8");
    /*    convert to something visible
	double min_range_=0;
	double max_range_=255;

    	cv::Mat img(cv_ptr->image.rows,cv_ptr->image.cols, CV_8UC1);
    	for(int i = 0; i < cv_ptr->image.rows; i++)
    	{
        float* Di = cv_ptr->image.ptr<float>(i);
        char* Ii = img.ptr<char>(i);
        //imshow ("img", img);
	//cv::waitKey(1);
        for(int j = 0; j < cv_ptr->image.cols; j++)
        {
            Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
        }
	}*/
        //cv::Mat src = img;


        //cv::Mat src = imread("//home//ekakalet//Documents//1527598067//2.png", IMREAD_GRAYSCALE);
	//cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::8UC1);
	

	// convert object to cv::Mat
	cv::Mat src = cv_ptr->image;

	//cv::Mat beConvertedMat(img.rows, img.cols, CV_8UC4, img.data);
	//imshow ("img", src);
	//cv::waitKey(1);
	//check that image has been delivered correctly
	//imwrite("/home/ekakalet/Documents/test.jpg", src);

        if (!src.empty())
        {
         	//namedWindow( "img", WINDOW_AUTOSIZE );
                //imshow ("img", src);
		//cv::waitKey(33);
        }
        
////////////////////////////////////////////////////////////////////////////////////////////
      	//cv_ptr = cv_bridge::toCvCopy(msg->heatmap, sensor_msgs::image_encodings::BGR8);
      	//src = cv_ptr->image;
      	//src = imread("//home//ekakalet//Programms//HEATMAPS//DATWOTESTa5_new.png", IMREAD_GRAYSCALE);
      	ROS_INFO("Done imread");
        //Convert the image to Gray
        src_gray=src;
///////////////////////////////cvtColor( src, src_gray, CV_BGR2GRAY );
/////////////////////////////////////////////////////////////////////////////////////////////
    	}
    	catch (cv_bridge::Exception& e)
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	}
std::clock_t c_start, c_end;
c_start = std::clock();
        
    	ROS_INFO("Ready for processing the crowd heatmap");
        // Create a window to display results
    	/////////////namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    	/// Create Trackbar to choose type of Threshold
    	/////////////////createTrackbar( trackbar_type,
        ////////////////            window_name, &threshold_type,
        //////////////////            max_type, Threshold_Demo );

    	///////////////createTrackbar( trackbar_value,
        /////////////            window_name, &threshold_value,
         ////////////////           max_value, Threshold_Demo );

    	/// Call the function to initialize
    	Threshold_Demo( 0, 0 );

    	/// Wait until user finishes program

    	/*while(true)
    	{


        int c;
        c = waitKey( 20 );
        if( (char)c == 27 )
        {
            break; }



    	}*/
    	cout<<"end of thresholding"<<endl;
    	//////////////////////from camera status, gimbal status//////////////////////////////
    	c_end = std::clock();
long double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
    cout<<endl;
    cout<<"end of thresholding"<<time_elapsed_ms<<endl;
    myTIMES.open ("times.txt"); 
    myTIMES <<"end of thresholding time "<<time_elapsed_ms<<"\n";
    cout<<endl;
	// roll (x-axis rotation)
	// pitch (y-axis rotation)
	// yaw (z-axis rotation)

        GEO_origin=dronetelemetry_msg->geo_position;
        //GEO_origin.latitude=GEO_origin.latitude+mount_position[0];
        //GEO_origin.longitude=GEO_origin.longitude+mount_position[1];
        //GEO_origin.altitude=GEO_origin.altitude+mount_position[2];
        origin=point3d(GEO_origin.latitude, GEO_origin.longitude, GEO_origin.altitude);
        w=dst.cols * 36*0.001/1920;//width in m according to sensor size width of blackmagic: 12.48*0.001/1920
    	h=dst.rows * 20.25*0.001/1080;//height in m according to sensor size height of blackmagic: 7.02*0.001/1080
    	cout<<"w="<<w<<" h="<<h<<endl;
        F=camerastatus_msg->focalLength*0.001;//0.018;//initial value 0.008,  FL-Range=14mm-42mm;
    	//direction=point3d(-cx, -cy, -F);//cx-w/2, cy-h/2, -F
        resolution=octree->getResolution();
	cx=0;//(900.259497081249)*0.0065*0.001; //principal point in m
	cy=0;//(590.2505880129497)*0.0065*0.001; //principal point in m
        cout<<"cx="<<cx<<" cy="<<cy<<endl;
    	roll=(M_PI /180)*gimbal_msg->roll;
    	pitch=(M_PI /180)*gimbal_msg->pitch;
    	yaw=(M_PI /180)*gimbal_msg->yaw;

        cout<<"roll pitch yaw" <<gimbal_msg->roll <<" "<<gimbal_msg->pitch <<" "<< gimbal_msg->yaw<< endl;



point3d p= point3d(-21.9/2, -29.9/2, 0.0);//40.00f, 40.0f, 60.3f




		
   cout << "generating single rays..." << endl;
   OcTree single_beams=*octree;
   int num_beams = 17;
   float beamLength = 6.0f;
   point3d single_origin =p;
   point3d single_origin_top =p;
   point3d single_endpoint(beamLength, 0.0f, 0.0f);

         
   for (int i=0; i<num_beams; i++) {
     for (int j=0; j<num_beams; j++) {
       if (!single_beams.insertRay(single_origin, single_origin+single_endpoint)) {
         cout << "ERROR while inserting ray from " << single_origin << " to " << single_endpoint << endl;
       }
       single_endpoint.rotate_IP (0,0,DEG2RAD(360.0/num_beams));
     }
     single_endpoint.rotate_IP (0,DEG2RAD(360.0/num_beams),0);
   }

         
   cout << "done." << endl;
   cout << "writing to beams.bt..." << endl;
   (single_beams.writeBinary("/home/ekakalet/ros/src/ground_visual_analysis/src/Maps/beams.bt"));




/*

	if (metric==0){
        origin=point3d(1061.64, -636.219, 4339.94);//44000.0f, 13000.0f, 86000.0f//75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//testAirSim: 
	roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
        myTIMES.open ("times.txt");
        
        //origin=point3d(38000.0f, 12000.0f, 86000.0f);////75000.0f, 75000.0f, 28000.0f//40.0f, 40.0f, 33.0f
        //tf::Quaternion q(0.002,  0.807,   -0.269,  0.525);
        //quaternion_to_euler_angles(q);
        //roll=(M_PI /180)*45;
    	//pitch=0;
    	//yaw=0;//(M_PI /180)*90;







       }
        else if(metric==1){
        origin=point3d(1661.64, -636.219, 4339.94);//44000.0f, 13000.0f, 86000.0f//75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//38000.0f, 12000.0f, 86000.0f
        //tf::Quaternion q(0, 0.7071 , 0 , 0.7071); //rotate by 90 degrees about y axis
        //quaternion_to_euler_angles(q);        	
	 roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
        //roll=0;
    	//pitch=0;
        //yaw=0;  
        myTIMES.open ("times.txt");
	
	}
        else if(metric==2){
        origin=point3d(1970.64, -636.219, 4339.94 );//42000.0f, 13000.0f, 86000.0f//75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//38000.0f, 12000.0f, 86000.0f
	//roll=0;
    	//pitch=(M_PI /180)*10;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
        myTIMES.open ("times.txt");
	}
        else if(metric==3){
        origin=point3d(2080.64, -636.219, 4339.94 );//42000.0f, 12000.0f, 86000.0f//75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//38000.0f, 12000.0f, 86000.0f
 	//tf::Quaternion q(0.5, 0.5 , 0.5 , 0.5); //rotate by  degrees about y axis
        //quaternion_to_euler_angles(q); 
	//roll=(M_PI /180)*10;
        //pitch=0;//(M_PI /180)*90;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
	myTIMES.open ("times.txt");         
	}
        else if(metric==4){
        origin=point3d(3100.64, -636.219, 4339.94 );//44000.0f, 13000.0f, 86000.0f//75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//38000.0f, 12000.0f, 86000.0f
	//roll=0;
    	//pitch=(M_PI /180)*10;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
	myTIMES.open ("times.txt");                
	}
        else if(metric==5){
        origin=point3d(4161.64, -636.219, 4339.94 );////75000.0f, 75000.0f, 28000.0f//40.0f, 40.0f, 33.0f//46000.0f, 12000.0f, 86000.0f
        //roll=(M_PI /180)*20;
    	//pitch=0;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
	myTIMES.open ("times.txt");
	}
        else if(metric==6){
        origin=point3d(5261.64, -636.219, 4339.94 );////75000.0f, 75000.0f, 28000.0f//45.0f, 40.0f, 33.0f   //48000.0f, 12000.0f, 86000.0f
         
        //roll=(M_PI /180)*30;
    	//pitch=0;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
	myTIMES.open ("times.txt");
	}
        else if(metric==7){
        origin=point3d(6361.64, -636.219, 4339.94 );////75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//48000.0f, 14000.0f, 86000.0f
        //roll=0;
    	//pitch=(M_PI /180)*10;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
       	myTIMES.open ("times.txt");
	}
        else if(metric==8){
        origin=point3d(7461.64, -636.219, 4339.94 );////75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f//50000.0f, 15000.0f, 86000.0f
        //roll=0;
    	//pitch=(M_PI /180)*20;
    	//yaw=0;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
	myTIMES.open ("times.txt");
        }
        else if(metric==9){
        origin=point3d(8061.64, -636.219, 4339.94);////75000.0f, 75000.0f, 28000.0f//50.0f, 40.0f, 33.0f// 52000.0f, 13000.0f, 86000.0f
        //roll=(M_PI /180)*10;
    	//pitch=(M_PI /180)*30;
    	//yaw=(M_PI /180)*60;
        roll=(M_PI /180)*(45 ); //
    	pitch=(M_PI /180)*(-40 );//
    	yaw=(M_PI /180)*(-120);//
	myTIMES.open ("times.txt"); 
	}
 */   	 ///////////////////////////////////////////////////////////////////////////////////////////
    	//cvtColor( dst, dst_gray, CV_BGR2GRAY );
    	dst_gray=dst.clone();
        if (!dst_gray.empty())
        {
         	namedWindow( "img", WINDOW_AUTOSIZE );
                imshow ("img", dst_gray);
		cv::waitKey(1);
        }
        ////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////
    	find_contours_and_project(dst_gray, octree ,ctree, w, h, F, roll, pitch, yaw, polygons_coord);

}
/*
void read_camera_settings()
{

double camera_matrix[3][3];
if (ros::param::get("/camera_matrix", camera_matrix))
{
 //principal point in mm of the focal plane used to projection
	cx=(camera_matrix[1][3])*0.0065*0.001;
	cy=(camera_matrix[2][3])*0.0065*0.001;
}

if (ros::param::get("relative_name", relative_name))
{
...
}





}
*/
bool KML_response(multidrone_msgs::KMLResponse::Request  &req, multidrone_msgs::KMLResponse::Response &res)
{

	ROS_INFO("sending back KML response");
	res.KML_file="KML_string";
	return true;

}

void initialization_of_clipper_polygons(){

              //initialization of clip, result

               // for (int i = 0; i < 3; i++) {
		//	result[0].push_back(IntPoint(0, 0));
		//}
	

}

int main(int argc, char **argv)
{
        
	ros::init(argc, argv, "semantic_map_manager");
	ros::NodeHandle n;
        
        //initialization_of_clipper_polygons();
	
        //MAP loading from a file

	//octomap::OcTree* octree = new octomap::OcTree("/home/ekakalet/Documents/new_college.bt"); 
	//ColorOcTree* ctree=new ColorOcTree(0.1); 
	
	//subscriber for receiving the heatmap
	
        //ros::Subscriber sub = n.subscribe<visualanalysis_msgs::CrowdHeatmap>("VisualSemantics/CrowdHeatmap", 1, CrowdProcessingCallback);

        //visualanalysis_msgs::TimedFrame msg;
	//ros::Subscriber sub = n.subscribe("frame_topic", 1,  CrowdProcessingCallback);

        message_filters::Subscriber<visualanalysis_msgs::CrowdHeatmap> image_sub(n, "drone_X/visual_analysis/crowd_heatmap", 1);
        message_filters::Subscriber<multidrone_msgs::GimbalStatus> gimbal_sub(n, "drone_X/gimbal/status", 1);
        message_filters::Subscriber<multidrone_msgs::CameraStatus> camerast_sub(n, "drone_X/camera/status", 1);
	message_filters::Subscriber<multidrone_msgs::DroneTelemetry> drone_telem_sub(n, "drone_X/telemetry", 1);

        typedef sync_policies::ApproximateTime<visualanalysis_msgs::CrowdHeatmap, multidrone_msgs::GimbalStatus, multidrone_msgs::CameraStatus, multidrone_msgs::DroneTelemetry> MySyncPolicy;
        
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, gimbal_sub, camerast_sub, drone_telem_sub);
        sync.registerCallback(boost::bind(&CrowdProcessingCallback, _1, _2, _3, _4));

 
        //publisher the crowdpolygons
	CrowdAnnotations_pub = n.advertise<visualanalysis_msgs::PolygonStamped_array>("SemanticMap/CrowdAnnotations", 1);
        while (ros::ok())
        {
    
    	CrowdAnnotations_pub.publish(msg2);
    	//cout<<"message of crowd polygons had been published"<<endl;
    	//cout<<"OUTPUT MESSAGE"<<msg2<<endl;
    	ros::spinOnce();
    	}         
        
        ros::ServiceServer service = n.advertiseService("response_static_annotations", KML_response);
 	ROS_INFO("Ready for KML response");
	ros::spin();

  	return 0;

}

/**
 * @function Threshold_Demo
 */
void Threshold_Demo( int, void* )
{
    /* 0: Binary
       1: Binary Inverted
       2: Threshold Truncated
       3: Threshold to Zero
       4: Threshold to Zero Inverted
     */

    threshold( src_gray, dst, threshold_value=85, max_BINARY_value,threshold_type=0 );//=67

    //imshow( window_name, dst );
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






////////////////////////////////////////////////////////////////////////////////////////


double PerpendicularDistance(point3d &pt,point3d &lineStart,point3d &lineEnd)
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

void RamerDouglasPeucker(vector<point3d> &pointList, double epsilon, vector<point3d> &out)
{
    if(pointList.size()<2)
        throw invalid_argument("Not enough points to simplify");

    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
        double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
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
        RamerDouglasPeucker(firstLine, epsilon, recResults1);
        RamerDouglasPeucker(lastLine, epsilon, recResults2);

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
std::string sort_in_counter_clockwise(vector<vector<point3d> > elements)
{
    std::string ss;
    std::ostringstream oss;
    ofstream myfile;
    myfile.open ("polygons.txt");
    Paths poly(1);
    for (std::size_t i = 0; i < elements.size(); ++i) {/// CHANGE FOR KML PROBLEM FROM 0 TO 1

        if(elements[i].size()!=0) {  //if (elements[i].size()<=30) break;
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

            elements[i].push_back(elements[i].at(0));// push back the first element in order to create a closed polyline for the polygon
            
             /*std::clock_t c_start4, c_end4;
             c_start4 = std::clock();
           vector<point3d> pointListOut;
             if(elements[i].size()>=650) { //while 650
                 //limit 250 : make simplification on the polygons so that MyMaps could display it
                    cout<<"IN_1"<<endl;
                    RamerDouglasPeucker(elements[i],  2.7e+13, pointListOut);//0.008
                    //for (std::size_t j = 0; j < pointListOut.size(); j++) {
                      //  oss << pointListOut.at(j).x() << ", " << pointListOut.at(j).y() << ", "
                       //     << pointListOut.at(j).z() << " \n";
                       //     }
                //elements[i].clear();
                elements[i]=pointListOut;
            }
           c_end4 = std::clock();
             long double time_elapsed_ms4 = 1000.0 * (c_end4-c_start4) / CLOCKS_PER_SEC;
    cout<<endl;
    cout<<"Simplification time"<<time_elapsed_ms4<<endl;
    myTIMES <<"Simplification time"<<time_elapsed_ms4<<"\n";
    cout<<endl;*/
            
            myfile <<i<<" "<<"\n";
            poly[0].clear();
            for (std::size_t j = 0; j <elements[i].size(); j++) {// fraction to 10000 in order to be depicted to mymaps
                oss << elements[i].at(j).x()/10000 << ", " << elements[i].at(j).y()/10000 << ", "
                    << elements[i].at(j).z()/10000 << " \n";
                
            //cout<<"polygon will be created"<<endl;
            //create the message for polygon
            
					
                        poly[0].push_back(IntPoint(elements[i][j].x()*(100000000000),elements[i][j].y()*(100000000000), elements[i][j].z()*(100000000000)));
 			//cout<<"Created vertex polygon "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
                        cout<<"Created vertex polygon "<<poly[0][j].X<<" "<<poly[0][j].Y<<endl;
                        myfile <<poly[0][j].X<<" "<<poly[0][j].Y<<"\n";
             }
             
                cout<<"Created polygon "<<endl;
                cout<<i<<"**************************"<<endl;
                std::clock_t c_start5, c_end5;
                c_start5 = std::clock();
                fusion_polygon_lines(poly);
                clip=result;
                cout << "done fusion polylines..." << endl;                
		c_end5 = std::clock();                
		long double time_elapsed_ms5 = 1000.0 * (c_end5-c_start5) / CLOCKS_PER_SEC;
    		cout<<endl;
    		cout<<"done fusion polylines...time"<<time_elapsed_ms5<<endl;
                myTIMES <<"done fusion polylines...time"<<time_elapsed_ms5<<"\n";
    		cout<<endl;
                write_clipper_polygon_result_to_kml(result);
                cout << "write_clipper_polygon_result_to_kml..." << endl;

                  
            oss << "</coordinates>\n"
                //  <<"</LineString>\n";
                << "</LinearRing>\n"
                //<< "</innerBoundaryIs>\n"
                << "</outerBoundaryIs>\n"
                << "</Polygon>\n"
                << "</Placemark>\n";
                
        }else{
            oss<<"\n";
        }
    
    }

    //cout<<"created the msg"<<endl;
    ////visualanalysis_msgs::PolygonStamped_array msg2;
    //CrowdAnnotations_pub.publish(msg2);
    //cout<<"message of crowd polygons had been published"<<endl;
    myfile.close();

    ss = oss.str();
    return ss;
};
std::string FormatPolygonCoordinates(vector<vector<point3d> > elements){
    std::string ss;
    std::ostringstream oss;
    //for(int i=0; i< elements.size(); i++) {


        //for (int j = 0; j < elements[i].size(); j++) {
            oss << sort_in_counter_clockwise( elements);

            //oss << elements[i].at(j).x() << ", " << elements[i].at(j).y() << ", 0" << " \n";//elements.at(i).z()
        //}

        ss = oss.str();
        return ss;
   // }
}
std::string FormatPolygon(vector<vector<point3d> > elements)
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
void write_to_kml(vector<vector<point3d> > coordinates){
    std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
    handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    kml_file_count++;
    // Open the KML file for writing:
    stringstream filename;
    filename << "//home//ekakalet//Programms//Sample_" << std::setw(4) << std::setfill('0') << kml_file_count << ".kml";
    handle.open(filename.str().c_str());
     
// Write to the KML file:
    handle << "<?xml version='1.0' encoding='utf-8'?>\n";
    handle << "<kml xmlns='http://www.opengis.net/kml/2.2'\n";
    handle << "xmlns:gx='http://www.google.com/kml/ext/2.2'\n";
    handle << "xmlns:kml='http://www.opengis.net/kml/2.2'\n";
    handle << "xmlns:atom='http://www.w3.org/2005/Atom'>\n";



    //for   (int i = 0; i < coordinates.size(); i++)
    //for (int j = 0; j < coordinates[i].size(); j++) {//sizeof(coordinates)


        handle << FormatPolygon(coordinates);



    //}
    handle << "</kml>\n";
    handle.close();

}
 // roll (x-axis rotation)
// pitch (y-axis rotation)
// yaw (z-axis rotation)
void quaternion_to_euler_angles(const tf::Quaternion &q)
{
		tf::Matrix3x3 m(q); //possibly quaternion in XYZ sequence
 	
		m.getRPY(roll, pitch, yaw, 1);
		//yaw=yaw+M_PI;//or +180 degrees in order to agree with camera frame {C}		
		std::cout << "In Rads: Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl; 
	
}
void rotation_based_in_euler_angles(double roll,double pitch,double yaw)
{//augmenting the angles and translation/rotation matrices
    double M[4][4];
    //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl; 
    //angles in radians
    double theta1=roll;//0; //rotation around the x axis with theta1 in radians
    double theta2=pitch; //0; //rotation around the y axis with theta2 in radians
    double theta3=yaw;//0; //rotation around the z axis with theta3 in radians
    //coordinates of COP
    double xgps=origin.x();
    double ygps=origin.y();
    double zgps=origin.z();

   // cout << "coordinates of COP "<<xgps<<" "<<ygps<< " " <<zgps<<endl;
   //Rz*Ry*Rx 
   M[0][0]=cos(theta2)*cos(theta3);
    M[0][1]=cos(theta3)*sin(theta1)*sin(theta2) - cos(theta1)*sin(theta3);
    M[0][2]=sin(theta1)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2);
    M[0][3]=xgps + ygps*(cos(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2)) - zgps*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta3)*sin(theta2)) - xgps*cos(theta2)*cos(theta3);
    M[1][0]=cos(theta2)*sin(theta3);
    M[1][1]=cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3);
    M[1][2]=cos(theta1)*sin(theta2)*sin(theta3) - cos(theta3)*sin(theta1);
    M[1][3]=ygps - ygps*(cos(theta1)*cos(theta3) + sin(theta1)*sin(theta2)*sin(theta3)) + zgps*(cos(theta3)*sin(theta1) - cos(theta1)*sin(theta2)*sin(theta3)) - xgps*cos(theta2)*sin(theta3);
    M[2][0]=-sin(theta2);
    M[2][1]=cos(theta2)*sin(theta1);
    M[2][2]=cos(theta1)*cos(theta2);
    M[2][3]=zgps + xgps*sin(theta2) - zgps*cos(theta1)*cos(theta2) - ygps*cos(theta2)*sin(theta1);
    M[3][0]=0;
    M[3][1]=0;
    M[3][2]=0;
    M[3][3]=1;


    //Rx*Ry*Rz in sequence ZYX (XYZ in bibliography)
 /* M[0][0]=cos(theta2)*cos(theta3);
    M[0][1]= -cos(theta2)*sin(theta3);
    M[0][2]=sin(theta2);
    M[0][3]=xgps - zgps*sin(theta2) - xgps*cos(theta2)*cos(theta3) + ygps*cos(theta2)*sin(theta3);
    M[1][0]=cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2);
    M[1][1]=cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3);
    M[1][2]=-cos(theta2)*sin(theta1);
    M[1][3]=ygps - xgps*(cos(theta1)*sin(theta3) + cos(theta3)*sin(theta1)*sin(theta2)) - ygps*(cos(theta1)*cos(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) + zgps*cos(theta2)*sin(theta1);
    M[2][0]=sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2);
    M[2][1]=cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3);
    M[2][2]=cos(theta1)*cos(theta2);
    M[2][3]=zgps - xgps*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2)) - ygps*(cos(theta3)*sin(theta1) + cos(theta1)*sin(theta2)*sin(theta3)) - zgps*cos(theta1)*cos(theta2);
    M[3][0]=0;
    M[3][1]=0;
    M[3][2]=0;
    M[3][3]=1;
*/
    //cout <<"M[0][] "<<M[0][0]<<" "<<M[0][1] <<" "<< M[0][2] <<" " << M[0][3]<<" " <<endl;
    //cout <<"M[1][] "<<M[1][0]<<" "<<M[1][1] <<" "<< M[1][2] <<" " << M[1][3]<<" " <<endl;
    //cout <<"M[2][] "<<M[2][0]<<" "<<M[2][1] <<" "<< M[2][2] <<" " << M[2][3]<<" " <<endl;
    //cout <<"M[3][] "<<M[3][0]<<" "<<M[3][1] <<" "<< M[3][2] <<" " << M[3][3]<<" " <<endl;
    double mult[4][1];
    double direction_B[4][1];
    //double origin_n[4][1];
    //double origin_e[4][1];
    //origin_n[0][0]=origin.x();
    //origin_n[1][0]=origin.y();
    //origin_n[2][0]=origin.z();
    //origin_n[3][0]=1;//homogeneous coordinates on point

    //OP
    direction_B[0][0] = direction.x();//-w/2;
    direction_B[1][0] = direction.y();//-h/2;
    direction_B[2][0] = direction.z();//-F;
    direction_B[3][0] = 0; //homogeneous coordinates on vector
    //OB
    //direction_B[1] =point3d(w/2, h/2, -F);
    //OC
    //direction_B[2] =point3d(w/2, -h/2, -F);
    //OD
    //direction_B[3] =point3d(-w/2, -h/2, -F);





    // Initializing elements of matrix mult to 0.
    /*int j=0;
    for(int i = 0; i <= 3; ++i){
        
        //origin_e[i][j]=0;
        mult[i][j]=0;
        
    }*/
    
    /*j=0;
    for(int i = 0; i <= 3; ++i){
       

        origin_e[i][j]=0;
    }*/
    // Multiplying matrix a and b and storing in array mult.
    int j=0;
    for(int i = 0; i <= 3; ++i){

        mult[i][j]=0;
        for(int k = 0; k <= 3; ++k)
        {
            mult[i][j] += M[i][k] * direction_B[k][j];
            //origin_e[i][j] += M[i][k] * origin_n[k][j];
        }
        
    }
    //cout <<"New direction rotated "<<mult[0][0]<<" "<<mult[1][0] <<" "<< mult[2][0] <<" " << mult[3][0]<<" " <<endl;

    // Multiplying matrix a and b and storing in array mult.
   /* j=0;
    for(int i = 0; i <= 3; ++i){
        
        for(int k = 0; k <=3; ++k)
        {
            origin_e[i][j] += M[i][k] * origin_n[k][j];
        }
        
    }*/
    //cout <<"New origin_n rotated "<<origin_n[0][0]<<" "<<origin_n[1][0] <<" "<< origin_n[2][0] <<" " << origin_n[3][0]<<" " <<endl;
    //cout <<"New origin_e rotated "<<origin_e[0][0]<<" "<<origin_e[1][0] <<" "<< origin_e[2][0] <<" " << origin_e[3][0]<<" " <<endl;


    ////////////////////////////////////////////////////////////////////////////////////////////
    direction= point3d(mult[0][0], mult[1][0], mult[2][0]);
    //direction.x()=mult[0][0];
    //direction.y()=mult[1][0];  
    //direction.z()=mult[2][0];
   // origin=point3d (origin_e[0][0],origin_e[1][0],origin_e[2][0]);
    //cout << endl;
    

   /* cout << "AFTER ROTATE: casting ray from " << origin  << " in the " << direction << " direction"<< endl;
    std::vector< point3d> ray;
    if(tree->castRay(origin, direction, end, true) ){//
        tree->isNodeOccupied(tree->search(end));
        //tree->insertRay(origin, end, false);//{

        //ctree->updateNode(end.x(), end.y(), end.z(), true);
        //ctree->setNodeColor(end.x(), end.y(), end.z(), 0, 255, 0);
        //std::cout << "Boundary tree hit: " << end << std::endl;
        //if(success){

        cout << "ray hit cell with center " << end << endl;
        //if (tree->computeRay(origin, end, ray)) {
        //for (int i=1; i<=ray.size(); i++){
        //cout <<"structure that holds the keys of all nodes traversed by the ray"<< ray[i] <<endl;}
        //}
        point3d intersection;
        bool success = tree->getRayIntersection(origin, direction, end, intersection);

        if(success) {
            cout << "entrance point is " << intersection << endl;
            cout << endl;
        }
        //}else{
        //  tree->isNodeOccupied(tree->search(end));
        // std::cout << "Boundary tree hit: " << end << std::endl;
        //}

    }
    if(!tree->search(end))
    {
        cout << "projection in a hole "  << endl;
        tree->search(end);
        cout <<"unknown voxel hit  " <<end<<endl;
    }
    
    ctree->writeBinary("my_tree.bt");
    cout<<"done writing of binary tree..."<<endl;*/
}



/*static void toEulerAngle(double X, double Y, double Z, double G , double& roll, double& pitch, double& yaw)//const Quaterniond& q
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (G * X + Y * Z);
	double cosr = +1.0 - 2.0 * (X * X + Y * Y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (G * Y - Z * X);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (G * Z + X * Y);
	double cosy = +1.0 - 2.0 * (Y * Y + Z * Z);  
	yaw = atan2(siny, cosy);
}*/
void fusion_polygon_lines(Paths poly)
{       
        ofstream myfile;
        myfile.open ("clipper_subj.txt");
	Paths subj(1);
	subj.clear();
	subj=poly;
	// poly is a polygon with vertices: elements[i][j]
	//
	for (int l=0; l< poly.size(); l++){
                	for (int i = 0; i < subj[0].size(); i++) {
			//subj[0].push_back(IntPoint(poly[l][i].X, poly[l][i].Y,poly[l][i].Z));
			myfile <<subj[l][i].X<<" "<<subj[l][i].Y<<subj[l][i].Z<<"\n";
                        }
                        //subj[0].push_back(subj[0].at(0));// push back the first element in order to create a closed polyline for the polygon
         }               
        //cout<<"subj="<<subj[0] <<endl;
	//draw input polygons with user-defined routine ... 
	//DrawPolygons(subj, 0x160000FF, 0x600000FF); //blue
	//DrawPolygons(clip, 0x20FFFF00, 0x30FF0000); //orange

	//clip_p=polygon;
	
	//perform union ...
	Clipper c;
	c.AddPaths(subj, ptSubject, true);
	c.AddPaths(clip, ptClip, true);
	c.Execute(ctUnion, result, pftNonZero, pftNonZero);
        //clip_p=result;
        
        //draw solution with user-defined routine ... 
	//DrawPolygons(result, 0x3000FF00, 0xFF006600); //solution shaded green
        //cout<<"result="<<result <<endl;
        myfile.close();

        return;
                 
}

void write_clipper_polygon_result_to_kml(Paths polygon)
{
        Paths solution;
	geometry_msgs::Point32 pt;
        point3d point;
        geometry_msgs::PolygonStamped poly;
	ofstream myfile;
        myfile.open ("clipper_result.txt");
	vector<vector<point3d> > coordinates(polygon.size());
               // cout<<" polygon contours "<<polygon.size()<<endl;
        //offsetting
	//ClipperOffset co;
        //co.AddPath(polygon, jtRound, etClosedPolygon);
        //co.Execute(solution, -7.0);
        //polygon=solution; 
            
		for (int l=0; l<polygon.size(); l++){
                        cout<<"result"<<l<<"="<<endl;
                        myfile <<l<<" "<<l<<"\n";
                        coordinates[l].clear();
                        
                        for (int i = 0; i <polygon[l].size(); i++) {
                        ////cout<<polygon[l][i].X<<" "<<polygon[l][i].Y<<endl;
                        //myfile <<polygon[l][i].X<<" "<<polygon[l][i].Y<<"\n";
                        pt.x=polygon[l][i].X;///100000000000;//maybe we lost decimal digits due to conversion Int to float32
 			pt.y=polygon[l][i].Y;///100000000000;
 			pt.z=polygon[l][i].Z;//0
                        ////cout<<"Created vertex polygon "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
                        myfile <<polygon[l][i].X<<" "<<polygon[l][i].Y<<" "<<pt.z<<"\n";   
                        point.x()=pt.x;//in order to be depicted to mymaps
                        point.y()=pt.y;//in order to be depicted to mymaps
                        point.z()=0;//pt.z;
                        coordinates[l].push_back(point);
                        }
                        coordinates[l].push_back(coordinates[l].at(0));// push back the first element in order to create a closed polyline for the polygon
                        /*std::clock_t c_start6, c_end6;
             		c_start6 = std::clock();
                        vector<point3d> pointListOut;
                        	if (coordinates[l].size()>=650) {//while 1250 650
                 			//limit 250 : make simplification on the polygons so that MyMaps could display it
                    			cout<<"IN"<<endl;
                    			RamerDouglasPeucker(coordinates[l], 2.7e+13 , pointListOut);//0.08//0.008//2.7e+13 //8.89e+12
                    			//coordinates[l].clear();
                			coordinates[l]=pointListOut;
            			}
                        //if (coordinates[l].size()<=30) coordinates[l].clear();
                        c_end6 = std::clock();
			long double time_elapsed_ms6 = 1000.0 * (c_end6-c_start6) / CLOCKS_PER_SEC;
    			cout<<endl;
    			cout<<"simplification2...time"<<time_elapsed_ms6<<endl;
                	myTIMES <<"simplification2...time"<<time_elapsed_ms6<<"\n";
    			cout<<endl;*/
                        for (int i = 0; i < coordinates[l].size(); i++) {
                        pt.x=polygon[l][i].X/10000000000000;//<-kmlbased/100000000000;//maybe we lost decimal digits due to conversion Int to float32     
 			pt.y=polygon[l][i].Y/10000000000000;///100000000000;
 			pt.z=polygon[l][i].Z/10000000000000;///100000000000;//0
			poly.polygon.points.push_back(pt);
                        poly.header.seq=l;
			poly.header.stamp = ros::Time::now();
			poly.header.frame_id ='0';
			}
                 
                        
                cout<<"Created polygon "<<endl;
            	
                //create the msg polygonstamped_array
                msg2.areas.push_back(poly);
		msg2.header.seq=l;
    		msg2.header.stamp = ros::Time::now();
    		msg2.header.frame_id ='0';
                cout<<l<<"Final************************"<<endl;
	    	}
    cout<<"created the msg"<<endl;
      
    
    //CrowdAnnotations_pub.publish(msg2);
    //cout<<"message of crowd polygons had been published"<<endl;
    //cout<<"OUTPUT MESSAGE"<<msg2<<endl;
      myfile.close();
  
        

        cout<<"Done_created_Coordinates..."<<endl;
        std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
    handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    kml_clipper_count++;
// Open the KML file for writing:
    stringstream filename;
    filename << "//home//ekakalet//Programms//Sample_clipper_result_" << std::setw(4) << std::setfill('0') << kml_clipper_count << ".kml";
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
       
     handle << FormatPolygon_clipper(coordinates);
              
     handle  << "</Document>\n";

     handle << "</kml>\n";
     handle.close();
        cout << "done writing coordinates to kml..." << endl;
}
std::string FormatPolygon_clipper(vector<vector<point3d> > elements){
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
                     oss << elements[i].at(j).x()/1000000000000000 << ", " << elements[i].at(j).y()/1000000000000000<< ", " << elements[i].at(j).z()/1000000000000000<< " \n";
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
void find_contours_and_project(Mat dst, OcTree* octree , ColorOcTree* ctree, double w, double h, double F, double roll, double pitch, double yaw, vector<vector<point3d> > &polygons_coord){
//find the orientation - direction

ofstream myfile;
        myfile.open ("intersection.txt");
// find contours
//vector<vector<point3d> > coordinates;
std::clock_t c_start, c_end;
std::clock_t c_start2, c_end2;
std::clock_t c_start3, c_end3;
c_start = std::clock();
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
//in order to project correctly not as projector but as the camera sees
flip(dst, dst, +1);
cv::rotate(dst, dst, ROTATE_180);
//
c_start2 = std::clock();
findContours(dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

// draw contours:
Mat imgWithContours = Mat::zeros(dst.rows, dst.cols, CV_8UC1);
RNG rng(12345);
for (int i = 0; i < contours.size(); i++)
{
Scalar color = Scalar(255,255,255);
drawContours(imgWithContours, contours, i, color, 1, 8, hierarchy, 0);
    cout<<"contours polyline coordinates"<<contours[i]<<endl;

}
c_end2 = std::clock();
long double time_elapsed_ms2 = 1000.0 * (c_end2-c_start2) / CLOCKS_PER_SEC;
    cout<<endl;
    cout<<"find and draw contours time"<<time_elapsed_ms2<<endl;
    cout<<endl;
    //namedWindow( "Contours", WINDOW_AUTOSIZE );
    //cv::imshow("Contours", imgWithContours);
    //waitKey(0);
    ///////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////
    vector<vector<point3d> > coordinates(contours.size());
    //double bgrPixel;
    point3d end;
    bool flag=true;
c_start3 = std::clock();
    direction.x()=-w/2;//-cx; 
    direction.y()=-h/2;//-cy;
    direction.z()=-F;
    cout<< "direction "<< direction.x() <<" " <<direction.y()<< " "<<direction.z()<<endl;
    //rotation_based_in_euler_angles( roll, pitch, yaw);
    double A=direction.x();//
    double B=direction.y();//
    double C=direction.z();//
    int l=0;
    for (int i = 0; i < contours.size(); i++){
        ////cout<<"NEW_CONTOUR"<<endl;
        int k=0;
        //////////////////////////////////////////
        cout<<"BEFORE contour size"<<contours[i].size()<<endl;
        vector<Point> pointListOut;         
        if(contours[i].size()>=10) { //while 650
                 //limit 250 : make simplification on the polygons so that MyMaps could display it
                    cout<<"IN_BEFORE_PROJECTION"<<endl;
                    RamerDouglasPeucker2D(contours[i],  6, pointListOut);//0.008, 2.7e+13,
                    contours[i]=pointListOut;
        }
        contours[i].push_back(contours[i].at(0));// push back the first element in order to create a closed polyline for the polygon
        ///////////////////////////////////////////////
        cout<<"AFTER contour size"<<contours[i].size()<<endl;
        for (int j = 0; j < contours[i].size();j++){;
           //// cout<<"2D coordinates " <<contours[i][j]<<endl;
            	
        //point3d direction(((-w / 2) + (contours[i][j].x * w )/ (imgWithContours.cols)), ((-h / 2)+ (contours[i][j].y * h) / (imgWithContours.rows)), -F);
        //cout <<"3D coordinates " <<((-w / 2) + (contours[i][j].x * w )/ (imgWithContours.cols)) << " " << ((-h / 2)+ (contours[i][j].y * h) / (imgWithContours.rows)) << " " << -F << endl;

          direction=point3d((A + (contours[i][j].x * w )/ imgWithContours.cols), (B + (contours[i][j].y * h) / imgWithContours.rows), C);
        ////cout <<"3D coordinates " <<(A + (contours[i][j].x * w )/ (imgWithContours.cols)) << " " << (B+ (contours[i][j].y * h) / (imgWithContours.rows)) << " " << C << endl;
                   //cout<<endl;
	           ////cout<<"BEFORE"<<" direction "<<direction<<endl;
                   rotation_based_in_euler_angles( roll, pitch, yaw);
		   ////cout<<"AFTER "<<" direction "<<direction<<endl;

                   if (octree->castRay(origin, direction, end, true)) {//

                    //cout<<"inside the raycast "<< endl; 
                    octree->isNodeOccupied(octree->search(end));
                    ////////////////////////////////////////////////////////////////////
                    //color the node

                    /*cout<< "l="<<l<<"k="<<k<<endl;
                    ctree->updateNode(end.x(), end.y(), end.z(), true);
                    ctree->setNodeColor(end.x(), end.y(), end.z(), 0,255, 0 );
                    ctree->integrateNodeColor( end.x(), end.y(), end.z(), 0, 255, 0 );*/

                    //coordinates[l].push_back(end);

                    //coordinates[l][k].x()=end.x();
                    //coordinates[l][k].y()=end.y();
                    //coordinates[l][k].z()=end.z();
                    ///////////////////////////////////////////////////////////////////
                    //k++;
                    //cout << " i="<<l<<" k="<<k<<endl;
                    ///cout << "ray hit cell with center " << end << endl;
                    point3d intersection;
                    bool success = octree->getRayIntersection(origin, direction, end, intersection);

                    if (success) {
                        //cout << "entrance point is " << intersection << endl;
                        //cout << endl;

                        //cout<< "l="<<l<<"k="<<k<<endl;
                        ColorOcTreeNode* n=ctree->updateNode(intersection,true);
                        n->setColor(255, 0, 0);
                        ctree->updateNode(intersection.x(), intersection.y(), intersection.z(), true);
                        ctree->setNodeColor(intersection.x(), intersection.y(), intersection.z(), 0,255, 0 );
                        ctree->ColorOcTree::integrateNodeColor( intersection.x(), intersection.y(), intersection.z(), 0, 255, 0 );
                        ctree->ColorOcTree::updateInnerOccupancy();
                        myfile <<intersection.x()<<" "<< intersection.y()<< " "<< intersection.z()<<"\n";
                       
                        coordinates[l].push_back(intersection);
                        k++;
                        flag=true;
                        
                    }

                }
                if (!octree->search(end)) {
                    flag=false;//means that the value end is not registered
                    //cout << "projection in a hole " << endl;
                    octree->search(end);
                    //cout << "unknown voxel hit  " << end << endl;
                    if(j==contours[i].size()-1)flag=true;
                }

                //}
            //}

        }
        //cout<<" "<<coordinates[l].size()<<endl;
        if (flag==true ) {
                         l++;
                         
                         }
    }
    
    c_end3 = std::clock();
long double time_elapsed_ms3 = 1000.0 * (c_end3-c_start3) / CLOCKS_PER_SEC;
myTIMES <<"Project_time"<<time_elapsed_ms3<<"\n";
    cout<<endl;
    cout<<"Project_time"<<time_elapsed_ms3<<endl;
    cout<<endl;
    ctree->writeBinary("my_tree_projected.bt");
    cout<<"done writing of binary tree..."<<endl;
    //for(int i=0; i<l-1; i++)
    //{ 
        polygons_coord=coordinates;
        write_to_kml(coordinates);
        cout << "done writing coordinates to kml..." << endl;
    c_end = std::clock();
    long double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
myTIMES <<"Total_time"<<time_elapsed_ms<<"\n";
    cout<<endl;
    cout<<time_elapsed_ms<<endl;
    cout<<endl;
    cout<<"roll="<<roll<<" pitch="<<pitch<<" yaw="<<yaw<<endl; 
        
    cout << "metric..." <<metric<< endl;
    metric++;
    myTIMES.close();  
    myfile.close();
        return;
    //}
}



