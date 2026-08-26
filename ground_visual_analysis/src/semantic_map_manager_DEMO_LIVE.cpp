////////////////////////////////////////////////////
///REVIEW DEMO LIVE

////////////////////////////////////////////

#include "ros/ros.h"
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
#include <iomanip>
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
#include <visualanalysis_msgs/TimedFrame.h>
#include <geometry_msgs/Point32.h>
#include <ctime>
#include <time.h>
#include "clipper.hpp"
#include "geodetic_conv.hpp"
#include <tf/LinearMath/Matrix3x3.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <multidrone_msgs/GimbalStatus.h>
#include <multidrone_msgs/CameraStatus.h>
#include <multidrone_msgs/DroneTelemetry.h>
#include "Eigen/Dense"

using namespace Eigen;
using namespace cv;
using namespace std;
using namespace octomap;
using namespace octomath;
using namespace ClipperLib;
using namespace message_filters;
using namespace visualanalysis_msgs;
using namespace multidrone_msgs;
using namespace geodetic_converter;

#define M_PI           3.14159265358979323846  /* pi */
//declaration of variables

double h,w,fx,W,F, resolution;
double roll,pitch, yaw;
double cx; //principal point in m
double cy; //principal point in m
point3d origin, direction;
const tf::Quaternion q;
int kml_file_count=0;
int image_count=0;
int kml_clipper_count=0;
string path = "//home//ekakalet//Programms" ;


string btFilename;
octomap::OcTree* octree = new octomap::OcTree("Maps/dr.binvox.bt");
ColorOcTree* ctree=new ColorOcTree(octree->getResolution()); 

int threshold_value = 0;
int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

// sensor_msgs Image message
sensor_msgs::Image msg1;
cv_bridge::CvImagePtr cv_ptr;
visualanalysis_msgs::DynamicMapAnnotations msg2;
visualanalysis_msgs::TimedFrame msg;
geographic_msgs::GeoPoint GEO_origin;
ros::Publisher CrowdAnnotations_pub;
Paths subj(1), clip(1), result(1);
ros::Time timestamp;

double pixel_size_x;
double pixel_size_y;
std::vector<double> data;
double mount_position[3];
double image_width;
double image_height;
double cx_pixels;
double cy_pixels;
int metric=0;
string path_KML;
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
void RamerDouglasPeucker(vector<Vector3d> &pointList, double epsilon, vector<Vector3d> &out);
double PerpendicularDistance(Vector3d &pt,Vector3d &lineStart,Vector3d &lineEnd);
double PerpendicularDistance2D(const Point &pt, const Point &lineStart, const Point &lineEnd);
void RamerDouglasPeucker2D(const vector<Point> &pointList, double epsilon, vector<Point> &out);
void rotation_based_in_euler_angles(double roll,double pitch,double yaw);
static void toEulerAngle(double X, double Y, double Z, double G , double& roll, double& pitch, double& yaw);
void fusion_polygon_lines(Paths poly);
void write_clipper_polygon_result_to_kml(Paths result);
std::string FormatPolygon_clipper(vector<vector<Vector3d> > elements);
void quaternion_to_euler_angles(const tf::Quaternion &q);
void read_camera_settings();
Vector3d convert_octomap_to_geodetic_coordinates(Vector3d pi);
Vector3d convert_geodetic_to_octomap_coordinates(double latitude, double longitude, double altitude);

void CrowdProcessingCallback(const visualanalysis_msgs::CrowdHeatmapConstPtr& msg, const multidrone_msgs::GimbalStatusConstPtr& gimbal_msg, const multidrone_msgs::CameraStatusConstPtr& camerastatus_msg, const multidrone_msgs::DroneTelemetryConstPtr& dronetelemetry_msg)
{
	
        ROS_INFO("Create a Copy of the crowd heat map from message");
        cout<<"Gimbal Status Message: Roll: "<< gimbal_msg->roll<<" Pitch: "<<gimbal_msg->pitch<<" Yaw: "<< gimbal_msg->yaw <<endl;
	cout<< "Timeframe     Timestamp: "<<msg->header.stamp<<endl;
        cout<< "Gimbal Status Timestamp: "<<gimbal_msg->header.stamp<<endl;
        cout<< "Camera Status Timestamp: "<<camerastatus_msg->header.stamp<<endl;
        cout<< "Drone Telemetry Timestamp: "<<dronetelemetry_msg->header.stamp<<endl;
        timestamp=msg->header.stamp;

    	try
    	{

			// copy sensor_msgs::Image
			msg1 = msg->heatmap;

			// convert sensor_msgs::Image to cv_bridge::CvImagePtr, pointer to image
			cv_bridge::CvImagePtr cv_ptr;

			cv_ptr = cv_bridge::toCvCopy(msg1, "mono8");

			// convert object to cv::Mat
			cv::Mat src =cv_ptr->image;//imread("//home//ekakalet//Documents//frame_1823Heatmap_A.png", IMREAD_GRAYSCALE);// 

			ROS_INFO("Done imread");
			//Convert the image to Gray
			src_gray=src;

    	}
    	catch (cv_bridge::Exception& e)
    	{
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	}
    	std::clock_t c_start, c_end;
    	c_start = std::clock();
        
    	ROS_INFO("Ready for processing the crowd heatmap");

    	/// Call the function to initialize
    	Threshold_Demo( 0, 0 );

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
		Vector3d pi;
		GEO_origin=dronetelemetry_msg->geo_position;
		//GEO_origin.latitude=GEO_origin.latitude+mount_position[0];
		//GEO_origin.longitude=GEO_origin.longitude+mount_position[1];
		//GEO_origin.altitude=GEO_origin.altitude+mount_position[2];
		pi=convert_geodetic_to_octomap_coordinates(GEO_origin.latitude, GEO_origin.longitude, GEO_origin.altitude+26);
		//origin=point3d(GEO_origin.latitude, GEO_origin.longitude, GEO_origin.altitude);
		origin=	point3d(pi[0],pi[1],pi[2]);
                cout<<" origin coordinates: "<<origin<<endl;	
		w=image_width*(camerastatus_msg->sensorWidth)*0.001/1920;//width in m according to sensor size width of blackmagic: 12.48*0.001/1920
		h=image_height*(camerastatus_msg->sensorHeight)*0.001/1080;//height in m according to sensor size height of blackmagic: 7.02*0.001/1080
		cout<<"w="<<w<<" h="<<h<<endl;
		cout<<"sensorWidth "<< camerastatus_msg->sensorWidth <<endl;
		cout<<"sensorHeight "<< camerastatus_msg->sensorHeight <<endl;
                cout<<"Focal Length "<< camerastatus_msg->focalLength <<endl;
		F=camerastatus_msg->focalLength*0.001;//0.018;//initial value 0.008,  FL-Range=14mm-42mm;
		//direction=point3d(-cx, -cy, -F);//cx-w/2, cy-h/2, -F
		resolution=octree->getResolution();
		cx=0;//(900.259497081249)*0.0065*0.001; //principal point in m
		cy=0;//(590.2505880129497)*0.0065*0.001; //principal point in m
		cout<<"cx="<<cx<<" cy="<<cy<<endl;
		roll=(M_PI /180)*(gimbal_msg->roll);
		pitch=(M_PI /180)*(gimbal_msg->pitch);
		yaw=(M_PI /180)*(-gimbal_msg->yaw-70);//-70 in order to be alinghted the x of octomap to North of Google Earth and -yaw because the two coord system have opposite orientation

		cout<<"roll pitch yaw " <<gimbal_msg->roll <<" "<<gimbal_msg->pitch <<" "<< gimbal_msg->yaw<< endl;


	 	stringstream image_name;
		dst_gray=dst.clone();
		if (!dst_gray.empty())
		{
			//namedWindow( "img", WINDOW_AUTOSIZE );
			//imshow ("img", dst_gray);
			//cv::waitKey(1);
   			image_name << path <<"//thres_image_" << std::setw(4) << std::setfill('0') << image_count << ".png";
    			imwrite(image_name.str().c_str(), dst_gray);
                        image_count++;
			
		}
		
		find_contours_and_project(dst_gray, octree ,ctree, w, h, F, roll, pitch, yaw, polygons_coord);
		
}

void read_camera_settings()
{
//principal point in mm of the focal plane used to projection
cx_pixels=data[2];
cy_pixels=data[5];
cx=(cx_pixels)*pixel_size_x*0.001;
cy=(cy_pixels)*pixel_size_y*0.001;


}
Vector3d convert_geodetic_to_octomap_coordinates( double latitude, double longitude, double altitude)
{
	Matrix3d R;//computed from MATLAB
	Vector3d t;//computed from MATLAB
	Vector3d qi;
        Vector3d pi;
	const double lat0=40.632831000000000;
	const double long0=22.955920000000000;
	const double h0=25;
	double east; 
	double north; 
	double up;
        GeodeticConverter converter;
	/*//working R,t
	R(0,0)=0.553431751070518;
	R(0,1)=-0.831566143355096;
	R(0,2)=-0.047021762329288;

        R(1,0)=0.831587780724243;
	R(1,1)=0.554843391723770;
	R(1,2)=-0.024709787748829;

	R(2,0)=0.046637536997028;
	R(2,1)=-0.025427541878741;
	R(2,2)=0.998588193529673;

        t[0]=1.371350668521600;
	t[1]=0.382592118653361;
	t[2]=0.652419317134261;
        */ //for 2 times scale
        R(0,0)=0.553431751070518;
	R(0,1)=-0.831566143355096;
	R(0,2)=-0.047021762329288;

        R(1,0)=0.831587780724243;
	R(1,1)=0.554843391723770;
	R(1,2)=-0.024709787748829;

	R(2,0)=0.046637536997028;
	R(2,1)=-0.025427541878741;
	R(2,2)=0.998588193529673;

        t[0]=0.067193143198446;
	t[1]=-1.927726109335918;
	t[2]=-0.945122360784805;

        converter.initialiseReference(lat0, long0, h0);
        converter.geodetic2Enu(latitude, longitude, altitude, &east, &north, &up);
        
        qi[0]=east;///2;
	qi[1]=north;///2;
	qi[2]=up;
        
        pi=R.transpose()*(qi - t);
 
        cout<<"xEast_oct: "<<pi[0]<< " yNorth_oct: "<< pi[1]<<" zUp_oct: "<<pi[2]<<endl;
        return pi;
	
}
Vector3d convert_octomap_to_geodetic_coordinates(Vector3d pi)
{
	Vector3d return_value;
	Matrix3d R;//computed from MATLAB
	Vector3d t;//computed from MATLAB
	Vector3d qi;
        double latitude;
	double longitude;
	double altitude;
	const double lat0=40.632831000000000;
	const double long0=22.955920000000000;
	const double h0=25;
        GeodeticConverter converter;
	Matrix3d Ry;
        Matrix3d Rz;
	Matrix3d R_yaw;
        Matrix3d Ryaw;
      /*  //working R,t
	R(0,0)=0.553431751070518;
	R(0,1)=-0.831566143355096;
	R(0,2)=-0.047021762329288;

        R(1,0)=0.831587780724243;
	R(1,1)=0.554843391723770;
	R(1,2)=-0.024709787748829;

	R(2,0)=0.046637536997028;
	R(2,1)=-0.025427541878741;
	R(2,2)=0.998588193529673;

        t[0]=1.371350668521600;
	t[1]=0.382592118653361;
	t[2]=0.652419317134261;


	Ry(0,0)=cos(180);
	Ry(0,1)=0;
	Ry(0,2)=sin(180);

        Ry(1,0)=0;
	Ry(1,1)=1;
	Ry(1,2)=0;

	Ry(2,0)=-sin(180);
	Ry(2,1)=0;
	Ry(2,2)=cos(180);
        
        Rz(0,0)=cos(90);
	Rz(0,1)=-sin(90);
	Rz(0,2)=0;

        Rz(1,0)=sin(90);
	Rz(1,1)=cos(90);
	Rz(1,2)=0;

	Rz(2,0)=0;
	Rz(2,1)=0;
	Rz(2,2)=1; 

        R_yaw(0,0)=cos(-yaw);
	R_yaw(0,1)=-sin(-yaw);
	R_yaw(0,2)=0;

        R_yaw(1,0)=sin(-yaw);
	R_yaw(1,1)=cos(-yaw);
	R_yaw(1,2)=0;

	R_yaw(2,0)=0;
	R_yaw(2,1)=0;
	R_yaw(2,2)=1; 
  
        

        Ryaw(0,0)=cos(yaw);
	Ryaw(0,1)=-sin(yaw);
	Ryaw(0,2)=0;

        Ryaw(1,0)=sin(yaw);
	Ryaw(1,1)=cos(yaw);
	Ryaw(1,2)=0;

	Ryaw(2,0)=0;
	Ryaw(2,1)=0;
	Ryaw(2,2)=1;  
	*/	
	 //for 2 times scale
        R(0,0)=0.553431751070518;
	R(0,1)=-0.831566143355096;
	R(0,2)=-0.047021762329288;

        R(1,0)=0.831587780724243;
	R(1,1)=0.554843391723770;
	R(1,2)=-0.024709787748829;

	R(2,0)=0.046637536997028;
	R(2,1)=-0.025427541878741;
	R(2,2)=0.998588193529673;

        t[0]=0.067193143198446;
	t[1]=-1.927726109335918;
	t[2]=-0.945122360784805;
        
        converter.initialiseReference(lat0, long0, h0);
         
        cout<<"x_octomap: "<<pi[0]<< " y_octomap: "<< pi[1]<<" z_octomap: "<<pi[2]<<endl;
        
        qi=R*pi + t;
        //qi=R_yaw*Ry*Rz*Ryaw*qi;
	const double east = qi[0];//*2; 
	const double north = qi[1];//*2;
	const double up = qi[2];
        cout<<"east: "<<east<< " north: "<< north<<" up: "<<up<<endl;
        
        converter.enu2Geodetic(east, north, up, &latitude, &longitude, &altitude);
        return_value[0]=  latitude;
        return_value[1]=  longitude;
        return_value[2]=  altitude;
        //printf("%10.10f %10.10f %10.10f\n",return_value[0],return_value[1],return_value[2]);
        cout<<"latitude: "<<return_value[0]<< " longitude: "<< return_value[1]<<" altitude: "<<return_value[2]<<endl;
        
        return return_value;
}		
bool DoGetSemanticMap(multidrone_msgs::GetSemanticMap::Request  &req, multidrone_msgs::GetSemanticMap::Response &res)
{

	ROS_INFO("sending back KML response");
	// Avoid reading directly from the file because of path problems when doing "git clone" or "git pull" from other computers.
	std::ifstream ifs_file(path_KML.c_str());
	std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());
	res.semantic_map = _semantic_map;
	return true;

}
bool DoPostSemanticAnnotations(multidrone_msgs::PostSemanticAnnotations::Request  &req, multidrone_msgs::PostSemanticAnnotations::Response &res)
{
    //method partially implemented
	ROS_INFO("receiving KML annotations");
	// Avoid reading directly from the file because of path problems when doing "git clone" or "git pull" from other computers.
	std::ifstream ifs_file(path_KML.c_str());
	std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());
	 _semantic_map=req.semantic_map;
	return true;

}
void print_info(double cx_pixels,double cy_pixels, double image_width, double image_height, double mount_position_0, double mount_position_1 , double mount_position_2, double pixel_size_x, double pixel_size_y, int drone_id_)
{

	cout << "Initializing semantic map manager node with parameters: " <<endl;
	cout << "camera_matrix: "<< cx_pixels <<" "<<cy_pixels <<endl;
	cout << "image_width: " << image_width <<endl;
	cout << "image_height: " << image_height <<endl;
	cout << "mount_position: " << mount_position_0<<" "<<mount_position_1<<" "<<mount_position_2<<endl;
	cout << "pixel_size_x: " << pixel_size_x <<endl;
	cout << "pixel_size_y: " << pixel_size_y<<endl;
	cout << "drone_id_: " << drone_id_<<endl;
}

int main(int argc, char **argv)
{
        
		ros::init(argc, argv, "semantic_map_manager");
		ros::NodeHandle n;

                XmlRpc::XmlRpcValue my_list;
                n.getParam("/drone_sensors", my_list);
		ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int32_t i = 0; i < my_list.size(); ++i) 
		{
		  XmlRpc::XmlRpcValue sublist = my_list[i];
                  XmlRpc::XmlRpcValue mount_position_;
      		  string name=sublist["name"];
		  if (name=="CineCamera")
			{
			mount_position_=sublist["mount_position"];
			mount_position[0]=static_cast<double>(mount_position_[0]);
			mount_position[1]=static_cast<double>(mount_position_[1]);
			mount_position[2]=static_cast<double>(mount_position_[2]);
			pixel_size_x=static_cast<double>(sublist["pixel_size_x"]);
			pixel_size_y=static_cast<double>(sublist["pixel_size_y"]);
			}
		}

		//n.getParam("kml_path", path_KML);// get kml path
		n.getParam("/camera_matrix/data", data);
		n.getParam("/image_width", image_width);
		n.getParam("/image_height", image_height);
		static int drone_id_;
	        n.param<int>("/drone_id", drone_id_, 1);
                
                read_camera_settings();

		print_info(cx_pixels, cy_pixels, image_width, image_height, mount_position[0], mount_position[1] , mount_position[2], pixel_size_x, pixel_size_y, drone_id_);
		
		std::stringstream ss;
		ss << "drone_" << drone_id_;
		std::string str = ss.str();
		message_filters::Subscriber<visualanalysis_msgs::CrowdHeatmap> image_sub(n, str+"/visual_analysis/crowd_heatmap", 1);
		message_filters::Subscriber<multidrone_msgs::GimbalStatus> gimbal_sub(n, str+"/gimbal/status", 1);
		message_filters::Subscriber<multidrone_msgs::CameraStatus> camerast_sub(n, str+"/camera/status", 1);
		message_filters::Subscriber<multidrone_msgs::DroneTelemetry> drone_telem_sub(n, str+"/telemetry", 1);

		typedef sync_policies::ApproximateTime<visualanalysis_msgs::CrowdHeatmap, multidrone_msgs::GimbalStatus, multidrone_msgs::CameraStatus, multidrone_msgs::DroneTelemetry> MySyncPolicy;

		Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), image_sub, gimbal_sub, camerast_sub, drone_telem_sub);
		sync.registerCallback(boost::bind(&CrowdProcessingCallback, _1, _2, _3, _4));


		//publisher the crowdpolygons
		CrowdAnnotations_pub = n.advertise<visualanalysis_msgs::DynamicMapAnnotations>("visual_analysis/dynamic_map_annotations", 1);
		while (ros::ok())
		{
			CrowdAnnotations_pub.publish(msg2);
			ros::spinOnce();
		}

		ros::ServiceServer service_get = n.advertiseService("get_semantic_map", DoGetSemanticMap);
		ROS_INFO("Ready for KML response");
		ros::ServiceServer service_post = n.advertiseService("post_semantic_annotations", DoPostSemanticAnnotations);
		ROS_INFO("Ready for receiving KML ");
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

    threshold( src_gray, dst, threshold_value=105, max_BINARY_value,threshold_type=0 );//=87,67

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
//-----------------------------------------------------------------------------------------------------------
double PerpendicularDistance(Vector3d &pt,Vector3d &lineStart,Vector3d &lineEnd)
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

void RamerDouglasPeucker(vector<Vector3d> &pointList, double epsilon, vector<Vector3d> &out)
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
        vector<Vector3d> recResults1;
        vector<Vector3d> recResults2;
        vector<Vector3d> firstLine(pointList.begin(), pointList.begin()+index+1);
        vector<Vector3d> lastLine(pointList.begin()+index, pointList.end());
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
//---------------------------------------------------------------------------------------------------------
std::string sort_in_counter_clockwise(vector<vector<point3d> > elements)
{
    std::string ss;
    std::ostringstream oss;
    ofstream myfile;
    myfile.open ("polygons.txt");
    Paths poly(1);
    for (std::size_t i = 0; i < elements.size(); ++i) {

        if(elements[i].size()!=0) {
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
			myfile <<i<<" "<<"\n";
			poly[0].clear();
			for (std::size_t j = 0; j <elements[i].size(); j++) {// fraction to 10000 in order to be depicted to mymaps
				oss << elements[i].at(j).x()/10000 << "," << elements[i].at(j).y()/10000 << ","
				<< elements[i].at(j).z()/10000 << " \n";

					                                                             
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

    myfile.close();

    ss = oss.str();
    return ss;
};
std::string FormatPolygonCoordinates(vector<vector<point3d> > elements){
    std::string ss;
    std::ostringstream oss;
	oss << sort_in_counter_clockwise( elements);
	ss = oss.str();
	return ss;

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
    filename << path <<"//Sample_" << std::setw(4) << std::setfill('0') << kml_file_count << ".kml";
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


    //Rx*Ry*Rz in sequence ZYX
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

    double mult[4][1];
    double direction_B[4][1];

    //OP
    direction_B[0][0] = direction.x();//-w/2;
    direction_B[1][0] = direction.y();//-h/2;
    direction_B[2][0] = direction.z();//-F;
    direction_B[3][0] = 0; //homogeneous coordinates on vector
    

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

    ////////////////////////////////////////////////////////////////////////////////////////////
    direction= point3d(mult[0][0], mult[1][0], mult[2][0]);

}
//-----------------------------------------------------------------------------------------------------------------------------------
void fusion_polygon_lines(Paths poly)
{       
        ofstream myfile;
        myfile.open ("clipper_subj.txt");
	Paths subj(1);
	subj.clear();
	subj=poly;
	// poly is a polygon with vertices: elements[i][j]
	//
	for (int l=0; l< poly.size(); l++)
	{
		for (int i = 0; i < subj[0].size(); i++)
		{

			myfile <<subj[l][i].X<<" "<<subj[l][i].Y<<subj[l][i].Z<<"\n";
		}

	}

	
	//perform union ...
	Clipper c;
	c.AddPaths(subj, ptSubject, true);
	c.AddPaths(clip, ptClip, true);
	c.Execute(ctUnion, result, pftNonZero, pftNonZero);
    myfile.close();

    return;
                 
}

void write_clipper_polygon_result_to_kml(Paths polygon)
{               
		double latitude;
                double longitude;
                double altitude;
                Vector3d coord(0,0,0);
                Vector3d pi(0,0,0);
		Paths solution;
		geometry_msgs::Point32 pt;
		point3d point;
		geometry_msgs::PolygonStamped poly;
		ofstream myfile;
		myfile.open ("clipper_result.txt");
		vector<vector<Vector3d> > coordinates(polygon.size());
          
                
		for (int l=0; l<polygon.size(); l++)
		{
                        cout<<"result"<<l<<"="<<endl;
                        myfile <<l<<" "<<l<<"\n";
                        coordinates[l].clear();
                        
                        for (int i = 0; i <polygon[l].size(); i++)
                        {
							////cout<<polygon[l][i].X<<" "<<polygon[l][i].Y<<endl;
							//myfile <<polygon[l][i].X<<" "<<polygon[l][i].Y<<"\n";
							//pt.x=polygon[l][i].X;///100000000000;//maybe we lost decimal digits due to conversion Int to float32
							//pt.y=polygon[l][i].Y;///100000000000;
							//pt.z=polygon[l][i].Z;//0
                                                        pi[0]=polygon[l][i].X/100000000000.0;
                                                        pi[1]=polygon[l][i].Y/100000000000.0;
                                                        pi[2]=polygon[l][i].Z/100000000000.0;
							coord=convert_octomap_to_geodetic_coordinates(pi);
							////cout<<"Created vertex polygon "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
							//myfile <<polygon[l][i].X<<" "<<polygon[l][i].Y<<" "<<pt.z<<"\n";
							printf("%10.10f %10.10f %10.10f\n",coord[0],coord[1],coord[2]);
                                                        point.x()=coord[0];//pt.x;//in order to be depicted to mymaps
							point.y()=coord[1];//pt.y;//in order to be depicted to mymaps
							point.z()=coord[2];//pt.z;
                                                        double temp=coord[0];
							coord[0]=coord[1];
							coord[1]=temp;
                                                        myfile <<setprecision(14)<<point.x()<<" "<<setprecision(14)<<point.y()<<" "<<setprecision(14)<<point.z()<<"\n";
                                                        cout<<"Created vertex polygon "<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
							coordinates[l].push_back(coord);
                        }
                        coordinates[l].push_back(coordinates[l].at(0));// push back the first element in order to create a closed polyline for the polygon
                        vector<Vector3d> pointListOut;
		        if(coordinates[l].size()>=10)
                        { //while 650
							 //limit 250 : make simplification on the polygons so that MyMaps could display it
								cout<<"IN_BEFORE_PROJECTION"<<endl;
								RamerDouglasPeucker(coordinates[l], 0.000006, pointListOut);//0.008, 2.7e+13,
								coordinates[l]=pointListOut;
                        }
			//if(coordinates[l].size()<=10)
			//{
                        //coordinates[l].clear();
			//}
                        for (int i = 0; i < coordinates[l].size(); i++)
                        {
							pt.x=polygon[l][i].X/100000000000;//<-kmlbased/100000000000;//maybe we lost decimal digits due to conversion Int to float32
							pt.y=polygon[l][i].Y/100000000000;///100000000000;
							pt.z=polygon[l][i].Z/100000000000;///100000000000;//0
							poly.polygon.points.push_back(pt);
							poly.header.seq=l;
							poly.header.stamp = timestamp;
							poly.header.frame_id ='0';
						}
                 
                        
                cout<<"Created polygon "<<endl;
            	
                //create the msg polygonstamped_array
                msg2.areas.push_back(poly);
                msg2.header.seq=l;
                msg2.header.stamp = timestamp;
                //msg2.header.frame_id ='0';
                cout<<l<<"Final************************"<<endl;
	    	}
    cout<<"created the msg"<<endl;
      
    
    myfile.close();
  
        

	cout<<"Done_created_Coordinates..."<<endl;
	std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
     handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
     kml_clipper_count++;
// Open the KML file for writing:
     stringstream filename;
     //filename << path <<"//Sample_clipper_result_" << std::setw(4) << std::setfill('0') << kml_clipper_count << ".kml";
     filename << path <<"//Sample_clipper_result.kml" ;
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

std::string FormatPolygon_clipper(vector<vector<Vector3d> > elements)
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
                     oss << setprecision(14) <<elements[i].at(j).x() << "," << setprecision(14) <<elements[i].at(j).y()<< "," << setprecision(14) <<elements[i].at(j).z()<< " \n";
                 } //  /1000000000000000
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

		ofstream myfile;
		myfile.open ("intersection.txt");
		// find contours
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
		vector<vector<point3d> > coordinates(contours.size());
		point3d end;
		bool flag=true;
		c_start3 = std::clock();
		//initial direction of projector onto x
		direction.x()=-w/2;//-w/2;////-cx;//TODO:direction.x()+(-w/2-cx);//where direction=M*(0,0,-1)
		direction.y()=F;//-h/2;////-cy;//TODO:direction.y()+(-h/2-cy);
		direction.z()=-h/2;//-F;////-F; //TODO:direction.z()+(-F+F);
		cout<< "direction "<< direction.x() <<" " <<direction.y()<< " "<<direction.z()<<endl;
		//rotation_based_in_euler_angles( roll, pitch, yaw);
		double A=direction.x();//
		double B=direction.y();//
		double C=direction.z();//
		int l=0;
		for (int i = 0; i < contours.size(); i++)
		{
				////cout<<"NEW_CONTOUR"<<endl;
				int k=0;
				//////////////////////////////////////////
				cout<<"BEFORE contour size"<<contours[i].size()<<endl;
				vector<Point> pointListOut;
				if(contours[i].size()>=10)
                { //while 650
							 //limit 250 : make simplification on the polygons so that MyMaps could display it
								cout<<"IN_BEFORE_PROJECTION"<<endl;
								RamerDouglasPeucker2D(contours[i],  6, pointListOut);//0.008, 2.7e+13,
								contours[i]=pointListOut;
                }
				contours[i].push_back(contours[i].at(0));// push back the first element in order to create a closed polyline for the polygon
				///////////////////////////////////////////////
				cout<<"AFTER contour size"<<contours[i].size()<<endl;
				for (int j = 0; j < contours[i].size();j++)
				{
					//// cout<<"2D coordinates " <<contours[i][j]<<endl;

					//cout <<"3D coordinates " <<((-w / 2) + (contours[i][j].x * w )/ (imgWithContours.cols)) << " " << ((-h / 2)+ (contours[i][j].y * h) / (imgWithContours.rows)) << " " << -F << endl;
                                         direction=point3d((A + (contours[i][j].x * w )/ imgWithContours.cols), B , (C+ (contours[i][j].y * h) / imgWithContours.rows));
					//direction=point3d((A + (contours[i][j].x * w )/ imgWithContours.cols), (B + (contours[i][j].y * h) / imgWithContours.rows), C);
					////cout <<"3D coordinates " <<(A + (contours[i][j].x * w )/ (imgWithContours.cols)) << " " << (B+ (contours[i][j].y * h) / (imgWithContours.rows)) << " " << C << endl;
					//cout<<endl;
                                        cout<<"CONTOUR COORDINATES: x="<<contours[i][j].x<<" y="<<contours[i][j].y<<endl;
					rotation_based_in_euler_angles( roll, pitch, yaw);


						   if (octree->castRay(origin, direction, end, true))
						   {


							octree->isNodeOccupied(octree->search(end));

							point3d intersection;
							bool success = octree->getRayIntersection(origin, direction, end, intersection);

							if (success)
								{

									ColorOcTreeNode* n=ctree->updateNode(intersection,true);
									n->setColor(255, 0, 0);
									ctree->updateNode(intersection.x(), intersection.y(), intersection.z(), true);
									ctree->setNodeColor(intersection.x(), intersection.y(), intersection.z(), 0,255, 0 );
									ctree->ColorOcTree::integrateNodeColor( intersection.x(), intersection.y(), intersection.z(), 0, 255, 0 );
									ctree->ColorOcTree::updateInnerOccupancy();
									myfile <<intersection.x()<<" "<< intersection.y()<< " "<< intersection.z()<<"\n";
                                                                        cout<<"ONTO THE OCTOMAP: "<<intersection.x()<<" "<< intersection.y()<< " "<< intersection.z()<<"\n";
									coordinates[l].push_back(intersection);
									k++;
									flag=true;

								}

							}
						if (!octree->search(end))
						{
							flag=false;//means that the value end is not registered
							//cout << "projection in a hole " << endl;
							octree->search(end);
							//cout << "unknown voxel hit  " << end << endl;
							if(j==contours[i].size()-1)flag=true;
						}


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
                cout<<" origin coordinates: "<<origin<<endl;
		cout << "metric..." <<metric<< endl;
		metric++;
		myTIMES.close();
		myfile.close();
		return;

}



