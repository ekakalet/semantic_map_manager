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
#include <pthread.h>
#include "Eigen/Dense"


using namespace cv;
using namespace std;
using namespace octomap;
using namespace octomath;
using namespace ClipperLib;
using namespace Eigen;
using namespace message_filters;
using namespace visualanalysis_msgs;
using namespace multidrone_msgs;

std::string sGeometricMapFile(getenv("SMM_GEOMETRIC_MAP_FILE"));
std::string sOutputFolder(getenv("SMM_OUTPUT_FOLDER"));
std::string const XML_FILE_PATH(getenv("XML_PATH"));
std::string const XML_FILE_BACKUP_PATH(getenv("XML_BACKUP_PATH"));
std::string const XML_FILE_SS_PATH(getenv("XML_SS_PATH"));
std::string const XML_FILE_MC_PATH(getenv("XML_MC_PATH"));

octomap::OcTree *octree = new octomap::OcTree(sGeometricMapFile);
ColorOcTree *ctree = new ColorOcTree(octree->getResolution());

double pixel_size_x;
double pixel_size_y;
std::vector<double> data;
double mount_position[3];
double image_width, image_height;
double cx_pixels;
double cy_pixels;
double cx; //principal point in m
double cy; //principal point in m
int numberOnes = 0;
int numberTwos = 0;
int numberThrees = 0;
int numberFours = 0;
int numberFives = 0;
int numberSixs = 0;
int numberSevens = 0;
int TotalTime = 0;
int No_callROS_Service = 0;
bool interface_test = true;

bool value;
bool value2;
bool value3;


#define M_PI           3.14159265358979323846  /* pi */
//declaration of variables

class Drone {
public:
    double h, w, fx, W, F, resolution;
    double roll, pitch, yaw;
    tf::Quaternion quat_tf;

    point3d origin, direction;
    const tf::Quaternion q;
    int kml_file_count = 0;
    int kml_clipper_count = 0;
    string path = sOutputFolder;

    pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;

    int threshold_value = 0;
    int threshold_type = 3;
    int const max_value = 255;
    int const max_type = 4;
    int const max_BINARY_value = 255;

// sensor_msgs Image message
    sensor_msgs::Image msg1;
    cv_bridge::CvImagePtr cv_ptr;

    visualanalysis_msgs::CrowdHeatmap msg;
    geometry_msgs::PoseStamped GEO_origin;

    Paths subj = Paths(1);
    Paths clip = Paths(1);
    Paths result = Paths(1);
    ros::Time timestamp;

    int metric = 0;
    float avg_htmp_fps = 0;
    int htmp_counter = 0;
    ros::Time previous_htmp_arrival_time;
    ros::Time new_htmp_arrival_time;
    ros::Duration htmp_arrival_diff;
    std::string smm_logfilename, avg_logfilename, gi_logfilename, ci_logfilename, ual_logfilename, cs_logfilename, gs_logfilename, dp_logfilename;
    std::ofstream smm_logfile, avg_FPS, gi_logfile, ci_logfile, ual_logfile, cs_it_logfile, gs_it_logfile, dp_it_logfile;
    string path_to_node;
    long double time_elapsed_ms;
    bool it_17_8 = true;
    bool it_16_8 = true;
    bool it_18_8 = true;
    multidrone_msgs::CameraStatus camera_status;
    multidrone_msgs::GimbalStatus gimbal_status;
    geometry_msgs::PoseStamped drone_pose;
    double cnt_msg = 0;


    int REQUIRED_CS_HZ = 30;
    int CS_HZ_TH = 1;
    float cs_fps, gs_fps, dp_fps;
    float cs_latency, gs_latency, dp_latency;
    int cs_counter = 0;
    ros::Time previous_cs_arrival_time, new_cs_arrival_time, previous_gs_arrival_time, new_gs_arrival_time, previous_dp_arrival_time, new_dp_arrival_time;
    ros::Duration cs_arrival_diff, gs_arrival_diff, dp_arrival_diff;
    int REQUIRED_GS_HZ = 30;
    int GS_HZ_TH = 1;
    int gs_counter = 0;
    int REQUIRED_DP_HZ = 30;
    int DP_HZ_TH = 1;
    int dp_counter = 0;

    string camera_status_topic, gimbal_status_topic, drone_pose_topic;

    ofstream myTIMES, myTIMES2, myTIMES3, file;


    cv::Mat src, dst, img, dst_gray, src_gray;
    string window_name = (char *) "Threshold Demo";


    string trackbar_type = (char *) "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
    string trackbar_value = (char *) "Value";

private:

    ros::NodeHandle n;
    int drone_id;
    message_filters::Subscriber<visualanalysis_msgs::CrowdHeatmap> image_sub;
    message_filters::Subscriber<multidrone_msgs::GimbalStatus> gimbal_sub;
    //message_filters::Subscriber<multidrone_msgs::CameraStatus> camerast_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> drone_ual_sub;
    typedef sync_policies::ApproximateTime<visualanalysis_msgs::CrowdHeatmap, multidrone_msgs::GimbalStatus, geometry_msgs::PoseStamped> MySyncPolicy; //multidrone_msgs::CameraStatus,
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    ros::Publisher CrowdAnnotations_pub;
    ros::Subscriber cs_sub;
    ros::Subscriber gs_sub;
    ros::Subscriber dp_sub;
public:

    Drone(ros::NodeHandle &node, int drone_id_) {


        n = node;
        drone_id = drone_id_;
        std::stringstream ss;
        ss << "drone_" << drone_id;
        std::string str = ss.str();


        myTIMES2.open("times_statistics_" + str + ".txt");
        myTIMES3.open("times_polygons_" + str + ".txt");

        if (interface_test) {
            // for capturing test data
            path_to_node = ros::package::getPath("ground_visual_analysis") + string("/");
            n.param<string>("smm_logfilename", smm_logfilename, "log/" + str + "_smm_status.log");
            n.param<string>("avg_logfilename", avg_logfilename, "log/" + str + "_avg_status.log");
            n.param<string>("gi_logfilename", gi_logfilename, "log/" + str + "_gi_status.log");
            n.param<string>("ci_logfilename", ci_logfilename, "log/" + str + "_ci_status.log");
            n.param<string>("ual_logfilename", ual_logfilename, "log/" + str + "_ual_status.log");
            smm_logfile.open(path_to_node + smm_logfilename);
            avg_FPS.open(path_to_node + avg_logfilename);
            gi_logfile.open(path_to_node + gi_logfilename);
            ci_logfile.open(path_to_node + ci_logfilename);
            ual_logfile.open(path_to_node + ual_logfilename);

            // Camera, Gimbal & Drone status subscribers
            n.param<string>("cs_logfilename", cs_logfilename, "log/" + str + "_camera_status_it.log");
            cs_it_logfile.open(path_to_node + cs_logfilename);
            camera_status_topic = str + string("/camera/status");
            ROS_INFO("Camera status topic: %s", camera_status_topic.c_str());
            cs_sub = n.subscribe(camera_status_topic, 1000, &Drone::cameraStatusCallback, this);
            n.param<bool>("it_17_8", it_17_8, true);

            n.param<string>("gs_logfilename", gs_logfilename, "log/" + str + "_gimbal_status_it.log");
            gs_it_logfile.open(path_to_node + gs_logfilename);
            gimbal_status_topic = str + string("/gimbal/status");
            ROS_INFO("Gimbal status topic: %s", gimbal_status_topic.c_str());
            gs_sub = n.subscribe(gimbal_status_topic, 1000, &Drone::gimbalStatusCallback, this);
            n.param<bool>("it_16_8", it_16_8, true);

            n.param<string>("dp_logfilename", dp_logfilename, "log/" + str + "_drone_pose_it.log");
            dp_it_logfile.open(path_to_node + dp_logfilename);
            drone_pose_topic = str + string("/ual/pose");
            ROS_INFO("Drone pose topic: %s", drone_pose_topic.c_str());
            dp_sub = n.subscribe(drone_pose_topic, 1000, &Drone::dronePoseCallback, this);
            n.param<bool>("it_18_8", it_18_8, true);
        }

        image_sub.subscribe(n, str + "/visual_analysis/crowd_heatmap", 1);
        gimbal_sub.subscribe(n, str + "/gimbal/status", 1);
        drone_ual_sub.subscribe(n, str + "/ual/pose", 1);
        sync.reset(new Sync(MySyncPolicy(100), image_sub, gimbal_sub, drone_ual_sub)); //camerast_sub,
        CrowdAnnotations_pub = n.advertise<visualanalysis_msgs::DynamicMapAnnotations>(
                "/visual_analysis/dynamic_map_annotations_not_fused", 1000);
    }

public:
    void DoProcess() {


        pthread_mutex_lock(&mutex);
        sync->registerCallback(boost::bind(&Drone::CrowdProcessingCallback, this, _1, _2, _3)); //_4
        pthread_mutex_unlock(&mutex);
    }

public:
    void CrowdProcessingCallback(const visualanalysis_msgs::CrowdHeatmapConstPtr &msg,
                                 const multidrone_msgs::GimbalStatusConstPtr &gimbal_msg,
                                 const geometry_msgs::PoseStampedConstPtr &drone_ual_msg) //const multidrone_msgs::CameraStatusConstPtr& camerastatus_msg,
    {
        if (interface_test) {
            //flow checks


            htmp_counter += 1;
            new_htmp_arrival_time = msg->header.stamp;
            htmp_arrival_diff += new_htmp_arrival_time - previous_htmp_arrival_time;
            avg_htmp_fps = htmp_counter / (float) htmp_arrival_diff.toSec();
            ROS_INFO("Frame: %d \t Avg FPS: %.2f", htmp_counter, avg_htmp_fps);
            avg_FPS << "Frame: " << htmp_counter << " Avg FPS: " << avg_htmp_fps << "\n" << endl;
            // set global param /interfaces/smm_vsa if fps >= 1
            if (avg_htmp_fps >= 1) {
                ros::param::set("/interfaces/smm_vsa", true);
            } else {
                ros::param::set("/interfaces/smm_vsa", false);
            }

            // set previous arrival time to current arrival time
            previous_htmp_arrival_time = new_htmp_arrival_time;
            //capturing test data
            smm_logfile << msg->header.seq << " " << msg->header.stamp << endl;
            gi_logfile << gimbal_msg->header.seq << " " << gimbal_msg->header.stamp << endl;
            ual_logfile << drone_ual_msg->header.seq << " " << drone_ual_msg->header.stamp << endl;
        }
        ROS_INFO("Create a Copy of the crowd heat map from message");
        cout << "Gimbal Status Message: Roll: " << gimbal_msg->roll << " Pitch: " << gimbal_msg->pitch << " Yaw: "
             << gimbal_msg->yaw << endl;
        cout << "CrowdHeatmap  Timestamp: " << msg->header.stamp << endl;
        cout << "Gimbal Status Timestamp: " << gimbal_msg->header.stamp << endl;
        cout << "Drone UAL Timestamp: " << drone_ual_msg->header.stamp << endl;
        timestamp = msg->header.stamp;

        try {

            // copy sensor_msgs::Image
            msg1 = msg->heatmap;

            // convert sensor_msgs::Image to cv_bridge::CvImagePtr, pointer to image
            cv_bridge::CvImagePtr cv_ptr;

            cv_ptr = cv_bridge::toCvCopy(msg1, "mono8");

            // convert object to cv::Mat
            cv::Mat src = cv_ptr->image;

            ROS_INFO("Done imread");
            //Convert the image to Gray
            src_gray = src;

        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        std::clock_t c_start, c_end;
        c_start = std::clock();

        ROS_INFO("Ready for processing the crowd heatmap");

        /// Call the function to initialize
        Threshold_Demo(0, 0);

        cout << "end of thresholding" << endl;
        //////////////////////from camera status, gimbal status//////////////////////////////
        c_end = std::clock();
        long double time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
        cout << endl;
        cout << "end of thresholding" << time_elapsed_ms << endl;
        myTIMES.open("times.txt");
        myTIMES << "end of thresholding time " << time_elapsed_ms << "\n" << endl;
        cout << endl;
        // roll (x-axis rotation)
        // pitch (y-axis rotation)
        // yaw (z-axis rotation)

        GEO_origin.pose.position = drone_ual_msg->pose.position;
        GEO_origin.pose.position.x = GEO_origin.pose.position.x + mount_position[0];
        GEO_origin.pose.position.y = GEO_origin.pose.position.y + mount_position[1];
        GEO_origin.pose.position.z = GEO_origin.pose.position.z + mount_position[2];
        origin = point3d(GEO_origin.pose.position.x, GEO_origin.pose.position.y, GEO_origin.pose.position.z);
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!resolution of input image!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//

        //w=dst.cols *(camerastatus_msg->sensorWidth)*0.001/640;//width in m according to sensor size width of blackmagic: 12.48*0.001/1920
        //h=dst.rows *(camerastatus_msg->sensorHeight)*0.001/360;//height in m according to sensor size height of blackmagic: 7.02*0.001/1080
        w = dst.cols * (12.48 * 0.001) / image_width;
        h = dst.rows * (7.02 * 0.001) / image_height;
        cout << "w=" << w << " h=" << h << endl;
        //F=30.0473*0.001;//(camerastatus_msg->focalLength)*0.001;//0.018;//initial value 0.008,  FL-Range=14mm-42mm;//21.5569991*0.001;//
        F = 14 * 0.001;
        cout << "FOCAL LENGTH :" << F << endl;
        resolution = octree->getResolution();

        cx = 0;//(900.259497081249)*0.0065*0.001; //principal point in m
        cy = 0;//(590.2505880129497)*0.0065*0.001; //principal point in m
        cout << "cx=" << cx << " cy=" << cy << endl;
        //gimbal orientation based on euler angles
        roll = (M_PI / 180) * gimbal_msg->roll;
        pitch = (M_PI / 180) * gimbal_msg->pitch;
        yaw = (M_PI / 180) * gimbal_msg->yaw;
        dst_gray = dst.clone();
        find_contours_and_project(dst_gray, octree, ctree, w, h, F, roll, pitch, yaw);


    }

/**
 * @function Threshold_Demo
 */
public:
    void Threshold_Demo(int, void *) {
        /* 0: Binary
           1: Binary Inverted
           2: Threshold Truncated
           3: Threshold to Zero
           4: Threshold to Zero Inverted
         */

        //threshold( src_gray, dst, threshold_value=85, max_BINARY_value,threshold_type=0 );//=67
        threshold(src_gray, dst, threshold_value = 57, max_BINARY_value,
                  threshold_type = 0);//=47_untill 15_11_2018(less regions)// =25 wellperforming(more regions)
    }
/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////2D_simplification//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
public:
    double PerpendicularDistance2D(const Point &pt, const Point &lineStart, const Point &lineEnd) {
        double dx = lineEnd.x - lineStart.x;
        double dy = lineEnd.y - lineStart.y;

        //Normalise
        double mag = pow(pow(dx, 2.0) + pow(dy, 2.0), 0.5);
        if (mag > 0.0) {
            dx /= mag;
            dy /= mag;
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

        return pow(pow(ax, 2.0) + pow(ay, 2.0), 0.5);
    }

public:
    void RamerDouglasPeucker2D(const vector<Point> &pointList, double epsilon, vector<Point> &out) {
        if (pointList.size() < 2)
            throw invalid_argument("Not enough points to simplify");

        // Find the point with the maximum distance from line between start and end
        double dmax = 0.0;
        size_t index = 0;
        size_t end = pointList.size() - 1;
        for (size_t i = 1; i < end; i++) {
            double d = PerpendicularDistance2D(pointList[i], pointList[0], pointList[end]);
            if (d > dmax) {
                index = i;
                dmax = d;
            }
        }

        // If max distance is greater than epsilon, recursively simplify
        if (dmax > epsilon) {
            // Recursive call
            vector<Point> recResults1;
            vector<Point> recResults2;
            vector<Point> firstLine(pointList.begin(), pointList.begin() + index + 1);
            vector<Point> lastLine(pointList.begin() + index, pointList.end());
            RamerDouglasPeucker2D(firstLine, epsilon, recResults1);
            RamerDouglasPeucker2D(lastLine, epsilon, recResults2);

            // Build the result list
            out.assign(recResults1.begin(), recResults1.end() - 1);
            out.insert(out.end(), recResults2.begin(), recResults2.end());
            if (out.size() < 2)
                throw runtime_error("Problem assembling output");
        } else {
            //Just return start and end points
            out.clear();
            out.push_back(pointList[0]);
            out.push_back(pointList[end]);
        }
    }

public:
    double PerpendicularDistance3D(point3d &pt, point3d &lineStart, point3d &lineEnd) {
        double dx = lineEnd.x() - lineStart.x();
        double dy = lineEnd.y() - lineStart.y();

        //Normalise
        double mag = pow(pow(dx, 2.0) + pow(dy, 2.0), 0.5);
        if (mag > 0.0) {
            dx /= mag;
            dy /= mag;
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

        return pow(pow(ax, 2.0) + pow(ay, 2.0), 0.5);
    }

public:
    void RamerDouglasPeucker3D(vector<point3d> &pointList, double epsilon, vector<point3d> &out) {
        if (pointList.size() < 2)
            throw invalid_argument("Not enough points to simplify");

        // Find the point with the maximum distance from line between start and end
        double dmax = 0.0;
        size_t index = 0;
        size_t end = pointList.size() - 1;
        for (size_t i = 1; i < end; i++) {
            double d = PerpendicularDistance3D(pointList[i], pointList[0], pointList[end]);
            if (d > dmax) {
                index = i;
                dmax = d;
            }
        }

        // If max distance is greater than epsilon, recursively simplify
        if (dmax > epsilon) {
            // Recursive call
            vector<point3d> recResults1;
            vector<point3d> recResults2;
            vector<point3d> firstLine(pointList.begin(), pointList.begin() + index + 1);
            vector<point3d> lastLine(pointList.begin() + index, pointList.end());
            RamerDouglasPeucker3D(firstLine, epsilon, recResults1);
            RamerDouglasPeucker3D(lastLine, epsilon, recResults2);

            // Build the result list
            out.assign(recResults1.begin(), recResults1.end() - 1);
            out.insert(out.end(), recResults2.begin(), recResults2.end());
            if (out.size() < 2)
                throw runtime_error("Problem assembling output");
        } else {
            //Just return start and end points
            out.clear();
            out.push_back(pointList[0]);
            out.push_back(pointList[end]);
        }
    }


//-----------------------------------------------------------------------------------------------------------

public:
    void publish_polygons(vector<vector<point3d> > elements) {

        ofstream myfile;
        myfile.open("polygons.txt");
        Paths poly = Paths(elements.size());
        int g = 0;
        for (std::size_t i = 0; i < elements.size(); ++i) {

            if (elements[i].size() != 0) {
                elements[i].push_back(elements[i].at(
                        0));// push back the first element in order to create a closed polyline for the polygon
                myfile << g << " " << "\n";
                poly[g].clear();
                for (std::size_t j = 0;
                     j < elements[i].size(); j++) {// fraction to 10000 in order to be depicted to mymaps

                    poly[g].push_back(IntPoint(elements[i][j].x() * (100000000000), elements[i][j].y() * (100000000000),
                                               elements[i][j].z() * (100000000000)));
                    cout << "Created vertex polygon " << poly[g][j].X << " " << poly[g][j].Y << endl;
                    myfile << poly[g][j].X << " " << poly[g][j].Y << "\n";
                }
                cout << "Created polygon " << endl;
                cout << g << "**************************" << endl;
                g++;
            }
        }

        myfile.close();
        write_clipper_polygon_result_to_kml(poly);
        cout << "write_clipper_polygon_result_to_kml..." << endl;
    };
//-----------------------------------------------------------------------------------------------------------
public:
    std::string sort_in_counter_clockwise(vector<vector<point3d> > elements) {
        std::string ss;
        std::ostringstream oss;
        ofstream myfile;
        myfile.open("polygons.txt");
        Paths poly = Paths(1);
        for (std::size_t i = 0; i < elements.size(); ++i) {

            if (elements[i].size() != 0) {
                oss << "<Placemark>\n"
                    << "<name>LinearRing" << i << "</name>\n"
                    << "<description>This is a polygon of crowded location</description>\n"
                    << "<styleUrl>#msn_ylw-pushpin</styleUrl>\n"

                    << "<Polygon>\n"
                    << "<extrude>1</extrude>\n"
                    << "<tessellate>1</tessellate>\n"
                    << "<altitudeMode>clampToGround</altitudeMode>\n"
                    << "<outerBoundaryIs>\n"
                    << "<LinearRing>\n"
                    << "<coordinates>\n";

                elements[i].push_back(elements[i].at(
                        0));// push back the first element in order to create a closed polyline for the polygon


                myfile << i << " " << "\n";
                poly[0].clear();
                for (std::size_t j = 0;
                     j < elements[i].size(); j++) {// fraction to 10000 in order to be depicted to mymaps
                    oss << elements[i].at(j).x() / 10000 << "," << elements[i].at(j).y() / 10000 << ","
                        << elements[i].at(j).z() / 10000 << " \n";


                    poly[0].push_back(IntPoint(elements[i][j].x() * (100000000000), elements[i][j].y() * (100000000000),
                                               elements[i][j].z() * (100000000000)));
                    cout << "Created vertex polygon " << poly[0][j].X << " " << poly[0][j].Y << endl;
                    myfile << poly[0][j].X << " " << poly[0][j].Y << "\n";
                }

                cout << "Created polygon " << endl;
                cout << i << "**************************" << endl;
                write_clipper_polygon_result_to_kml(poly);
                cout << "write_clipper_polygon_result_to_kml..." << endl;


                oss << "</coordinates>\n"
                    << "</LinearRing>\n"
                    << "</outerBoundaryIs>\n"
                    << "</Polygon>\n"
                    << "</Placemark>\n";

            } else {
                oss << "\n";
            }

        }

        myfile.close();

        ss = oss.str();
        return ss;
    };
public:
    std::string FormatPolygonCoordinates(vector<vector<point3d> > elements) {
        std::string ss;
        std::ostringstream oss;
        oss << sort_in_counter_clockwise(elements);
        ss = oss.str();
        return ss;

    }

public:
    std::string FormatPolygon(vector<vector<point3d> > elements) {
        std::ostringstream ss;
        ss << "<Document>\n"
           << "<name>crowded location</name>\n"
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
           << "<Pair>\n"
           << "<key>normal</key>\n"
           << "<styleUrl>#sn_ylw-pushpin</styleUrl>\n"
           << "</Pair>\n"
           << "<Pair>\n"
           << "<key>highlight</key>\n"
           << "<styleUrl>#sh_ylw-pushpin</styleUrl>\n"
           << "</Pair>\n"
           << "</StyleMap>\n"
           << "<Style id='sn_ylw-pushpin'>\n"
           << "<IconStyle>\n"
           << "<scale>1.1</scale>\n"
           << "<Icon>\n"
           << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n"
           << "</Icon>\n"
           << "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n"
           << "</IconStyle>\n"
           << " <PolyStyle>\n"
           << "<color>ff00ff55</color>\n"
           << "</PolyStyle>\n"
           << "</Style>\n"
           << FormatPolygonCoordinates(elements) << "\n"
           << "</Document>\n";

        return ss.str();
    }

public:
    void write_to_kml(vector<vector<point3d> > coordinates) {
        std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
        handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
        kml_file_count++;
        // Open the KML file for writing:
        stringstream filename;
        filename << path << "//Sample_" << ".kml"; //<< std::setw(4) << std::setfill('0') << kml_file_count << ".kml";
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
public:
    void quaternion_to_euler_angles(const tf::Quaternion &q) {
        Vector3d t, d;
        Matrix3d mat;
        tf::Matrix3x3 m(q); //possibly quaternion in XYZ sequence

        m.getRPY(roll, pitch, yaw, 1);
        std::cout << "In Rads: Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

    }

public:
    void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[3]) {
        res[0] = atan2(r31, r32);
        res[1] = asin(r21);
        res[2] = atan2(r11, r12);
    }

public:
    void quaternion_to_euler_angles_rotSeq(const tf::Quaternion &q) {
        double res[3];
        //case xyz:
        threeaxisrot(-2 * (q.y() * q.z() - q.w() * q.x()),
                     q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z(),
                     2 * (q.x() * q.z() + q.w() * q.y()),
                     -2 * (q.x() * q.y() - q.w() * q.z()),
                     q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z(),
                     res);

        roll = res[2];//0; //rotation around the x axis with theta1 in radians
        pitch = res[1]; //0; //rotation around the y axis with theta2 in radians
        yaw = res[0];//0; //rotation around the z axis with theta3 in radians
    }


public:
    void rotation_based_in_euler_angles(double roll, double pitch,
                                        double yaw) {//augmenting the angles and translation/rotation matrices
        double M[4][4];
        //angles in radians
        double theta1 = roll;//0; //rotation around the x axis with theta1 in radians
        double theta2 = pitch; //0; //rotation around the y axis with theta2 in radians
        double theta3 = yaw;//0; //rotation around the z axis with theta3 in radians
        //coordinates of COP
        double xgps = origin.x();
        double ygps = origin.y();
        double zgps = origin.z();

        //Rz*Ry*Rx (XYZ)
        M[0][0] = cos(theta2) * cos(theta3);
        M[0][1] = cos(theta3) * sin(theta1) * sin(theta2) - cos(theta1) * sin(theta3);
        M[0][2] = sin(theta1) * sin(theta3) + cos(theta1) * cos(theta3) * sin(theta2);
        M[0][3] = xgps + ygps * (cos(theta1) * sin(theta3) - cos(theta3) * sin(theta1) * sin(theta2)) -
                  zgps * (sin(theta1) * sin(theta3) + cos(theta1) * cos(theta3) * sin(theta2)) -
                  xgps * cos(theta2) * cos(theta3);
        M[1][0] = cos(theta2) * sin(theta3);
        M[1][1] = cos(theta1) * cos(theta3) + sin(theta1) * sin(theta2) * sin(theta3);
        M[1][2] = cos(theta1) * sin(theta2) * sin(theta3) - cos(theta3) * sin(theta1);
        M[1][3] = ygps - ygps * (cos(theta1) * cos(theta3) + sin(theta1) * sin(theta2) * sin(theta3)) +
                  zgps * (cos(theta3) * sin(theta1) - cos(theta1) * sin(theta2) * sin(theta3)) -
                  xgps * cos(theta2) * sin(theta3);
        M[2][0] = -sin(theta2);
        M[2][1] = cos(theta2) * sin(theta1);
        M[2][2] = cos(theta1) * cos(theta2);
        M[2][3] = zgps + xgps * sin(theta2) - zgps * cos(theta1) * cos(theta2) - ygps * cos(theta2) * sin(theta1);
        M[3][0] = 0;
        M[3][1] = 0;
        M[3][2] = 0;
        M[3][3] = 1;

        double mult[4][1];
        double direction_B[4][1];

        //OP
        direction_B[0][0] = direction.x();//-w/2;
        direction_B[1][0] = direction.y();//-h/2;
        direction_B[2][0] = direction.z();//-F;
        direction_B[3][0] = 0; //homogeneous coordinates on vector

        // Multiplying matrix a and b and storing in array mult.
        int j = 0;
        for (int i = 0; i <= 3; ++i) {

            mult[i][j] = 0;
            for (int k = 0; k <= 3; ++k) {
                mult[i][j] += M[i][k] * direction_B[k][j];
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////
        direction = point3d(mult[0][0], mult[1][0], mult[2][0]);

    }
//-----------------------------------------------------------------------------------------------------------------------------------
public:
    void fusion_polygon_lines(Paths poly) {
        ofstream myfile;
        myfile.open("clipper_subj.txt");
        Paths subj(1);
        subj = poly;
        // poly is a polygon with vertices: elements[i][j]
        //
        for (int l = 0; l < poly.size(); l++) {
            for (int i = 0; i < subj[0].size(); i++) {

                myfile << subj[l][i].X << " " << subj[l][i].Y << subj[l][i].Z << "\n";
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

public:
    void write_clipper_polygon_result_to_kml(Paths Polygon) {
        pthread_mutex_lock(&mutex2);
        bool flag = true;
        geometry_msgs::Point32 pt;
        point3d point;
        // geometry_msgs::PolygonStamped poly;
        ofstream myfile;
        myfile.open("clipper_result.txt");
        vector<vector<point3d> > coordinates(Polygon.size());
        visualanalysis_msgs::DynamicMapAnnotations msg2;

        myTIMES3 << Polygon.size() << "\n" << endl;
        for (int l = 0; l < Polygon.size(); l++) {
            cout << "result" << l << "=" << endl;
            myfile << l << " " << l << "\n";

            coordinates[l].clear();
            if (Polygon[l].size() >= 6) {
                for (int i = 0; i < Polygon[l].size(); i++) {
                    ////cout<<Polygon[l][i].X<<" "<<Polygon[l][i].Y<<endl;
                    //myfile <<Polygon[l][i].X<<" "<<Polygon[l][i].Y<<"\n";
                    pt.x = Polygon[l][i].X;///100000000000;//maybe we lost decimal digits due to conversion Int to float32
                    pt.y = Polygon[l][i].Y;///100000000000;
                    pt.z = Polygon[l][i].Z;//0
                    myfile << Polygon[l][i].X << " " << Polygon[l][i].Y << " " << Polygon[l][i].Z << "\n";
                    point.x() = pt.x;//in order to be depicted to mymaps
                    point.y() = pt.y;//in order to be depicted to mymaps
                    point.z() = 0;//pt.z;
                    coordinates[l].push_back(point);
                }

                coordinates[l].push_back(coordinates[l].at(
                        0));// push back the first element in order to create a closed polyline for the polygon
                flag = true;
            } else {
                flag = false;
            }
            if (flag == true) {
                myfile << Polygon[l][0].X << " " << Polygon[l][0].Y << " " << Polygon[l][0].Z << "\n";
                cout << "BEFORE contour size" << coordinates[l].size() << endl;
                vector<point3d> pointListOut;
                if (coordinates[l].size() >= 10) { //while 650
                    //limit 250 : make simplification on the polygons so that MyMaps could display it
                    cout << "IN_BEFORE_PROJECTION" << endl;
                    RamerDouglasPeucker3D(coordinates[l], 2.7e+13, pointListOut);//0.008, 6, 2.7e+13,
                    coordinates[l] = pointListOut;
                }
                cout << "AFTER contour size" << coordinates[l].size() << endl;
                geometry_msgs::PolygonStamped poly;
                //prepare coordinates in order to publish on the DynamicAnnotations topic
                for (int i = 0; i < coordinates[l].size(); i++) {
                    pt.x = coordinates[l][i].x();///10000000000000;//<-kmlbased/100000000000;//in order to not lost decimal digits due to conversion Int to float32 the scaling must be done in the receiver
                    pt.y = coordinates[l][i].y();///10000000000000;///100000000000;
                    pt.z = 0;//coordinates[l][i].z()/10000000000000;///100000000000;//0
                    poly.polygon.points.push_back(pt);

                }


                cout << "Created polygon " << endl;

                //create the msg polygonstamped_array
                poly.polygon.points.clear();
                poly.header.seq = l;
                poly.header.stamp = timestamp;
                poly.header.frame_id = '1';
                msg2.areas.push_back(poly);
                cout << l << "Final************************" << endl;
            }

            cout << "created the msg" << endl;
            msg2.header.seq = cnt_msg;
            msg2.header.stamp = timestamp;
            msg2.header.frame_id = '1';


            CrowdAnnotations_pub.publish(msg2);

            msg2.areas.clear();
            myfile.close();
            cnt_msg++;


            cout << "Done_created_Coordinates..." << endl;
            std::ofstream handle;

// http://www.cplusplus.com/reference/ios/ios/exceptions/
// Throw an exception on failure to open the file or on a write error.
            handle.exceptions(std::ofstream::failbit | std::ofstream::badbit);
            kml_clipper_count++;
// Open the KML file for writing:
            stringstream filename;
            filename << path
                     << "//Sample_clipper_result.kml";
            handle.open(filename.str().c_str());
// Write to the KML file:
            handle << "<?xml version='1.0' encoding='utf-8'?>\n";
            handle << "<kml xmlns='http://www.opengis.net/kml/2.2'\n";
            handle << "xmlns:gx='http://www.google.com/kml/ext/2.2'\n";
            handle << "xmlns:kml='http://www.opengis.net/kml/2.2'\n";
            handle << "xmlns:atom='http://www.w3.org/2005/Atom'>\n";


            handle << "<Document>\n";
            handle << "<name>crowded location</name>\n";
            handle << "<Style id='sh_ylw-pushpin'>\n";
            handle << "<IconStyle>\n";
            handle << "<scale>1.3</scale>\n";
            handle << "<Icon>\n";
            handle << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n";
            handle << "</Icon>\n";
            handle << "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n";
            handle << "</IconStyle>\n";
            handle << "<PolyStyle>\n";
            handle << "<color>ff00ff55</color>\n";
            handle << "</PolyStyle>\n";
            handle << "</Style>\n";
            handle << "<StyleMap id='msn_ylw-pushpin'>\n";
            handle << "<Pair>\n";
            handle << "<key>normal</key>\n";
            handle << "<styleUrl>#sn_ylw-pushpin</styleUrl>\n";
            handle << "</Pair>\n";
            handle << "<Pair>\n";
            handle << "<key>highlight</key>\n";
            handle << "<styleUrl>#sh_ylw-pushpin</styleUrl>\n";
            handle << "</Pair>\n";
            handle << "</StyleMap>\n";
            handle << "<Style id='sn_ylw-pushpin'>\n";
            handle << "<IconStyle>\n";
            handle << "<scale>1.1</scale>\n";
            handle << "<Icon>\n";
            handle << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>\n";
            handle << "</Icon>\n";
            handle << "<hotSpot x='20' y='2' xunits='pixels' yunits='pixels'/>\n";
            handle << "</IconStyle>\n";
            handle << " <PolyStyle>\n";
            handle << "<color>ff00ff55</color>\n";
            handle << "</PolyStyle>\n";
            handle << "</Style>\n";

            handle << FormatPolygon_clipper(coordinates);

            handle << "</Document>\n";

            handle << "</kml>\n";
            handle.close();
            cout << "done writing coordinates to kml..." << endl;
        }
        pthread_mutex_unlock(&mutex2);
    }

public:
    std::string FormatPolygon_clipper(vector<vector<point3d> > elements) {
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
                oss << elements[i].at(j).x() / 100000000000000000 << "," << elements[i].at(j).y() / 100000000000000000
                    << "," << elements[i].at(j).z() / 100000000000000000 << " \n";
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

public:
    void
    find_contours_and_project(Mat dst, OcTree *octree, ColorOcTree *ctree, double w, double h, double F, double roll,
                              double pitch, double yaw) {

        ofstream myfile;
        myfile.open("intersection.txt");
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
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = Scalar(255, 255, 255);
            drawContours(imgWithContours, contours, i, color, 1, 8, hierarchy, 0);
        }
        c_end2 = std::clock();
        long double time_elapsed_ms2 = 1000.0 * (c_end2 - c_start2) / CLOCKS_PER_SEC;
        cout << endl;
        cout << "find and draw contours time" << time_elapsed_ms2 << endl;
        cout << endl;

        vector<vector<point3d> > coordinates(contours.size());
        point3d end;
        bool flag = true;
        c_start3 = std::clock();
        direction.x() = -w / 2;
        direction.y() = -h / 2;
        direction.z() = -F;
        cout << "direction " << direction.x() << " " << direction.y() << " " << direction.z() << endl;
        double A = direction.x();//
        double B = direction.y();//
        double C = direction.z();//
        int l = 0;
        for (int i = 0; i < contours.size(); i++) {
            int k = 0;
            //////////////////////////////////////////
            cout << "BEFORE contour size" << contours[i].size() << endl;
            vector<Point> pointListOut;
            if (contours[i].size() >= 10) { //while 650
                //limit 250 : make simplification on the polygons so that MyMaps could display it
                cout << "IN_BEFORE_PROJECTION" << endl;
                RamerDouglasPeucker2D(contours[i], 6, pointListOut);//0.008, 2.7e+13,
                contours[i] = pointListOut;
            }
            ///////////////////////////////////////////////
            cout << "AFTER contour size" << contours[i].size() << endl;
            for (int j = 0; j < contours[i].size(); j++) {
                direction = point3d((A + (contours[i][j].x * w) / imgWithContours.cols),
                                    (B + (contours[i][j].y * h) / imgWithContours.rows), C);

                rotation_based_in_euler_angles(roll, pitch, yaw);


                if (octree->castRay(origin, direction, end, true)) {


                    octree->isNodeOccupied(octree->search(end));

                    point3d intersection;
                    bool success = octree->getRayIntersection(origin, direction, end, intersection);

                    if (success) {
                        myfile << intersection.x() << " " << intersection.y() << " " << intersection.z() << "\n";

                        coordinates[l].push_back(intersection);
                        k++;
                        flag = true;

                    } else {
                        if (k == 0) {
                            flag = false;

                        } else {
                            if (flag == false) {

                                flag = false;

                            } else {
                                //coordinates[l].push_back(coordinates[l][k-1]);
                                //k++;
                                flag = true;
                            }


                        }


                    }

                } else {

                    if (k == 0) {
                        flag = false;

                    } else {
                        if (flag == false) {

                            flag = false;

                        } else {
                            //coordinates[l].push_back(coordinates[l][k-1]);
                            //k++;
                            flag = true;
                        }


                    }
                    if (!octree->search(end)) {
                        flag = false;//means that the value end is not registered
                        //cout << "projection in a hole " << endl;
                        octree->search(end);
                        //cout << "unknown voxel hit  " << end << endl;
                        if (j == contours[i].size() - 1)flag = true;
                    }
                }

            }
            //cout<<" "<<coordinates[l].size()<<endl;
            if (flag == true) {
                l++;

            }
        }

        c_end3 = std::clock();
        long double time_elapsed_ms3 = 1000.0 * (c_end3 - c_start3) / CLOCKS_PER_SEC;
        myTIMES << "Project_time" << time_elapsed_ms3 << "\n";
        cout << endl;
        cout << "Project_time" << time_elapsed_ms3 << endl;
        cout << endl;
        //ctree->writeBinary("my_tree_projected.bt");
        //cout<<"done writing of binary tree..."<<endl;

        publish_polygons(coordinates);
        //write_to_kml(coordinates);
        coordinates.clear();
        cout << "done writing coordinates to kml..." << endl;
        c_end = std::clock();
        time_elapsed_ms = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
        myTIMES << "Total_time" << time_elapsed_ms << "\n";
        myTIMES2 << time_elapsed_ms << "\n" << endl;
        cout << endl;
        cout << time_elapsed_ms << endl;
        if (drone_id == 1) TotalTime = TotalTime + time_elapsed_ms;
        cout << endl;
        cout << "roll=" << roll << " pitch=" << pitch << " yaw=" << yaw << endl;
        if (drone_id == 1) numberOnes++;
        if (drone_id == 2) numberTwos++;
        if (drone_id == 3) numberThrees++;
        if (drone_id == 4) numberFours++;
        if (drone_id == 5) numberFives++;
        if (drone_id == 6) numberSixs++;
        if (drone_id == 7) numberSevens++;

        cout << "Drone_ID..." << drone_id << endl;

        cout << "metric..." << metric << endl;
        cout << "DRONE ONE..." << numberOnes << endl;
        cout << "DRONE TWO..." << numberTwos << endl;
        cout << "DRONE THREE..." << numberThrees << endl;
        cout << "DRONE FOUR..." << numberFours << endl;
        cout << "DRONE FIVE..." << numberFives << endl;
        cout << "DRONE SIX..." << numberSixs << endl;
        cout << "DRONE SEVEN..." << numberSevens << endl;
        cout << "TotalTime..." << TotalTime << endl;

        metric++;
        myTIMES.close();
        myfile.close();
        return;

    }

// IT 17-8
public:
    void cameraStatusCallback(const multidrone_msgs::CameraStatus &msg) {
        ros::Time now = ros::Time::now();
        camera_status = msg;
        if (it_17_8) {
            // Keep count, Hz & latency
            cs_counter += 1;
            new_cs_arrival_time = msg.header.stamp;
            cs_arrival_diff = new_cs_arrival_time - previous_cs_arrival_time;
            cs_latency = (float) (now - new_cs_arrival_time).toNSec();
            cs_fps = 1 / (float) cs_arrival_diff.toSec();

            // Log Hz
            ROS_INFO("Camera status: %5d - Hz: %.2f Latency: %.0f (ms)", msg.header.seq, cs_fps, cs_latency / 1000);
            cs_it_logfile << msg.header.seq << " " << msg.header.stamp << endl;

            // Set global param /interfaces/smm_ci if fps >= 30
            if (cs_fps >= REQUIRED_CS_HZ - CS_HZ_TH) {
                ros::param::set("/interfaces/smm_ci", true);
            } else {
                ros::param::set("/interfaces/smm_ci", false);
            }
            // TODO: we probably just need the secs
            ros::param::set("/interfaces/smm_last_receved_cs/sec", (int) new_cs_arrival_time.sec);
            ros::param::set("/interfaces/smm_last_receved_cs/nsec", (int) new_cs_arrival_time.nsec);

            previous_cs_arrival_time = msg.header.stamp;
        }
    }

// IT 16-8
public:
    void gimbalStatusCallback(const multidrone_msgs::GimbalStatus &msg) {
        ros::Time now = ros::Time::now();
        gimbal_status = msg;

        if (it_16_8) {
            // Keep count, Hz & latency
            gs_counter += 1;
            new_gs_arrival_time = msg.header.stamp;
            gs_arrival_diff = new_gs_arrival_time - previous_gs_arrival_time;
            gs_fps = 1 / (float) gs_arrival_diff.toSec();
            gs_latency = (float) (now - new_gs_arrival_time).toNSec();

            // Log hz & latency
            ROS_INFO("Gimbal status: %5d - Hz: %.2f Latency: %.0f (ms)", msg.header.seq, gs_fps, gs_latency / 1000);
            gs_it_logfile << msg.header.seq << " " << msg.header.stamp << endl;

            // Set global param /interfaces/smm_ci if fps >= 30
            if (gs_fps >= REQUIRED_GS_HZ - GS_HZ_TH) {
                ros::param::set("/interfaces/smm_gi", true);
            } else {
                ros::param::set("/interfaces/smm_gi", false);
            }
            ros::param::set("/interfaces/smm_last_receved_cs/sec", (int) new_cs_arrival_time.sec);
            ros::param::set("/interfaces/smm_last_receved_cs/nsec", (int) new_cs_arrival_time.nsec);

            previous_gs_arrival_time = new_gs_arrival_time;
        }
    }

// IT 18-8
public:
    void dronePoseCallback(const geometry_msgs::PoseStamped &msg) {
        ros::Time now = ros::Time::now();
        drone_pose = msg;
        if (it_18_8) {
            // Keep count, Hz & latency
            dp_counter += 1;
            new_dp_arrival_time = msg.header.stamp;
            dp_arrival_diff = new_dp_arrival_time - previous_dp_arrival_time;
            dp_fps = 1 / (float) dp_arrival_diff.toSec();
            dp_latency = (float) (now - new_dp_arrival_time).toNSec();

            // Log hz & latency
            ROS_INFO("Drone pose: %5d - Hz: %.2f Latency: %.0f (ms)", msg.header.seq, dp_fps, dp_latency / 1000);
            dp_it_logfile << msg.header.seq << " " << msg.header.stamp << endl;

            // Set global param /interfaces/smm_ci if fps >= 30
            if (dp_fps >= REQUIRED_DP_HZ - DP_HZ_TH) {
                ros::param::set("/interfaces/smm_ual", true);
            } else {
                ros::param::set("/interfaces/smm_ual", false);
            }
            ros::param::set("/interfaces/smm_last_receved_dp/sec", (int) new_dp_arrival_time.sec);
            ros::param::set("/interfaces/smm_last_receved_dp/nsec", (int) new_dp_arrival_time.nsec);

            previous_dp_arrival_time = msg.header.stamp;
        }
    }

};//end of class Drone



bool append_fragment(pugi::xml_node target, const pugi::xml_document &cached_fragment) {
    for (pugi::xml_node child = cached_fragment.first_child(); child; child = child.next_sibling())
        target.append_copy(child);
}


bool DoPostSemanticAnnotations(multidrone_msgs::PostSemanticAnnotations::Request &req,
                               multidrone_msgs::PostSemanticAnnotations::Response &res) {
    //execution only once at the beginning
    ROS_INFO("receiving KML annotations");

    pugi::xml_document doc;

    ros::param::get("~/interfaces/smm_ss", value);
    ros::param::get("~/interfaces/smm_mc", value2);
    ros::param::get("~once_executed", value3);
    if (value3) {
        No_callROS_Service++;
    }
    if (No_callROS_Service == 0)// AND if it is first time
    {

        if (!value) //if we do not have input from SS value==false
        {

            if (!value2) //if we do not have input from MC value==false
            {
                std::string _semantic_map = "<kml><Document><Folder><name>Supervisor</name></Folder><Folder><name>Dashboard</name></Folder></Document></kml>";

                // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
                cout << "Initialize" << endl;
                doc.load_string(_semantic_map.c_str());
            } else {
                //initialize "kml_path.kml" from the supervision station and we add the target trajectory information from the Mission Controller
                std::ifstream ifs_file(XML_FILE_PATH.c_str());
                std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());

                // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
                cout << "Initialize" << endl;
                doc.load_string(_semantic_map.c_str());

            }
        } else {
            //initialize "kml_path.kml" from the supervision station and we add the target trajectory information from the Mission Controller
            std::ifstream ifs_file(XML_FILE_PATH.c_str());
            std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());

            // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
            cout << "Initialize" << endl;
            doc.load_string(_semantic_map.c_str());
        }
    } else // if it is NOT first time
    {
        if (!value3)// if it is NOT first time
        {
            if (!value) //if we do not have input from SS
            {

                if (value2) //if we  have input from MC value==true
                {
                    //initialize "kml_path.kml" from the supervision station and we add the target trajectory information from the Mission Controller
                    std::ifstream ifs_file(XML_FILE_PATH.c_str());
                    std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)),
                                              std::istreambuf_iterator<char>());

                    // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
                    cout << "Initialize" << endl;
                    doc.load_string(_semantic_map.c_str());
                } else {
                    std::string _semantic_map = "<kml><Document><Folder><name>Supervisor</name></Folder><Folder><name>Dashboard</name></Folder></Document></kml>";

                    // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
                    cout << "Initialize" << endl;
                    doc.load_string(_semantic_map.c_str());
                }
            } else {
                //initialize "kml_path.kml" from the supervision station and we add the target trajectory information from the Mission Controller
                std::ifstream ifs_file(XML_FILE_PATH.c_str());
                std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());

                // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
                cout << "Initialize" << endl;
                doc.load_string(_semantic_map.c_str());
            }

        } else //value3==true the initializer had executed once
        {

            //initialize "kml_path.kml" from the supervision station and we add the target trajectory information from the Mission Controller
            std::ifstream ifs_file(XML_FILE_PATH.c_str());
            std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());

            // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.
            cout << "Initialize" << endl;
            doc.load_string(_semantic_map.c_str());
        }


    }


    if (req.clear == true) {
        for (pugi::xml_node Folder : doc.child("kml").child("Document").children()) {
            if (std::string(Folder.child("name").child_value()) == std::string("Dashboard")) {

                bool keepremoving = true;
                while (keepremoving) {
                    keepremoving = Folder.remove_child("Placemark");
                }
            }
        }
    } else {

        // Read XML string into temporary document
        const std::string AnnotationsString = req.semantic_map;
        cout << AnnotationsString << endl;
        pugi::xml_document tmpDoc;

        if (tmpDoc.load_string(AnnotationsString.c_str())) {

            pugi::xml_node kml2 = tmpDoc.child("kml");
            pugi::xml_node Document2 = kml2.child("Document");
            pugi::xml_node Folder2 = Document2.child("Folder");
            cout << "root" << endl;


            for (pugi::xml_node Folder : doc.child("kml").child("Document").children()) {
                // A valid XML document must have a single root node

                if ((req.supervisor_or_dashboard == true) && (std::string(Folder.child("name").child_value()) ==
                                                              std::string(
                                                                      "Dashboard")))//false if KML from supervisor, true if from dashboard
                {
                    cout << "DASHBOARD" << endl;

                    if ((req.clear == false) &&
                        (std::string(Folder.child("name").child_value()) == std::string("Dashboard"))) {
                        cout << std::string(Folder.child("name").child_value()) << endl;
                        cout << Folder.name() << endl;
                        for (pugi::xml_node Placemark2 : tmpDoc.child("kml").child("Document").children()) {
                            ostringstream oss;
                            Placemark2.print(oss);
                            string xml_SS = oss.str();
                            cout << xml_SS << endl;
                            pugi::xml_document tmpDoc2;
                            if (tmpDoc2.load_string(xml_SS.c_str())) {

                                if (append_fragment(Folder, tmpDoc2)) {
                                    // Create child node to hold external XML data
                                    if (interface_test) {
                                        tmpDoc.save_file(XML_FILE_SS_PATH.c_str(), PUGIXML_TEXT("  "));


                                    }
                                    ros::param::set("~/interfaces/smm_mc", true);
                                    ros::param::set("~once_executed", true);
                                }
                            }
                        }
                    }

                } else if ((req.supervisor_or_dashboard == false) &&
                           (std::string(Folder.child("name").child_value()) == std::string("Supervisor"))) {
                    cout << "SUPERVISOR" << endl;

                    if (std::string(Folder.child("name").child_value()) == std::string("Supervisor")) {
                        cout << std::string(Folder.child("name").child_value()) << endl;
                        cout << Folder.name() << endl;

                        bool keepremoving = true;
                        while (keepremoving) {
                            keepremoving = Folder.remove_child("Placemark");
                        }

                        tmpDoc.child("kml").child("Document").child("Folder").remove_child("name");
                        for (pugi::xml_node Placemark2 : tmpDoc.child("kml").child("Document").child(
                                "Folder").children()) {
                            ostringstream oss;
                            Placemark2.print(oss);
                            string xml_SS = oss.str();
                            cout << xml_SS << endl;
                            pugi::xml_document tmpDoc2;


                            if (tmpDoc2.load_string(xml_SS.c_str())) {

                                if (append_fragment(Folder, tmpDoc2)) {
                                    // Create child node to hold external XML data
                                    if (interface_test) {
                                        tmpDoc.save_file(XML_FILE_SS_PATH.c_str(), PUGIXML_TEXT("  "));


                                    }
                                    ros::param::set("~/interfaces/smm_ss", true);
                                    ros::param::set("~once_executed", true);
                                }
                            }
                        }

                    }
                }
            }
        } else {
            cout << " Error to loading" << endl;
        }
    }
    cout << "load" << endl;

    cout << "Read XML string + append " << endl;
    bool saveSucceeded = doc.save_file(XML_FILE_PATH.c_str(), PUGIXML_TEXT("  "));
    assert(saveSucceeded);
    if (!value3) //if once_executed is false - 1st time
    {
        saveSucceeded = doc.save_file(XML_FILE_BACKUP_PATH.c_str(), PUGIXML_TEXT("  "));
        assert(saveSucceeded);
    }
    cout << "end process" << endl;

    No_callROS_Service++;
    ros::param::set("~once_executed", true);
    return true;

}

bool DoGetSemanticMap(multidrone_msgs::GetSemanticMap::Request &req, multidrone_msgs::GetSemanticMap::Response &res) {

    ROS_INFO("sending back KML response");
    // Avoid reading directly from the file because of path problems when doing "git clone" or "git pull" from other computers.
    std::ifstream ifs_file(XML_FILE_PATH.c_str());
    std::string _semantic_map((std::istreambuf_iterator<char>(ifs_file)), std::istreambuf_iterator<char>());

    res.semantic_map = _semantic_map;
    return true;

}

void read_camera_settings() {
//principal point in mm of the focal plane used to projection
    cx_pixels = data[2];
    cy_pixels = data[5];
    cx = (cx_pixels) * pixel_size_x * 0.001;
    cy = (cy_pixels) * pixel_size_y * 0.001;


}

void print_info(double cx_pixels, double cy_pixels, double image_width, double image_height, double mount_position_0,
                double mount_position_1, double mount_position_2, double pixel_size_x, double pixel_size_y,
                int drone_id_) {

    cout << "Initializing semantic map manager node with parameters: " << endl;
    cout << "camera_matrix: " << cx_pixels << " " << cy_pixels << endl;
    cout << "image_width: " << image_width << endl;
    cout << "image_height: " << image_height << endl;
    cout << "mount_position: " << mount_position_0 << " " << mount_position_1 << " " << mount_position_2 << endl;
    cout << "pixel_size_x: " << pixel_size_x << endl;
    cout << "pixel_size_y: " << pixel_size_y << endl;
    cout << "drone_id_: " << drone_id_ << endl;

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "semantic_map_manager");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    pn.param<bool>("interface_test", interface_test, true);

    pn.param<bool>("/interfaces/smm_ss", value, false);
    pn.param<bool>("/interfaces/smm_mc", value2, false);
    if (pn.hasParam("once_executed")) {

        ros::param::get("~once_executed", value3);
    } else {
        pn.param<bool>("~once_executed", value3, false);
    }


    XmlRpc::XmlRpcValue my_list;
    n.getParam("/drone_sensors", my_list);
    ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < my_list.size(); ++i) {

        XmlRpc::XmlRpcValue sublist = my_list[i];
        XmlRpc::XmlRpcValue mount_position_;
        string name = sublist["name"];
        if (name == "CineCamera") {
            mount_position_ = sublist["mount_position"];
            mount_position[0] = static_cast<double>(mount_position_[0]);
            mount_position[1] = static_cast<double>(mount_position_[1]);
            mount_position[2] = static_cast<double>(mount_position_[2]);
            pixel_size_x = static_cast<double>(sublist["pixel_size_x"]);
            pixel_size_y = static_cast<double>(sublist["pixel_size_y"]);
        }
    }

    //n.getParam("kml_path", path_KML);// get kml path
    n.getParam("/camera_matrix/data", data);
    n.getParam("/image_width", image_width);
    n.getParam("/image_height", image_height);
    int drone_id_;
    n.param<int>("/drone_id", drone_id_, 1);


    read_camera_settings();

    print_info(cx_pixels, cy_pixels, image_width, image_height, mount_position[0], mount_position[1], mount_position[2],
               pixel_size_x, pixel_size_y, drone_id_);

    //Drone drone(n, drone_id_);
    Drone drone(n, 1);
    boost::thread t1(&Drone::DoProcess, &drone);
    t1.join();
    Drone drone2(n, 2);
    boost::thread t2(&Drone::DoProcess, &drone2);
    t2.join();
    Drone drone3(n, 3);
    boost::thread t3(&Drone::DoProcess, &drone3);
    t3.join();
    /*Drone drone4(n, 4);
boost::thread t4(&Drone::DoProcess, &drone4);
        t4.join();
        Drone drone5(n, 5);
        boost::thread t5(&Drone::DoProcess, &drone5);
        t5.join();
        Drone drone6(n, 6);
        boost::thread t6(&Drone::DoProcess, &drone6);
        t6.join();
        Drone drone7(n, 7);
        boost::thread t7(&Drone::DoProcess, &drone7);
        t7.join();*/



    ros::ServiceServer service_post = n.advertiseService("post_semantic_annotations", DoPostSemanticAnnotations);
    ROS_INFO(
            "Ready for receiving KML: If SMM node is crushed or closed, you MUST make TRUE the variable 'once_executed' in the LAUNCH file");
    ros::ServiceServer service_get = n.advertiseService("get_semantic_map", DoGetSemanticMap);
    ROS_INFO("Ready for KML response");


    if (interface_test) {
        ros::Time time_begin = ros::Time::now();
        drone.previous_htmp_arrival_time = time_begin;
        drone2.previous_htmp_arrival_time = time_begin;
        drone3.previous_htmp_arrival_time = time_begin;

        drone.previous_cs_arrival_time = time_begin;
        drone.previous_gs_arrival_time = time_begin;
        drone.previous_dp_arrival_time = time_begin;


        drone2.previous_cs_arrival_time = time_begin;
        drone2.previous_gs_arrival_time = time_begin;
        drone2.previous_dp_arrival_time = time_begin;


        drone3.previous_cs_arrival_time = time_begin;
        drone3.previous_gs_arrival_time = time_begin;
        drone3.previous_dp_arrival_time = time_begin;
    }

    ros::MultiThreadedSpinner spinner(3); // Use 3 threads
    spinner.spin(); // spin() will not return until the node has been shutdown

    if (interface_test) {
        drone.smm_logfile.close();
        drone2.smm_logfile.close();
        drone3.smm_logfile.close();
        drone.avg_FPS.close();
        drone2.avg_FPS.close();
        drone3.avg_FPS.close();
        drone.gi_logfile.close();
        drone2.gi_logfile.close();
        drone3.gi_logfile.close();
        drone.ci_logfile.close();
        drone2.ci_logfile.close();
        drone3.ci_logfile.close();
        drone.ual_logfile.close();
        drone2.ual_logfile.close();
        drone3.ual_logfile.close();

        drone.cs_it_logfile.close();
        drone.gs_it_logfile.close();
        drone.dp_it_logfile.close();

        drone2.cs_it_logfile.close();
        drone2.gs_it_logfile.close();
        drone2.dp_it_logfile.close();

        drone3.cs_it_logfile.close();
        drone3.gs_it_logfile.close();
        drone3.dp_it_logfile.close();
    }

    drone.myTIMES2.close();
    drone2.myTIMES2.close();
    drone3.myTIMES2.close();
    drone.myTIMES3.close();
    drone2.myTIMES3.close();
    drone3.myTIMES3.close();
    return 0;

}
