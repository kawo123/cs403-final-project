#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265 //M_PI

using std::max;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using std::cout;
using std::vector;

const float p_x = 320;
const float p_y = 240;
const float f_x = 588.446;
const float f_y = -564.227;

const float a = 3.008;
const float b = -0.002745;

//projector screen plane
const Vector3f screenP0(0, 0, 0); // The screen is at the origin the kinect position should be relitive to the screen
const Vector3f screenN(1, 0, 0);
const float screenWidth = 1.5;
const float screenHight = 1.5;

Vector3f kinectT;
float kinectTheta;
Matrix3f kinectR;

struct cylinder
{
  Vector3f p0;
  Vector3f l;
  float r;
};

ros::Publisher PointCloudPublisher;
ros::Publisher screenPublisher;
ros::Publisher markerPublisher;

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
Point32 ConvertVectorToPoint(const Vector3f& vector) {
  Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// Helper function to find the magnitude of Vector2f
float FindVectorMaginitude(const Vector2f V){
  return (sqrt(pow(V.x(), 2)+pow(V.y(), 2))); 
}

// Helper function to find the magnitude of x and y
float FindVectorMaginitude(const float x, const float y){
  return (sqrt(pow(x, 2)+pow(y, 2))); 
}

bool checkCylinder(const struct cylinder){//given a struct cylinder
	//for displaying cylinder on rviz, use marker
	//checks if cylinder radius is acceptable and whether or not the line corresponding with the cylinder intercepts the projector plane (rviz projector plane)
	//if yes, return true
	//if not return false

  // Vector3f p0 = req.cylinder.p0;
  // Vector3f l = req.cylinder.l;
  // float r = req.cylinder.r;

  // if(r > 0.4 and r<0.8){
  //   if((p0 - screenP0).dot(l) == 0){
  //     return true;
  //   }
  // }

  // return false;
}

bool displayScreen(const vector<struct cylinder>){
  //make a point cloud and display the screen
  //screenPublisher.publish();
  return true;
}

struct cylinder getBestCylinder(vector<Vector3f> filteredPointCloud){
  struct cylinder c;
  return c;
}

bool GetCylinderFilteredPointCloud(const sensor_msgs::Image& depth_image, 
                                    vector<Vector3f> point_cloud, 
                                    vector<Vector3f> filtered_point_cloud){
  // user variables
  int max_neighborhoods = 0;
  int neightborhood_size = 100; 
  int deviation = neightborhood_size / 2; 
  int num_local_sample = 20; 
  int point_cloud_size = point_cloud.size(); 

  // parameters
  int num_point_per_row = depth_image.width / 10; 

  // Fast plane filtering for cylinders
  int i = 0; 
  int neighborhood_counter = 0; 
  while(i < point_cloud_size && neighborhood_counter < max_neighborhoods){
    int D0 = rand() % point_cloud_size; 
    int D1 = D0 + (rand() % neightborhood_size + 1) - deviation;
    // abs(D1 - D0): distance from D0 to D1
    // abs((D1 / num_point_per_row) - (D0 / num_point_per_row)): distance from row of D0 to row of D1
    while( abs(D1 - D0) > deviation || abs((D1 / num_point_per_row) - (D0 / num_point_per_row)) > deviation){
      D1 = D0 + (rand() % neightborhood_size + 1) - deviation;
    } 
    int D2 = D0 + (rand() % neightborhood_size + 1) - deviation;
    while( abs(D2 - D0) > deviation || abs((D2 / num_point_per_row) - (D0 / num_point_per_row)) > deviation){
      D2 = D0 + (rand() % neightborhood_size + 1) - deviation;
    } 
    
    Vector3f randomD0 = point_cloud[D0];
    Vector3f randomD1 = point_cloud[D1];
    Vector3f randomD2 = point_cloud[D2];

    // TODO: Finish Fast Sampling plane Filtering

    // cout<<"The size of point cloud is " << point_cloud_size <<endl;

    i++; 
    neighborhood_counter++; 
  }

  return true; 
}

void FitMinimalCylindericalModel(const Vector3f& P1,
                         const Vector3f& P2,
                         const Vector3f& P3,
                         const Vector3f& P4,
                         const Vector3f& P5
                         ){

}

void RANSAC(vector<Vector3f> point_cloud){
  // TODO: Added return value or side effect
  int point_cloud_size = point_cloud.size(); 

  // Vector3f random1 = point_cloud[rand() % point_cloud_size];
  // Vector3f random2 = point_cloud[rand() % point_cloud_size];
  // Vector3f random3 = point_cloud[rand() % point_cloud_size];
  // Vector3f random4 = point_cloud[rand() % point_cloud_size];
  // Vector3f random5 = point_cloud[rand() % point_cloud_size];

}

void DepthImageCallback(const sensor_msgs::Image& depth_image){
	vector<Vector3f> temp_point_cloud;
	int count = 10;
  
  // Setting random seed
  srand(time(NULL)); 

  	for (unsigned int y = 0; y < depth_image.height; ++y) {
    for (unsigned int x = 0; x < depth_image.width; ++x) {
      // Add code here to only process only every nth pixel
      if(count <= 0){
        uint16_t byte0 = depth_image.data[2 * (x + y * depth_image.width) + 0];
        uint16_t byte1 = depth_image.data[2 * (x + y * depth_image.width) + 1];
        if (!depth_image.is_bigendian) {
          std::swap(byte0, byte1);
        }
        // Combine the two bytes to form a 16 bit value, and disregard the
        // most significant 4 bits to extract the lowest 12 bits.
        const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
        // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.
       float depth = 1/ (a + (b*raw_depth));

        Vector3f point(depth * ((x - p_x)/f_x),  depth * ((y - p_y)/f_y), depth);
        temp_point_cloud.push_back(point);
        count = 10;
    	} else {
      		count--;
    	}
   	} 
	}


	vector<Vector3f> point_cloud;
	for (size_t i = 0; i < temp_point_cloud.size(); ++i){
	  Vector3f P(temp_point_cloud[i].z(), -temp_point_cloud[i].x(), temp_point_cloud[i].y());
    //P = kinectR*P + kinectT; //rotate the kinect inage
	  point_cloud.push_back(P);
  }
	  

	sensor_msgs::PointCloud point_cloud_msg;
	point_cloud_msg.header = depth_image.header;
	point_cloud_msg.points.resize(point_cloud.size());
	for (size_t i = 0; i < point_cloud.size(); ++i) {
	  point_cloud_msg.points[i] = ConvertVectorToPoint(point_cloud[i]);
	}

  PointCloudPublisher.publish(point_cloud_msg);
  ROS_INFO("DepthImageCallback called");

  //ransac for cylinders
  vector<Vector3f> filtered_point_cloud; 
  GetCylinderFilteredPointCloud(depth_image, point_cloud, filtered_point_cloud);

  //use checkCylinder and getBestCylinder

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_final");
  ros::NodeHandle n;

  kinectT << 0.0, 0.0, screenHight/2;
  kinectTheta = PI/4;//45 degrees
  kinectR.row(0) << cos(kinectTheta), 0, sin(kinectTheta);//turn down about y axis
  kinectR.row(1) <<         0,        1,        0;
  kinectR.row(2) <<-sin(kinectTheta), 0, cos(kinectTheta);


  PointCloudPublisher = 
      n.advertise<sensor_msgs::PointCloud>("kinect_PointCloud", 1);
  screenPublisher = 
      n.advertise<sensor_msgs::PointCloud>("screen_PointCloud", 1);
  /*markerPublisher = 
      n.advertise<sensor_msgs::PointCloud>("arm_marker", 1);*/

  ros::Subscriber depth_image_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);

  ros::spin();

  return 0;
}










