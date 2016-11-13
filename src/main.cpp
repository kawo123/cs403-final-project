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
#include <visualization_msgs/Marker.h>

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
const float screenHeight = 1.5;

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
ros::Publisher ransacPublisher;

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

// Calculating distance between two vectors
float CalculateDistance(const Vector3f P1, Vector3f P2){
  float diffX = P1.x() - P2.x();
  float diffY = P1.y() - P2.y();
  float diffZ = P1.z() - P2.z();

  return sqrt(pow(diffX, 2) + pow(diffY, 2) + pow(diffZ, 2));
}

bool checkCylinder(const struct cylinder){//given a struct cylinder
	//for displaying cylinder on rviz, use marker
	//checks if cylinder radius is acceptable and whether or not the line corresponding with the cylinder intercepts the projector plane (rviz projector plane)
	//if yes, return true
	//if not return false

  // Vector3f p0 = cylinder.p0;
  // Vector3f l = cylinder.l;
  // float r = cylinder.r;

  // if(r > 0.4 and r<0.8){
  //   if((p0 - screenP0).dot(l) == 0){
  //     return true;
  //   }
  // }

  // return false;
}

/*bool displayCylinder(const struct cylinder){
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/arm";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = (cylinder.r) * 2; //diameter in x direction
  marker.scale.y = (cylinder.r) * 2; //diameter in y direction
  marker.scale.z = cylinder.l.norm(); //specifies height which is the length of l? (is l supposed to be vector normal to p0 which has length of l?)
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  markerPublisher.publish(marker);
}*/

bool displayScreen(const vector<struct cylinder>){
  //make a point cloud and display the screen
  //screenPublisher.publish();

  //screenpublisher needs to publish a point cloud of the screen
  vector<Vector3f> point_cloud; //y is zero while x is the width and z is the height
  const float left = -(screenWidth/2); //relative to kinect position
  const float right = screenWidth/2;
  for(size_t top = left; top<right; ++top){
    //for height = screenheight
    Vector3f point(top,0,screenHeight);
    point_cloud.push_back(point);
  }
  for(size_t bottom = left; bottom<right; ++bottom){
    //for height = 0
    Vector3f point(bottom,0,0);
    point_cloud.push_back(point);
  }
  for(size_t height = 0; height<screenHeight; ++height){
    //for left side where point is left
    Vector3f leftpoint(left,0,height);
    point_cloud.push_back(leftpoint);
    //for right side where point is right
    Vector3f rightpoint(right,0,height);
    point_cloud.push_back(rightpoint);
  }

  sensor_msgs::PointCloud point_cloud_msg;
  //point_cloud_msg.header = Header();
  point_cloud_msg.points.resize(point_cloud.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud_msg.points[i] = ConvertVectorToPoint(point_cloud[i]);
  }
  ROS_INFO("screenPublisher called");
  screenPublisher.publish(point_cloud_msg);
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

void FitMinimalPlane(const Vector3f& avg,
                     const Vector3f& P2,
                     const Vector3f& P3,
                     Vector3f* n,
                     Vector3f* P0) {
  *P0 = avg;
  const Vector3f P21 = P2 - avg;
  const Vector3f P31 = P3 - avg;
  *n = (P21.cross(P31)).normalized();
}

void FindInliers(const Vector3f& n,
                 const Vector3f& P0,
                 float radius, 
                 float epsilon,
                 const vector<Vector3f>& point_cloud,
                 vector<Vector3f>* inliers) {
  inliers->clear();
  n.normalized(); 
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    float projection = abs((point_cloud[i] - P0).dot(n)); 
    float theta = acos(projection / point_cloud[i].norm()); 
    float dist = sin(theta) * point_cloud[i].norm(); 
    // printf("In FindInliers: The distance is %f\n", dist);
    if (fabs(dist - radius) < epsilon) {
      inliers->push_back(point_cloud[i]);
    }
  }
}

void RANSAC(vector<Vector3f> point_cloud, vector<Vector3f>* filtered_point_cloud){
  // TODO: Added return value or side effect
  int point_cloud_size = point_cloud.size(); 

  Vector3f n; 
  Vector3f P0; 
  vector<Vector3f> inliers; 
  float dist_epsilon = 2; 
  float inlier_fraction = 0.0; 
  float min_inlier_fraction = 0.3; 
  
  do{
    Vector3f P1 = point_cloud[rand() % point_cloud_size];
    Vector3f P2 = point_cloud[rand() % point_cloud_size];
    Vector3f P3 = point_cloud[rand() % point_cloud_size];
    Vector3f avg = (P1 + P2 + P3) / 3; 
    float r = (CalculateDistance(P1, avg) + CalculateDistance(P2, avg) + CalculateDistance(P3, avg))/3; 
    // Vector3f random4 = point_cloud[rand() % point_cloud_size];
    // Vector3f random5 = point_cloud[rand() % point_cloud_size];
    FitMinimalPlane(avg, P2, P3, &n, &P0);
    FindInliers(n, P0, r, dist_epsilon, point_cloud, &inliers);

    inlier_fraction = static_cast<float>(inliers.size()) / static_cast<float>(point_cloud.size());
    printf("In RANSAC: the inlier_fraction is %f\n", inlier_fraction);
  }while(inlier_fraction < min_inlier_fraction);

  *filtered_point_cloud = inliers; 

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
	  
  //ransac for cylinders
  vector<Vector3f> filtered_point_cloud; 
  // GetCylinderFilteredPointCloud(depth_image, point_cloud, filtered_point_cloud);
  // RANSAC(point_cloud, &filtered_point_cloud); 
  // cout << filtered_point_cloud[0] << std::endl;

  //use checkCylinder and getBestCylinder

  // Publshing point cloud
	sensor_msgs::PointCloud point_cloud_msg;
	point_cloud_msg.header = depth_image.header;
	point_cloud_msg.points.resize(point_cloud.size());
	for (size_t i = 0; i < point_cloud.size(); ++i) {
	  point_cloud_msg.points[i] = ConvertVectorToPoint(point_cloud[i]);
	}
  PointCloudPublisher.publish(point_cloud_msg);
  ROS_INFO("DepthImageCallback called");

  // publishing filtered point cloud
  sensor_msgs::PointCloud filtered_point_cloud_msg;
  filtered_point_cloud_msg.header = depth_image.header;
  filtered_point_cloud_msg.points.resize(filtered_point_cloud.size());
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    filtered_point_cloud_msg.points[i] = ConvertVectorToPoint(filtered_point_cloud[i]);
  }
  ransacPublisher.publish(filtered_point_cloud_msg);

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_final");
  ros::NodeHandle n;

  kinectT << 0.0, 0.0, screenHeight/2;
  kinectTheta = PI/4;//45 degrees
  kinectR.row(0) << cos(kinectTheta), 0, sin(kinectTheta);//turn down about y axis
  kinectR.row(1) <<         0,        1,        0;
  kinectR.row(2) <<-sin(kinectTheta), 0, cos(kinectTheta);


  PointCloudPublisher = 
      n.advertise<sensor_msgs::PointCloud>("kinect_PointCloud", 1);
  screenPublisher = 
      n.advertise<sensor_msgs::PointCloud>("screen_PointCloud", 1);
  ransacPublisher = 
      n.advertise<sensor_msgs::PointCloud>("ransac_filtered_point_cloud", 1);

  /*markerPublisher = 
      n.advertise<sensor_msgs::PointCloud>("arm_marker", 1);*/
  /*markerPublisher = 
      n.advertise<visualization_msgs::Marker>("arm_marker", 1);*/
  

  ros::Subscriber depth_image_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);

  ros::spin();

  return 0;
}










