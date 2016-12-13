#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define PI 3.14159265 //M_PI

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point;
using geometry_msgs::Point32;
using std::fabs;
using std::max;
using std::atan2;
using std::cout;
using std::vector;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using std::cout;
using std::vector;


struct Rectangle{
  Vector3f topleft;
  Vector3f topright;
  Vector3f bottomright;
  Vector3f bottomleft;

  Rectangle(Vector3f point1, Vector3f point2, Vector3f point3, Vector3f point4){
    topleft = point1;
    topright = point2;
    bottomright = point3;
    bottomleft = point4;
  }
};


struct Line{ //this is Line struct for displaying the screen only
  Vector3f p1;
  Vector3f p2; 

  Line(Vector3f x, Vector3f y){
    p1 = x; 
    p2 = y; 
  }
}; 


struct cylinder
{
  Vector3f p0;
  Vector3f l;
  float r;
};

struct line
{
  Vector3f p0;
  Vector3f l;
};


const float p_x = 320;
const float p_y = 240;
const float f_x = 588.446;
const float f_y = -564.227;

const float a = 3.008;
const float b = -0.002745;

const float Kinect_min_range = 0.8;
const float Kinect_max_range = 10;

//projector screen plane
const Vector3f screenP0(0, 0, 0); // The screen is at the origin the kinect position should be relitive to the screen
const Vector3f screenN(1, 0, 0);
const float screenWidth = 1.5;
const float screenHeight = 1.5;

Vector3f kinectT;
float kinectTheta;
Matrix3f kinectR;

vector<Line> map; 


vector<struct line> last_found_lines;

ros::Publisher PointCloudPublisher;
ros::Publisher markersPublisher;
ros::Publisher ransacPublisher;
ros::Publisher windowPublisher;

// Markers for visualization.
Marker screen_marker;
Marker laser_marker;
Marker laser_dot_marker;

// Initialize all markers.
void InitMarkers() {
  screen_marker.header.frame_id = "kinect_0";
  screen_marker.id = 1;
  screen_marker.type = Marker::LINE_LIST;
  screen_marker.action = Marker::MODIFY;
  screen_marker.scale.x = 0.05;
  screen_marker.scale.y = 0.05;
  screen_marker.color.a = 1.0;
  screen_marker.color.r = 0.0;
  screen_marker.color.g = 1.0;
  screen_marker.color.b = 0.0;

  laser_marker.header.frame_id = "kinect_0";
  laser_marker.id = 2;
  laser_marker.type = Marker::LINE_LIST;
  laser_marker.action = Marker::MODIFY;
  laser_marker.scale.x = 0.05;
  laser_marker.scale.y = 0.05;
  laser_marker.color.a = 1.0;
  laser_marker.color.r = 1.0;
  laser_marker.color.g = 0.0;
  laser_marker.color.b = 0.0;


  laser_dot_marker.header.frame_id = "kinect_0";
  laser_dot_marker.id = 3;
  laser_dot_marker.type = Marker::POINTS;
  laser_dot_marker.action = Marker::MODIFY;
  laser_dot_marker.scale.x = 0.2;
  laser_dot_marker.scale.y = 0.2;
  laser_dot_marker.color.a = 1.0;
  laser_dot_marker.color.r = 0.0;
  laser_dot_marker.color.g = 0.0;
  laser_dot_marker.color.b = 1.0;
}

// Helper function to convert ROS Point to Eigen Vectors.
Vector3f ConvertPointToVector(const Point& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point.
Point ConvertVectorToPoint(const Vector3f& vector) {
  Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPoint32ToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
Point32 ConvertVectorToPoint32(const Vector3f& vector) {
  Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// Helper function to visualize a point.
void DrawPoint(const Vector3f& p, Marker* marker) {
  marker->points.push_back(ConvertVectorToPoint(p));
}

// Helper function to visualize an edge.
void DrawLine(const Vector3f& p1,
              const Vector3f& p2,
              Marker* marker) {
  marker->points.push_back(ConvertVectorToPoint(p1));
  marker->points.push_back(ConvertVectorToPoint(p2));
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

// checks if the given line intersects with the screen's plane
// returns true if it is valid and false otherwize
bool checkLine(line line, Rectangle screen, Vector3f *intersection){
  //followed this tutorial: http://gamedev.stackexchange.com/questions/7331/ray-plane-intersection-to-find-the-z-of-the-intersecting-point
  //uses Rectangle as two triangles
  Vector3f e1 = screen.topleft - screen.bottomleft;
  Vector3f e2 = screen.topright - screen.topleft;
  Vector3f s1 = (line.l).cross(e2);

  float divisor = s1.dot(e1);
  if(divisor == 0){
    return false; //not hit
  }
  //otherwise if divisor isn't 0, won't divide by 0
  float invDivisor = 1 / divisor;

  //compute barycentric(?) coordinate
  Vector3f d = line.p0 - screen.bottomleft;

  float b1 = d.dot(s1) * invDivisor;
  if(b1 < 0 || b1 >1){
    return false; //not hit
  }

  //compute second barycentric coordinate
  Vector3f s2 = d.cross(e1);
  float b2 = (line.l).dot(s2) * invDivisor;

  if(b2 < 0 || b1 + b2 > 1){
    return false; //not hit
  }

  //compute t to intersection distance
  float t = e2.dot(s2) * invDivisor; //Z value?
  *intersection = line.p0 + (t * line.l);
  return true;
}

//
Vector3f lineIntersectPlane(line line){
    Rectangle screen = Rectangle(Vector3f(-1.5, 0, 0), Vector3f(1.5, 0, 0), Vector3f(-1.5, 1.5, 0), Vector3f(-1.5, 0, 0));
    Vector3f intersection;
    intersection.x() = 0;   
    intersection.y() = 0;
    intersection.z() = 0;
    //return the Point
    //else returns the origin (could be Point anywhere; I just made intersection at (0,0,0) because I 
    //just want to show that the line doesn't intersect with plane)
    if(!checkLine(line, screen, &intersection)){
      ROS_INFO("Does not insect with plane");
    }
    return intersection;
}



// displays the bounding lines  of the screen on a map of Lines
void displayScreen(){
  map.push_back(Line(Vector3f(-1.5, 0, 0), Vector3f(1.5, 0, 0))); 
  map.push_back(Line(Vector3f(1.5, 0, 0), Vector3f(-1.5, 1.5, 0))); 
  map.push_back(Line(Vector3f(-1.5, 1.5, 0), Vector3f(-1.5, 0, 0))); 
  map.push_back(Line(Vector3f(-1.5, 0, 0), Vector3f(-1.5, 0, 0))); 


  for (size_t i = 0; i < map.size(); ++i) {
    DrawLine(map[i].p1, map[i].p2, &screen_marker);
  }

}

//displays the laser points and pushes to laser dot markers
void displayPoints(const vector<Vector3f> laserpointers){
  for(size_t i = 0; i<laserpointers.size(); ++i){
    DrawPoint(laserpointers[i], &laser_dot_marker);
  }
}
//displays the lasers and pushes to laser line markers
void displayLines(const vector<struct line> lines){
  for(size_t i = 0; i<lines.size(); ++i){
    DrawLine(lines[i].p0, lines[i].l, &laser_marker);
  }
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


struct line getBestFitLine(vector<Vector3f> point_cloud){
  struct line l;
  l.p0(0) = 0;
  l.p0(1) = 0;
  l.p0(2) = 0;
  for (size_t i = 0; i < point_cloud.size(); ++i){
    l.p0(0) += point_cloud[i](0);
    l.p0(1) += point_cloud[i](1);
    l.p0(2) += point_cloud[i](2);
  }
  l.p0(0) = l.p0(0)/point_cloud.size();
  l.p0(1) = l.p0(1)/point_cloud.size();
  l.p0(2) = l.p0(2)/point_cloud.size();
  return l;
}

void FindInliers(Vector3f P,
 Vector3f Q,
 float epsilon,
 const vector<Vector3f>& point_cloud,
 vector<Vector3f>* inliers) {
  inliers->clear();
  // v.normalized(); 
  Vector3f AP; 
  Vector3f PQ; 
  Vector3f A; 
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    A = point_cloud[i]; 
    AP = P - A; 
    PQ = Q - P; 
    Vector3f projection = (AP.dot(PQ) / pow(PQ.norm(), 2)) * PQ; 
    float distance = CalculateDistance(AP, projection);
    if(fabs(distance) < epsilon){
      inliers->push_back(A);
    }
  }
}

bool RANSAC(vector<Vector3f> point_cloud, vector<Vector3f>* filtered_point_cloud){
  if (point_cloud.size() == 0){
    return false;
  }

  vector<Vector3f> inliers; 
  float best_inlier_fraction = 0;
  float dist_epsilon = 0.06;
  float inlier_fraction = 0.0; 
  float min_inlier_fraction = 0.55;


  // RANSAC for line
  for(size_t i = 0; i < 20; ++i) {
    Vector3f P1 = point_cloud[rand() % point_cloud.size()];
    Vector3f P2 = point_cloud[rand() % point_cloud.size()];

    vector<Vector3f> new_inliers;
    FindInliers(P1, P2, dist_epsilon, point_cloud, &new_inliers);

    inlier_fraction = static_cast<float>(new_inliers.size()) / static_cast<float>(point_cloud.size());
    //printf("In RANSAC: the inlier_fraction is %f\n", inlier_fraction);
    if (inlier_fraction > min_inlier_fraction && inlier_fraction > best_inlier_fraction) {
      best_inlier_fraction = inlier_fraction;
      inliers = new_inliers;
    }
  }
  if (best_inlier_fraction == 0){
  	return false;
  }
  printf("In RANSAC: the best inlier_fraction is %f\n", best_inlier_fraction);
  *filtered_point_cloud = inliers;
  return true;
}

vector<Vector3f> getWindow(const vector<Vector3f> point_cloud, 
                           const Vector3f p, 
                           const float window_size){
  vector<Vector3f> window;
  const float w = window_size/2;
  const float Xmax = p.x() + w;
  const float Xmin = p.x() - w;
  const float Ymax = p.y() + w;
  const float Ymin = p.y() - w;
  const float Zmax = p.z() + w;
  const float Zmin = p.z() - w;
  for (size_t i = 0; i < point_cloud.size(); ++i){
  	if (point_cloud[i].x() > Xmin && point_cloud[i].x() < Xmax 
  		&& point_cloud[i].y() > Ymin && point_cloud[i].y() < Ymax
  		&& point_cloud[i].z() > Zmin && point_cloud[i].z() < Zmax){
     window.push_back(point_cloud[i]);
 }
}
return window;
}

void FSLF(vector<Vector3f> point_cloud, 
  unsigned int n, 
  unsigned int k,
  float window_size, 
  const vector<struct line> old_lines,    
  vector< vector<Vector3f> >* filtered_point_clouds, 
  vector<struct line>* valid_lines){ 

  if (point_cloud.size() == 0){
    return;
  }

  printf("0\n");
  vector< vector<Vector3f> > inlier_point_clouds;
  vector<struct line> lines; 
  unsigned int minWindowpoints = 100;
  float safetyDist = 2;
  printf("1\n");

  //temp
  //window_size = 0.5;
  //n = 2;
  //k = 40;

  /*for (size_t i = 0; i < last_found_lines.size(); ++i){
  	vector<Vector3f> point_cloud_window = getWindow(point_cloud, last_found_lines[i].p0, window_size);
  	ROS_INFO("starting ransac");
    vector<Vector3f> filtered_point_cloud;
    if (RANSAC(point_cloud_window, &filtered_point_cloud)){
      ROS_INFO("found a line");
      inlier_point_clouds.push_back(filtered_point_cloud);
      lines.push_back(getBestFitLine(filtered_point_cloud));
      if (inlier_point_clouds.size() > n - 1){
      	break;
      }
    }
    k--;
  }*/

  do{
  	/*vector<Vector3f> point_cloud_window;
  	unsigned int count = 0;
  	do {
     count++;
     Vector3f p = point_cloud[rand() % point_cloud.size()];
     point_cloud_window = getWindow(point_cloud, p, window_size);
     ROS_INFO("point_cloud_window.size(): %d", point_cloud_window.size());
     if (count > k){
       break;
     }
   }while(point_cloud_window.size() < minWindowpoints);*/

   Vector3f p;
   bool pIsValid = false;
   printf("2\n");
   while (!pIsValid && k > 0){
     p = point_cloud[rand() % point_cloud.size()];
     pIsValid = true;
     for(size_t i = 0; i < old_lines.size(); ++i){
       if ((old_lines[i].p0 - p).norm() < safetyDist){
         pIsValid = false;
         --k;
         break;
       }
     }
     printf("done with lines\n");
   }
   printf("3\n");
   if (k <= 0){
     break;
   }
   vector<Vector3f> point_cloud_window = getWindow(point_cloud, p, window_size);

   //Vector3f p = point_cloud[rand() % point_cloud.size()];
   //vector<Vector3f> point_cloud_window = getWindow(point_cloud, p, window_size);

   ROS_INFO("starting ransac");
   vector<Vector3f> filtered_point_cloud;
   if (RANSAC(point_cloud_window, &filtered_point_cloud)){
    ROS_INFO("found a line");
    struct line newLine = getBestFitLine(filtered_point_cloud);
    float centered_window_percent_inliers = ((float)filtered_point_cloud.size())/((float)getWindow(point_cloud, newLine.p0, window_size).size());
    ROS_INFO("centered percent inliers: %f", centered_window_percent_inliers);

    if (centered_window_percent_inliers >= 0.55){
      inlier_point_clouds.push_back(filtered_point_cloud);
      lines.push_back(newLine);
    }
    
    if (inlier_point_clouds.size() > n - 1){
     break;
   }
 }
 k--;
}while(k > 0);
*filtered_point_clouds = inlier_point_clouds; 
*valid_lines = lines;
printf("4\n");
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
 float P_range = sqrt(P.x()*P.x() + P.y()*P.y());
 if (P_range < Kinect_max_range && P_range > Kinect_min_range){
	  	//P = kinectR*P + kinectT; //rotate the kinect image
   point_cloud.push_back(P);
 }
}

  //ransac for cylinders
  // GetCylinderFilteredPointCloud(depth_image, point_cloud, filtered_point_cloud);

  //RANSAC(point_cloud, &filtered_point_cloud);

vector< vector<Vector3f> > filtered_point_clouds; 
vector<struct line> lines;

vector< vector<Vector3f> > newfiltered_point_clouds;
vector<struct line> newlines;

for (size_t i = 0; i < last_found_lines.size(); ++i){
  FSLF(getWindow(point_cloud, last_found_lines[i].p0, 1), 1, 8, 0.8, lines, &newfiltered_point_clouds, &newlines);
  for(size_t i = 0; i < newlines.size(); ++i){
    lines.push_back(newlines[i]);
  }
  for(size_t i = 0; i < newfiltered_point_clouds.size(); ++i){
    filtered_point_clouds.push_back(newfiltered_point_clouds[i]);
  }
}

FSLF(point_cloud, 1, 40, 0.8, lines, &newfiltered_point_clouds, &newlines);
for(size_t i = 0; i < newlines.size(); ++i){
  lines.push_back(newlines[i]);
}
for(size_t i = 0; i < newfiltered_point_clouds.size(); ++i){
  filtered_point_clouds.push_back(newfiltered_point_clouds[i]);
}
last_found_lines = lines;
ROS_INFO("found %d lines", lines.size());
  //vector<Vector3f> point_cloud_window = FSLF(point_cloud, &filtered_point_cloud);

  // cout << filtered_point_cloud[0] << std::endl;

  //use checkCylinder and getBestCylinder

  // Publshing point cloud
sensor_msgs::PointCloud point_cloud_msg;
point_cloud_msg.header = depth_image.header;
point_cloud_msg.points.resize(point_cloud.size());
for (size_t i = 0; i < point_cloud.size(); ++i) {
 point_cloud_msg.points[i] = ConvertVectorToPoint32(point_cloud[i]);
}
PointCloudPublisher.publish(point_cloud_msg);
ROS_INFO("DepthImageCallback called");

  // publishing filtered point cloud
sensor_msgs::PointCloud filtered_point_cloud_msg;
filtered_point_cloud_msg.header = depth_image.header;

size_t size = 0;
for (size_t i = 0; i < filtered_point_clouds.size(); ++i){
 size += filtered_point_clouds[i].size();
}

filtered_point_cloud_msg.points.resize(size);

size_t iter = 0;
for (size_t i = 0; i < filtered_point_clouds.size(); ++i) {
 for (size_t j = 0; j < filtered_point_clouds[i].size(); ++j){
   filtered_point_cloud_msg.points[iter] = ConvertVectorToPoint32(filtered_point_clouds[i][j]);
   ++iter;
 }
}
ransacPublisher.publish(filtered_point_cloud_msg);

  // Publishing window
  /*sensor_msgs::PointCloud point_cloud_window_msg;
  point_cloud_window_msg.header = depth_image.header;
  point_cloud_window_msg.points.resize(point_cloud_window.size());
  for (size_t i = 0; i < point_cloud_window.size(); ++i) {
    point_cloud_window_msg.points[i] = ConvertVectorToPoint(point_cloud_window[i]);
  }
  windowPublisher.publish(point_cloud_window_msg);*/

}




int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_final");
  ros::NodeHandle n;
  kinectT << 0.0, 0.0, screenHeight/2;
  kinectTheta = PI/4;//45 degrees
  kinectR.row(0) << cos(kinectTheta), 0, sin(kinectTheta);//turn down about y axis
  kinectR.row(1) <<         0,        1,        0;
  kinectR.row(2) <<-sin(kinectTheta), 0, cos(kinectTheta);

  MarkerArray markers;
  markers.markers.clear();
  //displayScreen()

  markers.markers.push_back(screen_marker);
  markers.markers.push_back(laser_dot_marker);
  markers.markers.push_back(laser_marker);

  markersPublisher.publish(markers);

  PointCloudPublisher = 
  n.advertise<sensor_msgs::PointCloud>("kinect_PointCloud", 1);
  ransacPublisher = 
  n.advertise<sensor_msgs::PointCloud>("ransac_filtered_point_cloud", 1);
  windowPublisher = 
  n.advertise<sensor_msgs::PointCloud>("window_point_cloud", 1);
  markersPublisher = 
  n.advertise<visualization_msgs::MarkerArray>("laser_pointer_simulation", 10);
  

  ros::Subscriber depth_image_subscriber =
  n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);

  ros::spin();

  return 0;
}










