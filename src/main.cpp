#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

bool checkCylinder(){//given 4 points
	//for displaying cylinder on rviz, use marker
	//checks if cylinder radius is acceptable and whether or not the line corresponding with the cylinder intercepts the projector plane (rviz projector plane)
	//if yes, return true
	//if not return false

}

void ImageToPointCloud(const sensor_msgs::Image& Image){
	vector<Vector3f> temp_point_cloud;
	int count = 10;
  	for (unsigned int y = 0; y < Image.height; ++y) {
    for (unsigned int x = 0; x < Image.width; ++x) {
      // Add code here to only process only every nth pixel
      if(count <= 0){
        uint16_t byte0 = Image.data[2 * (x + y * Image.width) + 0];
        uint16_t byte1 = Image.data[2 * (x + y * Image.width) + 1];
        if (!Image.is_bigendian) {
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
	    point_cloud.push_back(P);
	  

	  sensor_msgs::PointCloud point_cloud_msg;
	  point_cloud_msg.header = Image.header;
	  point_cloud_msg.points.resize(point_cloud.size());
	  for (size_t i = 0; i < point_cloud.size(); ++i) {
	    point_cloud_msg.points[i] = ConvertVectorToPoint(point_cloud[i]);
	  }
	}
}

void bestCylinder(){
	//finds best fit cylinder for filtered point cloud

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_final");
  ros::NodeHandle n;



  ros::spin();

  return 0;
}










