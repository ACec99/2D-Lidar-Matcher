#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include "tf/tf.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher vel_pub ;

ros::Subscriber vel_sub;

const Eigen::Isometry2f getTransform(const std::string& from, const std::string& to) {
	
  Eigen::Isometry2f MTB = Eigen::Isometry2f::Identity();
  
  if(tfBuffer.canTransform(from, to, ros::Time(0))){
    geometry_msgs::TransformStamped transformStamped=tfBuffer.lookupTransform(from, to, ros::Time(0));
    tf2::Quaternion q;
    tf2::convert(transformStamped.transform.rotation , q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    auto tr = transformStamped.transform.translation;
    //take the translation and rotation 
    MTB.linear()=Rtheta(yaw); 
    MTB.translation()=Vector2f(tr.x, tr.y);
  }
  else{
    std::cerr << "cannot transform correctly" << endl;
  }
  cerr << from << "->" << to << endl;
  cerr << MTB.matrix() << endl;
  return MTB;
}

void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in) {}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "2D_Lidar_Matcher");
	
	ros::NodeHandle n; //initialization of a node
	
	vel_sub = n.subscribe("/base_scan",1000,LaserCallBack); //I read the lasercan and update the Pose2D
																					
	vel_pub = n.advertise<geometry_msgs::Pose2D>("/pose2D", 1000); //I write on the /pose2D topic the correct coordinates 
	
	ros::spin(); //if you are subscribing to messages, services or actions, you must call spin to process the event
	
	return 0;
}

