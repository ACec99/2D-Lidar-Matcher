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

std::unique_ptr<ICP> ICPLaser;

tf2_ros::Buffer listener;

int num_groups = 0;

bool Fixed_or_Moving = false;

const Eigen::Isometry2f getTransform(const std::string& from, const std::string& to) {
	//this will give us the coordinate of the child frame in the parent frame
	Eigen::Isometry2f Transformation = Eigen::Isometry2f::Identity();
	geometry_msgs::TransformStamped transformStamped;
	try {
		transformStamped = listener.lookupTransform(from, to, ros::Time(0)); //this operation "look up" the updated transform
		tf2::Quaternion q;
		tf2::convert(transformStamped.transform.rotation , q);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		auto tr = transformStamped.transform.translation;
		//take the translation and rotation 
		Transformation.linear()=Rtheta(yaw); 
		Transformation.translation()=Vector2f(tr.x, tr.y);
	}
	catch ( tf::TransformException& e ) {
		std::cout << e.what();
		//ros::Duration(1.0).sleep();
		std::cerr << "cannot transform correctly" << endl;
	}
	cerr << from << "->" << to << endl;
	cerr << Transformation.matrix() << endl;
	return Transformation;
}
	


void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
	//for a sensor_msgs::LaserScan we have:
	//float32 angle_min = start angle of the scan [rad]
	//float32 angle_max = end angle of the scan [rad]
	//float32 angle_increment = angular distance between measurements [rad] = angular distance between consecutive different value of ranges ( ex. angular distance between ranges[i] and ranges[i+1])
	float angle_min = scan_in->angle_min;
	float angle_max = scan_in->angle_max;
	float angle_increment = scan_in->angle_increment;
	float size = std::ceil((angle_max-angle_min)/angle_increment); //in this way I have calculated the value of the angular distance of each sample
	cerr << "this is the current num_groups" << num_groups << endl;
	if (num_groups == 0) {
		Eigen::Isometry2f MTB=getTransform("map","base_link");
		Eigen::Isometry2f BTL=getTransform("base_link","base_laser_link");
		Eigen::Isometry2f MTL = MTB*BTL;
		ICPLaser = std::unique_ptr<ICP>(new ICP(BTL,MTL,20,size));
		Fixed_or_Moving = true;
	}
	else if ( num_groups == 1 ) {
		Fixed_or_Moving = false;
	}
	else {
		ICPLaser->updateOld();
		Fixed_or_Moving = false;
	}
	int angle = angle_min;
	int idx=0;
	for (int i=0; i<size; i++) {
		float value = scan_in->ranges[i];
		angle += angle_increment;
		float val_x = value*cos(angle);
		float val_y = value*sin(angle);
		//cerr << "Sono VAL_X" << val_x << endl;
		//cerr << "Sono VAL_Y" << val_y << endl;
		ICPLaser->ValuesInsertion(Fixed_or_Moving,idx,Eigen::Vector2f(val_x,val_y));
		idx ++;
	}
	num_groups += 1;
	if ( num_groups == 1 ) { //this because if we still have only one group (a moving group of points) we cannot make nothing. 
		return;
	}
	cerr << "start the running of the ICP" << endl;
	ICPLaser->run(5); //here run the ICP
	cerr << "the ICP is done" << endl;
	ICPLaser->updateMTL(); //here we update the isometry 
	Eigen::Isometry2f MTB = ICPLaser->MTB();
	cerr << "Matrix MTB (map to base_link) computed" << endl;
	cerr << MTB.matrix() << endl;
	
	//now we have to acquire the pose and pusblish it
	geometry_msgs::Pose2D::Ptr vel_pose;
	vel_pose = boost::make_shared<geometry_msgs::Pose2D>();
	//With boost::make_shared() you can create a smart pointer of type boost::shared_ptr 
	//without having to calling the constructor of boost::shared_ptr yourself.
	//The advantage of boost::make_shared() is that the memory for the object that has to 
	//be allocated dynamically and the memory for the reference counter used by the smart 
	//pointer internally can be reserved in one chunk. 
	//Using boost::make_shared() is more efficient than calling new to create a dynamically 
	//allocated object and calling new again in the constructor of boost::shared_ptr to allocate memory for the reference counter.

	vel_pose->x = MTB.translation()(0);
	vel_pose->y = MTB.translation()(1);
	vel_pose->theta = Eigen::Rotation2Df(MTB.rotation()).angle();
	vel_pub.publish(vel_pose); //here we publish the pose2D
	
	//now we want to be able to show/read the different transformations (in relation to the changing in position) of the robot => 
	//we can do this in the odom frame (=through the odometry system). So, we have to make a transformation from /map to /odom

	Eigen::Isometry2f OTB = getTransform("odom","base_link");
	Eigen::Isometry2f BTO = OTB.inverse();
	Eigen::Isometry2f MTO = MTB*BTO;
	
	cerr << "Matrix MTO (= map to odom) computed" << endl;
	cerr<< MTO.matrix()<<endl;
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped ts;
	
	ts.header.stamp = ros::Time::now();
	ts.header.frame_id = "/map";
	ts.child_frame_id = "/odom";
	//here (in these two following assignements) we have to put the forward transform between /map and /odom
	ts.transform.translation.x = MTO.translation()(0);
	ts.transform.translation.y = MTO.translation()(1);
	//
	ts.transform.translation.z = 0.0;
	tf2::Quaternion q;
	q.setRPY(0, 0, Eigen::Rotation2Df(MTO.rotation()).angle());
	ts.transform.rotation.x = q.x();
	ts.transform.rotation.y = q.y();
	ts.transform.rotation.z = q.z();
	ts.transform.rotation.w = q.w(); 
	br.sendTransform(ts); 
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "lidar_matcher");
	
	ros::NodeHandle n; //initialization of a node
	
	vel_pub = n.advertise<geometry_msgs::Pose2D>("/pose2D", 1000); //I write on the /pose2D topic the coordinates of the position of the robot 
	
	tf2_ros::TransformListener tfListene(listener);
	
	//ROS_INFO("Sono prima del vel_sub");
	ros::Subscriber vel_sub = n.subscribe("/base_scan",1000,LaserCallBack); //I read the lasercan and update the Pose2D
	//ROS_INFO("Sono dopo il vel_sub");
	
	ros::spin(); //if you are subscribing to messages, services or actions, you must call spin to process the event
	
	return 0;
}


