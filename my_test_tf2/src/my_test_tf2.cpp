#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include "tf2/LinearMath/Vector3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <math.h>

void transformWithObjectTF(tf2::Transform &transfrm){
  //define some points
  tf2::Vector3 point1_unrotated(0.5,0,0.5);
  tf2::Vector3 point2_unrotated(-0.5,0,0.5);
  tf2::Vector3 point3_unrotated(-0.5,0,-0.5);
  tf2::Vector3 point4_unrotated(0.5,0,-0.5);
 
  //rotate this points with this transformation matrix
  tf2::Vector3 point1_rotated = transfrm * point1_unrotated;
  tf2::Vector3 point2_rotated = transfrm * point2_unrotated;
  tf2::Vector3 point3_rotated = transfrm * point3_unrotated;
  tf2::Vector3 point4_rotated = transfrm * point4_unrotated;

  //display points after rotation
  ROS_INFO_STREAM("[ c++ ]");
  ROS_INFO_STREAM("p1 [ x:" << point1_rotated.x() << " y:" << point1_rotated.y() << " z:" << point1_rotated.z() << " ]");
  ROS_INFO_STREAM("p2 [ x:" << point2_rotated.x() << " y:" << point2_rotated.y() << " z:" << point2_rotated.z() << " ]");
  ROS_INFO_STREAM("p3 [ x:" << point3_rotated.x() << " y:" << point3_rotated.y() << " z:" << point3_rotated.z() << " ]");
  ROS_INFO_STREAM("p4 [ x:" << point4_rotated.x() << " y:" << point4_rotated.y() << " z:" << point4_rotated.z() << " ]");
  
  //moveit cartesian path constrains needs geometry_msgs::Pose so convert to geometry_msgs
  geometry_msgs::Vector3 point1_rotated_msg = tf2::toMsg(point1_rotated);
  geometry_msgs::Vector3 point2_rotated_msg = tf2::toMsg(point2_rotated);
  geometry_msgs::Vector3 point3_rotated_msg = tf2::toMsg(point3_rotated);
  geometry_msgs::Vector3 point4_rotated_msg = tf2::toMsg(point4_rotated);

  //display points after rotation
  ROS_INFO_STREAM("[ converted to geometry_msgs ]");
  ROS_INFO_STREAM("p1 [ x:" << point1_rotated_msg.x << " y:" << point1_rotated_msg.y << " z:" << point1_rotated_msg.z << " ]");
  ROS_INFO_STREAM("p2 [ x:" << point2_rotated_msg.x << " y:" << point2_rotated_msg.y << " z:" << point2_rotated_msg.z << " ]");
  ROS_INFO_STREAM("p3 [ x:" << point3_rotated_msg.x << " y:" << point3_rotated_msg.y << " z:" << point3_rotated_msg.z << " ]");
  ROS_INFO_STREAM("p4 [ x:" << point4_rotated_msg.x << " y:" << point4_rotated_msg.y << " z:" << point4_rotated_msg.z << " ]");
}

void transformWithMSGTF(tf2::Transform &transfrm){
  //the same but directly working with msgs

  //transform tf2::Transform to geometry_msg
  geometry_msgs::TransformStamped transfrmStamped_msg;
  tf2::Stamped<tf2::Transform> transfrmStamped(transfrm, ros::Time(), "my_frame");
  tf2::convert(transfrmStamped, transfrmStamped_msg);
  
  //define some points
  geometry_msgs::Point point1_unrotated_msg;
  point1_unrotated_msg.x = 0.5;
  point1_unrotated_msg.y = 0;
  point1_unrotated_msg.z = 0.5;

  geometry_msgs::Point point2_unrotated_msg;
  point2_unrotated_msg.x = -0.5;
  point2_unrotated_msg.y = 0;
  point2_unrotated_msg.z = 0.5;

  geometry_msgs::Point point3_unrotated_msg;
  point3_unrotated_msg.x = -0.5;
  point3_unrotated_msg.y = 0;
  point3_unrotated_msg.z = -0.5;

  geometry_msgs::Point point4_unrotated_msg;
  point4_unrotated_msg.x = 0.5;
  point4_unrotated_msg.y = 0;
  point4_unrotated_msg.z = -0.5;

  //output variables
  geometry_msgs::Point point1_rotated_msg;
  geometry_msgs::Point point2_rotated_msg;
  geometry_msgs::Point point3_rotated_msg;
  geometry_msgs::Point point4_rotated_msg;

  //transform
  tf2::doTransform(point1_unrotated_msg, point1_rotated_msg, transfrmStamped_msg);
  tf2::doTransform(point2_unrotated_msg, point2_rotated_msg, transfrmStamped_msg);
  tf2::doTransform(point3_unrotated_msg, point3_rotated_msg, transfrmStamped_msg);
  tf2::doTransform(point4_unrotated_msg, point4_rotated_msg, transfrmStamped_msg);

  //display points after rotation
  ROS_INFO_STREAM("[ direct geometry_msgs transform ]");
  ROS_INFO_STREAM("p1 [ x:" << point1_rotated_msg.x << " y:" << point1_rotated_msg.y << " z:" << point1_rotated_msg.z << " ]");
  ROS_INFO_STREAM("p2 [ x:" << point2_rotated_msg.x << " y:" << point2_rotated_msg.y << " z:" << point2_rotated_msg.z << " ]");
  ROS_INFO_STREAM("p3 [ x:" << point3_rotated_msg.x << " y:" << point3_rotated_msg.y << " z:" << point3_rotated_msg.z << " ]");
  ROS_INFO_STREAM("p4 [ x:" << point4_rotated_msg.x << " y:" << point4_rotated_msg.y << " z:" << point4_rotated_msg.z << " ]");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;
  
  //define transformation matrix of quaternion and a vector
  tf2::Transform transfrm(tf2::Quaternion(tf2::Vector3(0,1,0), M_PI/4.0), tf2::Vector3(0,0,0));
  
  //do the tests
  transformWithObjectTF(transfrm);
  transformWithMSGTF(transfrm);
 
  return 0;
};
