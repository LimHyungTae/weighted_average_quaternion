#include "ros/ros.h"
#include "wma.hpp"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void print_quaternion(tf::Quaternion& q){
  cout<<"x y z w: "<< q.x()<<" , "<<q.y()<<" , "<<q.z()<<" , "<<q.w()<<endl;
}

int main(int argc, char **argv)
{
  tf::Quaternion q1;
  q1.setW(1);    q1.setX(0);    q1.setY(0);    q1.setZ(0);

  tf::Quaternion q2(0.33, 0.32, 0.34, 1); // x, y, z, w in order
  q2.normalize();
  print_quaternion(q2);
  tf::Quaternion q3(0.33, -1.2, 0.34, 1);
  q3.normalize();
  print_quaternion(q3);

  std::vector<tf::Quaternion> qs = {q2, q3};

  std::vector<float> weights = {2./3., 1./3.};
  auto q_wa_1_3 = calc_weighted_mean(qs, weights);
  std::vector<float> weights2 = {1./3., 2./3.};
  auto q_wa_2_3 = calc_weighted_mean(qs, weights2);

  auto q_slerp_1_3 = slerp(q2, q3, 0.3333333333);
  auto q_slerp_2_3 = slerp(q2, q3, 0.6666666666);

  auto q_lerp_1_3 = lerp(q2, q3, 0.3333333333);
  auto q_lerp_2_3 = lerp(q2, q3, 0.6666666666);
  std::cout<<"\033[1;32m";
  print_quaternion(q_slerp_1_3);
  print_quaternion(q_lerp_1_3);
  print_quaternion(q_wa_1_3);
  std::cout<<"\033[0m";

  std::cout<<"\033[1;33m";
  print_quaternion(q_slerp_2_3);
  print_quaternion(q_lerp_2_3);
  print_quaternion(q_wa_2_3);
  std::cout<<"\033[0m";


  ros::init(argc, argv, "q_interp");

  ros::NodeHandle n;

  ros::Publisher slerp_pub = n.advertise<geometry_msgs::PoseArray>("pose_slerp", 1000);
  ros::Publisher lerp_pub = n.advertise<geometry_msgs::PoseArray>("pose_lerp", 1000);
  ros::Publisher wa_pub = n.advertise<geometry_msgs::PoseArray>("pose_wa", 1000);

  ros::Publisher slerp_text_pub = n.advertise<visualization_msgs::Marker>("text/slerp", 1);
  ros::Publisher lerp_text_pub = n.advertise<visualization_msgs::Marker>("text/lerp", 1);
  ros::Publisher wa_text_pub = n.advertise<visualization_msgs::Marker>("text/wa", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // Set SLERP pose
    geometry_msgs::PoseArray msg_slerp, msg_lerp, msg_wa;
    geometry_msgs::Pose pose;
    pose.position.x = 0; pose.position.y = 0; pose.position.z = 0;
    tf::quaternionTFToMsg(q2, pose.orientation);
    msg_slerp.poses.push_back(pose);

    pose.position.x = 1; pose.position.y = 0; pose.position.z = 0;
    tf::quaternionTFToMsg(q_slerp_1_3, pose.orientation);
    msg_slerp.poses.push_back(pose);

    pose.position.x = 2; pose.position.y = 0; pose.position.z = 0;
    tf::quaternionTFToMsg(q_slerp_2_3, pose.orientation);
    msg_slerp.poses.push_back(pose);

    pose.position.x = 3; pose.position.y = 0; pose.position.z = 0;
    tf::quaternionTFToMsg(q3, pose.orientation);
    msg_slerp.poses.push_back(pose);

    msg_slerp.header.frame_id = "/map";
    msg_slerp.header.stamp = ros::Time::now();

    // LERP
    pose.position.x = 0; pose.position.y = 2; pose.position.z = 0;
    tf::quaternionTFToMsg(q2, pose.orientation);
    msg_lerp.poses.push_back(pose);

    pose.position.x = 1; pose.position.y = 2; pose.position.z = 0;
    tf::quaternionTFToMsg(q_lerp_1_3, pose.orientation);
    msg_lerp.poses.push_back(pose);

    pose.position.x = 2; pose.position.y = 2; pose.position.z = 0;
    tf::quaternionTFToMsg(q_lerp_2_3, pose.orientation);
    msg_lerp.poses.push_back(pose);

    pose.position.x = 3; pose.position.y = 2; pose.position.z = 0;
    tf::quaternionTFToMsg(q3, pose.orientation);
    msg_lerp.poses.push_back(pose);

    msg_lerp.header.frame_id = "/map";
    msg_lerp.header.stamp = ros::Time::now();

    // Weighted average
    pose.position.x = 0; pose.position.y = 4; pose.position.z = 0;
    tf::quaternionTFToMsg(q2, pose.orientation);
    msg_wa.poses.push_back(pose);

    pose.position.x = 1; pose.position.y = 4; pose.position.z = 0;
    tf::quaternionTFToMsg(q_wa_1_3, pose.orientation);
    msg_wa.poses.push_back(pose);

    pose.position.x = 2; pose.position.y = 4; pose.position.z = 0;
    tf::quaternionTFToMsg(q_wa_2_3, pose.orientation);
    msg_wa.poses.push_back(pose);

    pose.position.x = 3; pose.position.y = 4; pose.position.z = 0;
    tf::quaternionTFToMsg(q3, pose.orientation);
    msg_wa.poses.push_back(pose);

    msg_wa.header.frame_id = "/map";
    msg_wa.header.stamp = ros::Time::now();

    // Publish
    slerp_pub.publish(msg_slerp);
    lerp_pub.publish(msg_lerp);
    wa_pub.publish(msg_wa);

    // viz
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = -2;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.text = "slerp";
    slerp_text_pub.publish(marker);

    marker.pose.position.y = 2;
    marker.text = "lerp";
    lerp_text_pub.publish(marker);

    marker.pose.position.y = 4;
    marker.text = "wa";
    wa_text_pub.publish(marker);

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
