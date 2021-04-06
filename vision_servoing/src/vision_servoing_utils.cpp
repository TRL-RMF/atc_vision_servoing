/*
 * vision_servoing_utils.cpp
 *
 *  Created on: Apr 5, 2021
 *      Author: timityjoe
 */

#include "vision_servoing_utils.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


#define DEBUG_VS_UTILS 0
#define DEBUG_SEND_GOAL 1


namespace vision_servoing {

//--------------------------------------------------------------------------------
void debugTagMsg(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, double& pixelPosRight, double& pixelPosDown, double& tagArea)
{
#if 1
    ROS_INFO("	debugTagSize() tag id:%i, size:%.2f, pxRight:%.2f, pxDown:%.2f, tagArea:%.2f",
    		msg->detections[0].id[0], msg->detections[0].size[0],
			msg->detections[0].pixelPosRight, msg->detections[0].pixelPosDown,
			msg->detections[0].tagArea);
#endif
}

//--------------------------------------------------------------------------------
void populateMarker(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, geometry_msgs::PoseStamped& marker_pose_camera_new)
{
    marker_pose_camera_new.pose.position.x = msg->detections[0].pose.pose.pose.position.x;
    marker_pose_camera_new.pose.position.y = 0; //msg->detections[0].pose.pose.pose.position.y;
    marker_pose_camera_new.pose.position.z = msg->detections[0].pose.pose.pose.position.z;
    marker_pose_camera_new.pose.orientation.x = msg->detections[0].pose.pose.pose.orientation.x;
    marker_pose_camera_new.pose.orientation.y = msg->detections[0].pose.pose.pose.orientation.y;
    marker_pose_camera_new.pose.orientation.z = msg->detections[0].pose.pose.pose.orientation.z;
    marker_pose_camera_new.pose.orientation.w = msg->detections[0].pose.pose.pose.orientation.w;

#if DEBUG_VS_UTILS
    ROS_INFO("marker pose x:%.3f, y:%.3f",
    		marker_pose_camera_new.pose.position.x, marker_pose_camera_new.pose.position.z);
#endif

}

//--------------------------------------------------------------------------------
void calculateAverage(const int MAX_POSE_COUNT, std::vector<geometry_msgs::PoseStamped>& marker_pose_vec_, geometry_msgs::PoseStamped& marker_pose_camera_avg)
{
    double sum_x = 0.0f;
    double sum_z = 0.0f;
    double sum_q_z = 0.0f;
    double sum_q_w = 0.0f;
    for (const auto pose : marker_pose_vec_)
    {
      sum_x += pose.pose.position.x;
      sum_z += pose.pose.position.z;
      sum_q_z += pose.pose.orientation.z;
      sum_q_w += pose.pose.orientation.w;
    }

    double avg_x = sum_x / MAX_POSE_COUNT;
    double avg_z = sum_z / MAX_POSE_COUNT;
    double avg_q_z = sum_q_z / MAX_POSE_COUNT;
    double avg_q_w = sum_q_w / MAX_POSE_COUNT;

    //geometry_msgs::PoseStamped marker_pose_camera_avg;
    //marker_pose_camera_avg.header = marker_pose_camera_new.header;

    // Mod by Tim:
    //marker_pose_camera_avg.header.frame_id = tf_frame_mod;

    // marker_pose_camera_avg.header.stamp = ros::Time::now();
    marker_pose_camera_avg.pose.position.x = avg_x;
    marker_pose_camera_avg.pose.position.z = avg_z;
    marker_pose_camera_avg.pose.orientation.z = avg_q_z;
    marker_pose_camera_avg.pose.orientation.w = avg_q_w;

#if DEBUG_VS_UTILS
    ROS_INFO("	marker pose (camera frame) x:%.3f, y:%.3f, %s",
        marker_pose_camera_avg.pose.position.x, marker_pose_camera_avg.pose.position.z, marker_pose_camera_avg.header.frame_id.c_str());
#endif
}

//--------------------------------------------------------------------------------
bool isNotWithinRange(const geometry_msgs::PoseStamped& marker_pose_camera_avg, const geometry_msgs::PoseStamped& marker_pose_camera_)
{
	return(	fabs(marker_pose_camera_avg.pose.position.x - marker_pose_camera_.pose.position.x) > 0.2f ||
	          fabs(marker_pose_camera_avg.pose.position.z - marker_pose_camera_.pose.position.z) > 0.2f ||
	          fabs(marker_pose_camera_avg.pose.orientation.z - marker_pose_camera_.pose.orientation.z) > 0.1f ||
	          fabs(marker_pose_camera_avg.pose.orientation.w - marker_pose_camera_.pose.orientation.w) > 0.1f);
}

//--------------------------------------------------------------------------------
bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& stagingGoalReached)
{
	MoveBaseClient ac("caato_0/move_base", true);

    ac.cancelAllGoals();
    move_base_msgs::MoveBaseGoal goal;
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up...");
    }

#if DEBUG_SEND_GOAL
	ROS_INFO("	sendGoal() - [move_base] action server up... ");
#endif


	//goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.frame_id = nav_goal_pose.header.frame_id;
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose = nav_goal_pose;

	// Mod by Tim: Test. MoveBase aborts because goal plan has no solution...
	//goal.target_pose.pose.position.x = 12.1;
	//goal.target_pose.pose.position.y = -25.2;

	ROS_INFO(" 	goal pose(x= %.2f y=%.2f) frame_id:%s",
			goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.header.frame_id.c_str());
	ac.sendGoal(goal);
	ac.waitForResult();
	std::string flag_success = "x";

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("	Goal success");
		flag_success = "true";
		//msg_debug.data = req.wp_name +","+"success";

		stagingGoalReached = true;
		return true;
	}
	else
	{
		ROS_INFO("	Failed to achieved goal:%s ",  ac.getState().toString().c_str() );
		flag_success = "false";
		//msg_debug.data = req.wp_name +","+"fail";
		return false;
	}

	ROS_INFO("	sendGoal() - UNKNOWN");
	return false;

}



} /* namespace vision_servoing */
