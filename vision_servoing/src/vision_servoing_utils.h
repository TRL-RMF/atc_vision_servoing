/*
 * vision_servoing_utils.h
 *
 *  Created on: Apr 5, 2021
 *      Author: timityjoe
 */

#ifndef VISION_SERVOING_SRC_VISION_SERVOING_UTILS_H_
#define VISION_SERVOING_SRC_VISION_SERVOING_UTILS_H_


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

#include "apriltag_ros/AprilTagDetectionArray.h"



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace vision_servoing
{

void debugTagMsg(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, double& pixelPosRight, double& pixelPosDown, double& tagArea);

void populateMarker(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg, geometry_msgs::PoseStamped& marker_pose_camera_new);

void calculateAverage(const int MAX_POSE_COUNT, std::vector<geometry_msgs::PoseStamped>& marker_pose_vec_, geometry_msgs::PoseStamped& marker_pose_camera_avg);

bool isNotWithinRange(const geometry_msgs::PoseStamped& marker_pose_camera_avg, const geometry_msgs::PoseStamped& marker_pose_camera_);

bool sendGoal(const geometry_msgs::PoseStamped& nav_goal_pose, bool& stagingGoalReached);

} /* namespace vision_servoing */

#endif /* VISION_SERVOING_SRC_VISION_SERVOING_UTILS_H_ */
