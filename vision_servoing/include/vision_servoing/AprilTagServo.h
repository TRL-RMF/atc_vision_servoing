/*
 * AprilTagServo.h
 *
 *  Created on: Apr 6, 2021
 *      Author: timityjoe
 */

#ifndef VISION_SERVOING_SRC_APRILTAGSERVO_H_
#define VISION_SERVOING_SRC_APRILTAGSERVO_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "apriltag_ros/AprilTagDetectionArray.h"

#include "vision_servoing_utils.h"



namespace vision_servoing {

class AprilTagServo {
public:
	AprilTagServo(ros::NodeHandle& nh_private);
	virtual ~AprilTagServo();

	double calculateLinearSpeedCommand(const double& P_linear, const double& MAX_SPEED_METRE_SEC, const double& tagArea, bool& hasReached);

	double calculateYawRateCommand(const double& P_yaw, const double& MAX_YAW_RATE_RADSEC, const double& pixelPosRight, bool& hasReached);

	void aprilTagPoseCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

private:
	  std::string target_frame_;
	  tf2_ros::Buffer buffer_;
	  tf2_ros::TransformListener tf2_listener_;
	  message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> apriltag_detect_array_sub_;
	  tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray> tf2_filter_;

	  static constexpr int MAX_POSE_COUNT = 20;
	  geometry_msgs::PoseStamped marker_pose_camera_, marker_pose_map_;
	  std::vector<geometry_msgs::PoseStamped> marker_pose_vec_;
	  geometry_msgs::PoseStamped nav_goal_pose_;
	  //ros::Publisher nav_goal_pub_;

	  double offset_x_, offset_z_, offset_yaw_;

	  // Mod by Tim:
	  double pixelPosRight;
	  double pixelPosDown;
	  double tagArea;
	  bool stagingGoalReached;
};

} /* namespace vision_servoing */

#endif /* VISION_SERVOING_SRC_APRILTAGSERVO_H_ */
