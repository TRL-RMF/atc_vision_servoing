/*
 * AprilTagServo2.cpp
 *
 *  Created on: Apr 6, 2021
 *      Author: timityjoe
 */

#include "vision_servoing/AprilTagServo2.h"
#include "vision_servoing/vision_servoing_utils.h"

#define DEBUG_APRILTAG_SERVO 1
#define DEBUG_APRILTAG_YAW_CMD 0
#define DEBUG_APRILTAG_SPD_CMD 0

namespace vision_servoing {

AprilTagServo2::AprilTagServo2(ros::NodeHandle& nh_private, std::mutex* mtxPtr) :
    	    //target_frame_("/map"),
    		target_frame_("d435_color_optical_frame"),
    		//target_frame_("caato_0/d435_color_optical_frame"),
    	    tf2_listener_(buffer_),
    	    tf2_filter_(apriltag_detect_array_sub_, buffer_, target_frame_, 10, nh_private),
			mtx_ptr(mtxPtr),
    		stagingGoalReached(false),
			stagingGoalCalculated(false),
			dockingGoalReached(false)
{
	ROS_INFO("PoseApriltag.");
    // ros::NodeHandle nh_private("~");
    apriltag_detect_array_sub_.subscribe(nh_private, "/tag_detections", 10);
    tf2_filter_.registerCallback(boost::bind(&AprilTagServo2::aprilTagPoseCb, this, _1));

    //nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    //nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("caato_0/move_base/current_goal", 1);
    nh_private.param("offset_x", offset_x_, 3.0);
    nh_private.param("offset_z", offset_z_, 0.0);
    nh_private.param("offset_yaw", offset_yaw_, 0.0);
}

//-----------------------------------------------------------------------------
AprilTagServo2::~AprilTagServo2() {

}

//-----------------------------------------------------------------------------
void AprilTagServo2::reset()
{
	stagingGoalCalculated = false;

	stagingGoalReached = false;
	dockingGoalReached = false;

	marker_pose_vec_.clear();
}

//--------------------------------------------------------------------------------
void AprilTagServo2::aprilTagPoseCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
	mtx_ptr->lock();

#if DEBUG_APRILTAG_SERVO
	  //ROS_INFO("AprilTagServo2::aprilTagPoseCb() ");
#endif

	// * Do this function, after msg is checked to be non-zero
    //vision_servoing::debugTagMsg(msg, pixelPosRight, pixelPosDown, tagArea);

    // Mod by Tim:
	//const std::string tf_frame_mod = target_frame_;
    const std::string tf_frame_mod = "caato_0/"+target_frame_;


    if (!msg->detections.size())
    {
#if DEBUG_APRILTAG_SERVO
	  //ROS_WARN("	msg->detections.size() 0! tf_frame:%s", tf_frame_mod.c_str());
#endif

	  // Mod by Tim: reset containers
	  marker_pose_vec_.clear();
	  //marker_pose_camera_;
	  //marker_pose_map_;
	  mtx_ptr->unlock();
      return;
    }

    vision_servoing::debugTagMsg(msg, pixelPosRight, pixelPosDown, tagArea);

//    if(tagArea < 4000.0)
//    {
//#if DEBUG_APRILTAG_SERVO
//	  ROS_WARN("	Tag detected but not visibly big tagArea:%.2f", tagArea);
//#endif
//	  mtx_ptr->unlock();
//      return;
//    }

    // Get mean pose in camera frame
    geometry_msgs::PoseStamped marker_pose_camera_new;
    marker_pose_camera_new.header = msg->detections[0].pose.header;

    // Mod by Tim:
    marker_pose_camera_new.header.frame_id = tf_frame_mod;
    vision_servoing::populateMarker(msg, marker_pose_camera_new);


    marker_pose_vec_.push_back(marker_pose_camera_new);
    if (marker_pose_vec_.size() == MAX_POSE_COUNT)
    {
      //geometry_msgs::PoseStamped marker_pose_camera_avg;
      marker_pose_camera_avg.header = marker_pose_camera_new.header;
      vision_servoing::calculateAverage(MAX_POSE_COUNT, marker_pose_vec_, marker_pose_camera_avg);
      stagingGoalCalculated = true;

#if DEBUG_APRILTAG_SERVO
      ROS_INFO("	calculateAverage() done, stagingGoalCalculated...");
#endif
      // determineMotionGoal() previously done here...
    }

	mtx_ptr->unlock();
}

//--------------------------------------------------------------------------------
bool AprilTagServo2::calcMotionGoal(geometry_msgs::PoseStamped& staging_goal_pose)
{
#if DEBUG_APRILTAG_SERVO
	  //ROS_INFO("AprilTagServo2::determineMotionGoal() ");
#endif

	if (vision_servoing::isNotWithinRange(marker_pose_camera_avg, marker_pose_camera_))
	{
#if DEBUG_APRILTAG_SERVO
		ROS_INFO("1) isNotWithinRange() - Update goal...");
#endif
		  geometry_msgs::TransformStamped tf;

		  try
		  {
			// Mod by Tim:
			//buffer_.transform(marker_pose_camera_avg, marker_pose_map_, target_frame_);
			//ROS_INFO("	1:%s 2:%s ", marker_pose_camera_avg.header.frame_id.c_str() ,marker_pose_map_.header.frame_id.c_str());
			//buffer_.transform(marker_pose_camera_avg, marker_pose_map_, tf_frame_mod);
			buffer_.transform(marker_pose_camera_avg, marker_pose_map_, "map");

			// Mod by Tim:
			tf = buffer_.lookupTransform("map", "ID0", ros::Time::now(), ros::Duration(1));

			ROS_INFO("	marker_pose_map_ (map frame) x:%.3f, y:%.3f, %s",
				  marker_pose_map_.pose.position.x, marker_pose_map_.pose.position.z, marker_pose_map_.header.frame_id.c_str());
		  }
		  catch (tf2::TransformException &ex)
		  {
			ROS_WARN("Failure: %s", ex.what());
		  }

		  // Use yaw from tf_echo tools
		  tf2::Quaternion q(tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w);

		  tf2::Matrix3x3 m(q);
		  double roll, pitch, yaw;
		  m.getRPY(roll, pitch, yaw);
		  ROS_INFO("roll: %.3f, pitch: %.3f, yaw: %.3f", roll, pitch, yaw);
		  tf2::Quaternion ref_q;
		  ref_q.setRPY(0.0f, 0.0f, yaw + 1.5707f + 3.14159f);

		  // Combine the nav_goal_pose with position from marker_pose_map_,
		  // orientation from lookupTransform between map and Apriltag
		  geometry_msgs::PoseStamped nav_goal_pose;
		  nav_goal_pose = marker_pose_map_;
		  nav_goal_pose.header.frame_id = "map";
		  nav_goal_pose.header.stamp = ros::Time::now();
		  nav_goal_pose.pose.position.x -= offset_x_ * cos(yaw + 1.5707f);
		  nav_goal_pose.pose.position.y -= offset_x_ * sin(yaw + 1.5707f);
		  nav_goal_pose.pose.orientation.x = ref_q.x();
		  nav_goal_pose.pose.orientation.y = ref_q.y();
		  nav_goal_pose.pose.orientation.z = ref_q.z();
		  nav_goal_pose.pose.orientation.w = ref_q.w();

		  staging_goal_pose = nav_goal_pose;

		  // Update new goal if changed
		  if ( fabs(nav_goal_pose.pose.position.x - nav_goal_pose_.pose.position.x) > 0.2 ||
			  fabs(nav_goal_pose.pose.position.y - nav_goal_pose_.pose.position.y) > 0.2 )
		  {
#if DEBUG_APRILTAG_SERVO
	  ROS_INFO("1a) Goal changed, publishing goal pose...");
#endif
			//nav_goal_pub_.publish(nav_goal_pose);
			vision_servoing::sendGoal(nav_goal_pose, stagingGoalReached);
		  }
		  marker_pose_camera_ = marker_pose_camera_avg;

		  return true;
	}
	else
	{
		ROS_WARN("(3) determineMotionGoal() unknown! ");
		return false;
	}

	ROS_WARN("(4) determineMotionGoal() unknown! ");
	return false;
}

//--------------------------------------------------------------------------------
// Simple P controller for yaw
// Assume err = 40degs, we wish the
// turn rate to be 0.1radsec
// So P = 0.0025;
// turn rate to be 0.25radsec
// So P = 0.00625;
// turn rate to be 0.5radsec
// So P = 0.0125;
bool AprilTagServo2::calcDockingCmds(double& linearSpdCmd, double& linearYawRateCmd)
{
	if((!vision_servoing::isNotWithinRange(marker_pose_camera_avg, marker_pose_camera_)) && stagingGoalReached && (!dockingGoalReached))
	{
#if DEBUG_APRILTAG_SERVO
		//ROS_INFO("calcDockingCmds() - sending steeering cmds...");
#endif

		  const double P_linear = 0.0009;
		  const double& MAX_SPEED_METRE_SEC = 0.3;
		  bool hasReached1 = false;
		  linearSpdCmd = calculateLinearSpeedCommand(P_linear, MAX_SPEED_METRE_SEC, tagArea, hasReached1);

		  //const double P_yaw = 0.0025;
		  //const double MAX_YAW_RATE_RADSEC = 0.1;
		  //const double P_yaw = 0.00625;
		  //const double MAX_YAW_RATE_RADSEC = 0.25;
		  const double P_yaw = 0.0125;
		  const double MAX_YAW_RATE_RADSEC = 0.5;
		  bool hasReached2 = false;
		  linearYawRateCmd = calculateYawRateCommand(P_yaw, MAX_YAW_RATE_RADSEC, pixelPosRight, hasReached2);

		  dockingGoalReached = (hasReached1 && hasReached2) ? (true) : (false);

		  return true;
	}
	else
	{
		// Reset if this condition hits...
#if DEBUG_APRILTAG_SERVO
		ROS_WARN("calcDockingCmds() - unknown state! stagingGoalReached:%i, dockingGoalReached:%i ", stagingGoalReached, dockingGoalReached);
#endif
		stagingGoalReached = false;
		dockingGoalReached = false;

		return false;
	}

	return false;
}

//--------------------------------------------------------------------------------
// See vision_servoing/msg/AprilTagDetection.msg
// Tag Area, when near ~4,000, reached 166,000
double AprilTagServo2::calculateLinearSpeedCommand(const double& P_linear, const double& MAX_SPEED_METRE_SEC, const double& tagArea, bool& hasReached)
{
	//const double tag_area_setpoint = 166000;
	//const double tag_area_setpoint = 170000;
	const double tag_area_setpoint = 180000;
	const double tag_area_error = -(tag_area_setpoint - tagArea)/1000.0;	// Remove a few (1000) zeroes, for easier calculation

#if DEBUG_APRILTAG_SPD_CMD
	ROS_INFO("AprilTagServo2::calculateLinearSpeedCommand() tag_area_error:%.2f", tag_area_error);
#endif

	// Simple P controller
	// Assume err = 166K, we wish the
	// turn rate to be 1.0 meter sec
	// So P = 0.0060;
	// turn rate to be 0.6 meter sec
	// So P = 0.0036;
	// turn rate to be 0.3 meter sec
	// So P = 0.0018;
	// turn rate to be 0.15 meter sec
	// So P = 0.0009;
	double cmd_spd_meter_sec = 0;
	if(fabs(tag_area_error) > 5.0)
	{
		cmd_spd_meter_sec = P_linear * tag_area_error;
		hasReached = false;
	}
	else
	{
		cmd_spd_meter_sec = 0;
		hasReached = true;
	}

	// Limit yaw rate
	if(cmd_spd_meter_sec > MAX_SPEED_METRE_SEC)
	{
		cmd_spd_meter_sec = MAX_SPEED_METRE_SEC;
	}
	else if(cmd_spd_meter_sec < -MAX_SPEED_METRE_SEC)
	{
		cmd_spd_meter_sec = -MAX_SPEED_METRE_SEC;
	}

#if DEBUG_APRILTAG_SPD_CMD
	ROS_INFO("	cmd_spd_meter_sec:%.2f", cmd_spd_meter_sec);
#endif

	return cmd_spd_meter_sec;

}

//--------------------------------------------------------------------------------
double AprilTagServo2::calculateYawRateCommand(const double& P_yaw, const double& MAX_YAW_RATE_RADSEC, const double& pixelPosRight, bool& hasReached)
{
	// pixelPosRight 0.0 is left | 0.5 is center | 1.0 is right
	const double boreSightError = 0.5 - pixelPosRight;

	// Intel Realsense d435
	// https://ark.intel.com/content/www/us/en/ark/products/128255/intel-realsense-depth-camera-d435.html
	// Field of View  85.2deg Horiz x 58 deg Vert
	// Resolution     1280 X 720
	// If working in normalized (0.0 - 1.0), then the "gain" is simply 85.2
	const double yaw_error_degs = -boreSightError * 85.2;

#if DEBUG_APRILTAG_YAW_CMD
	ROS_INFO("AprilTagServo2::calculateYawRateCommand() yaw_error_degs:%.2f", yaw_error_degs);
#endif
	// Simple P controller
	// Assume err = 40degs, we wish the
	// turn rate to be 0.1radsec
	// So P = 0.0025;
	// turn rate to be 0.25radsec
	// So P = 0.00625;
	// turn rate to be 0.5radsec
	// So P = 0.0125;
	double cmd_yaw_rate_radsec = 0;
	if(fabs(yaw_error_degs) > 2.0)
	{
		cmd_yaw_rate_radsec = P_yaw * yaw_error_degs;
		hasReached = false;
	}
	else
	{
		cmd_yaw_rate_radsec = 0;
		hasReached = true;
	}

	// Limit yaw rate
	if(cmd_yaw_rate_radsec > MAX_YAW_RATE_RADSEC)
	{
		cmd_yaw_rate_radsec = MAX_YAW_RATE_RADSEC;
	}
	else if(cmd_yaw_rate_radsec < -MAX_YAW_RATE_RADSEC)
	{
		cmd_yaw_rate_radsec = -MAX_YAW_RATE_RADSEC;
	}

#if DEBUG_APRILTAG_YAW_CMD
	ROS_INFO("	cmd_yaw_rate_radsec:%.2f, hasReached:%i", cmd_yaw_rate_radsec, hasReached);
#endif

	return cmd_yaw_rate_radsec;
}




} /* namespace vision_servoing */
