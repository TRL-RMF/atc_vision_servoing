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

#define DEBUG_VS 1


//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PoseApriltag
{
public:
  PoseApriltag(ros::NodeHandle& nh_private) :
    //target_frame_("/map"),
	target_frame_("d435_color_optical_frame"),
	//target_frame_("caato_0/d435_color_optical_frame"),
    tf2_listener_(buffer_),
    tf2_filter_(apriltag_detect_array_sub_, buffer_, target_frame_, 10, nh_private),
	stagingGoalReached(false)
  {
    ROS_INFO("PoseApriltag.");
    // ros::NodeHandle nh_private("~");
    apriltag_detect_array_sub_.subscribe(nh_private, "/tag_detections", 10);
    tf2_filter_.registerCallback(boost::bind(&PoseApriltag::apriltagPoseCb, this, _1));
    
    //nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    //nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("caato_0/move_base/current_goal", 1);
    nh_private.param("offset_x", offset_x_, 3.0);
    nh_private.param("offset_z", offset_z_, 0.0);
    nh_private.param("offset_yaw", offset_yaw_, 0.0);
  }


//--------------------------------------------------------------------------------
  void apriltagPoseCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
  {
#if DEBUG_VS
	  //ROS_INFO("PoseApriltag::apriltagPoseCb() ");
#endif

    vision_servoing::debugTagMsg(msg, pixelPosRight, pixelPosDown, tagArea);

    // Mod by Tim:
	//const std::string tf_frame_mod = target_frame_;
    const std::string tf_frame_mod = "caato_0/"+target_frame_;


    if (!msg->detections.size())
    {
#if DEBUG_VS
	  ROS_INFO("	msg->detections.size() 0! tf_frame:%s", tf_frame_mod.c_str());
#endif

	  // Mod by Tim: reset containers
	  marker_pose_vec_.clear();
	  //marker_pose_camera_;
	  //marker_pose_map_;
      return;
    }

    // Get mean pose in camera frame
    geometry_msgs::PoseStamped marker_pose_camera_new;
    marker_pose_camera_new.header = msg->detections[0].pose.header;

    // Mod by Tim:
    marker_pose_camera_new.header.frame_id = tf_frame_mod;
    vision_servoing::populateMarker(msg, marker_pose_camera_new);


    marker_pose_vec_.push_back(marker_pose_camera_new);
    if (marker_pose_vec_.size() == MAX_POSE_COUNT)
    {
      geometry_msgs::PoseStamped marker_pose_camera_avg;
      marker_pose_camera_avg.header = marker_pose_camera_new.header;
      vision_servoing::calculateAverage(MAX_POSE_COUNT, marker_pose_vec_, marker_pose_camera_avg);


      if (vision_servoing::isNotWithinRange(marker_pose_camera_avg, marker_pose_camera_))
      {
#if DEBUG_VS
        ROS_INFO("1) Update goal...");
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

        // Update new goal if changed
        if ( fabs(nav_goal_pose.pose.position.x - nav_goal_pose_.pose.position.x) > 0.2 ||
            fabs(nav_goal_pose.pose.position.y - nav_goal_pose_.pose.position.y) > 0.2 )
        {
#if DEBUG_VS
        ROS_INFO("1a) Goal changed, publishing goal...");
#endif
          //nav_goal_pub_.publish(nav_goal_pose);
          vision_servoing::sendGoal(nav_goal_pose, stagingGoalReached);
        }
        marker_pose_camera_ = marker_pose_camera_avg;
      }
      else if((!vision_servoing::isNotWithinRange(marker_pose_camera_avg, marker_pose_camera_)) && stagingGoalReached)
      {
#if DEBUG_VS
        ROS_INFO("1b) Goal reached, sending steeering cmds...");
#endif


      }

    }
}

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


//-------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vision_servoing_2phase");
  ros::NodeHandle nh("~");
  PoseApriltag pose_apriltag(nh);

  ros::spin();

  return 0;
}
