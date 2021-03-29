#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

#include "apriltag_ros/AprilTagDetectionArray.h"

class PoseApriltag
{
public:
  PoseApriltag(ros::NodeHandle& nh_private) :
    target_frame_("map"),
    tf2_listener_(buffer_),
    tf2_filter_(apriltag_detect_array_sub_, buffer_, target_frame_, 10, nh_private)
  {
    ROS_INFO("PoseApriltag.");
    // ros::NodeHandle nh_private("~");
    apriltag_detect_array_sub_.subscribe(nh_private, "/tag_detections", 10);
    tf2_filter_.registerCallback(boost::bind(&PoseApriltag::apriltagPoseCb, this, _1));
    
    nav_goal_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    nh_private.param("offset_x", offset_x_, 0.9);
    nh_private.param("offset_z", offset_z_, 0.0);
    nh_private.param("offset_yaw", offset_yaw_, 0.0);
  }

  void apriltagPoseCb(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
  {
    if (!msg->detections.size())
    {
      return;
    }

    // Get mean pose in camera frame
    geometry_msgs::PoseStamped marker_pose_camera_new;
    marker_pose_camera_new.header = msg->detections[0].pose.header;
    marker_pose_camera_new.pose.position.x = msg->detections[0].pose.pose.pose.position.x;
    marker_pose_camera_new.pose.position.y = 0; //msg->detections[0].pose.pose.pose.position.y;
    marker_pose_camera_new.pose.position.z = msg->detections[0].pose.pose.pose.position.z;
    marker_pose_camera_new.pose.orientation.x = msg->detections[0].pose.pose.pose.orientation.x;
    marker_pose_camera_new.pose.orientation.y = msg->detections[0].pose.pose.pose.orientation.y;
    marker_pose_camera_new.pose.orientation.z = msg->detections[0].pose.pose.pose.orientation.z;
    marker_pose_camera_new.pose.orientation.w = msg->detections[0].pose.pose.pose.orientation.w;
      
    marker_pose_vec_.push_back(marker_pose_camera_new);
    if (marker_pose_vec_.size() == MAX_POSE_COUNT)
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

      geometry_msgs::PoseStamped marker_pose_camera_avg;
      marker_pose_camera_avg.header = marker_pose_camera_new.header;
      // marker_pose_camera_avg.header.stamp = ros::Time::now();
      marker_pose_camera_avg.pose.position.x = avg_x;
      marker_pose_camera_avg.pose.position.z = avg_z;
      marker_pose_camera_avg.pose.orientation.z = avg_q_z;
      marker_pose_camera_avg.pose.orientation.w = avg_q_w;

      ROS_INFO("marker pose camera: %.3f, %.3f", 
          marker_pose_camera_avg.pose.position.x, marker_pose_camera_avg.pose.position.z);

      if (fabs(marker_pose_camera_avg.pose.position.x - marker_pose_camera_.pose.position.x) > 0.2f || 
          fabs(marker_pose_camera_avg.pose.position.z - marker_pose_camera_.pose.position.z) > 0.2f || 
          fabs(marker_pose_camera_avg.pose.orientation.z - marker_pose_camera_.pose.orientation.z) > 0.1f ||
          fabs(marker_pose_camera_avg.pose.orientation.w - marker_pose_camera_.pose.orientation.w) > 0.1f)

      {
        ROS_INFO("Update goal...");
        geometry_msgs::TransformStamped tf;
        try 
        {
          buffer_.transform(marker_pose_camera_avg, marker_pose_map_, target_frame_);
          tf = buffer_.lookupTransform("map", "ID0", ros::Time::now(), ros::Duration(1));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("Failure %s", ex.what());
        }

        // Use yaw from tf_echo tools
        tf2::Quaternion q(
              tf.transform.rotation.x,
              tf.transform.rotation.y,
              tf.transform.rotation.z,
              tf.transform.rotation.w 
            ); 

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

        if ( fabs(nav_goal_pose.pose.position.x - nav_goal_pose_.pose.position.x) > 0.2 ||
            fabs(nav_goal_pose.pose.position.y - nav_goal_pose_.pose.position.y) > 0.2 )
        {
          nav_goal_pub_.publish(nav_goal_pose);
        }
        marker_pose_camera_ = marker_pose_camera_avg;
      }
      marker_pose_vec_.clear();
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
  ros::Publisher nav_goal_pub_;

  double offset_x_, offset_z_, offset_yaw_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vision_servoing_apriltag");
  ros::NodeHandle nh("~");
  PoseApriltag pose_apriltag(nh);

  ros::spin();

  return 0;
}
