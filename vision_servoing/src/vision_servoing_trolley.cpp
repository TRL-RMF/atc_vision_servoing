#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

class PoseTrolley
{
public:
  PoseTrolley() : 
    target_frame_("map"),
    tf2_listener_(buffer_),
    tf2_filter_(trolley_pose_sub_, buffer_, target_frame_, 10, 0)
  {
    ROS_INFO("PoseTrolley.");
    ros::NodeHandle nh("~");
    trolley_pose_sub_.subscribe(nh, "/cuda_icp/trolley_pose", 10);
    tf2_filter_.registerCallback(boost::bind(&PoseTrolley::trolleyPoseCb, this, _1));
    nav_goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    nh.param("offset_x", offset_x_, 1.1);
    nh.param("offset_z", offset_z_, 0.0);
    nh.param("offset_yaw", offset_yaw_, 0.0);
  }

  void trolleyPoseCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    ROS_INFO("Update goal...");
    trolley_pose_cam_.header = msg->header; 
    trolley_pose_cam_.pose.position = msg->pose.position;
    trolley_pose_cam_.pose.orientation = msg->pose.orientation;

    geometry_msgs::TransformStamped tf;
    try 
    {
      buffer_.transform(trolley_pose_cam_, trolley_pose_map_, target_frame_);
      tf = buffer_.lookupTransform("map", "trolley", msg->header.stamp, ros::Duration(5));

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
      ref_q.setRPY(0.0f, 0.0f, yaw - 1.5707f);

      // Combine the nav_goal_pose with position from marker_pose_map_, 
      // orientation from lookupTransform between map and Apriltag
      geometry_msgs::PoseStamped nav_goal_pose;
      nav_goal_pose = trolley_pose_map_;
      nav_goal_pose.header.frame_id = "map";
      nav_goal_pose.header.stamp = ros::Time::now();
      nav_goal_pose.pose.position.x -= offset_x_ * cos(yaw - 1.5707f);
      nav_goal_pose.pose.position.y -= offset_x_ * sin(yaw - 1.5707f);
      nav_goal_pose.pose.orientation.x = ref_q.x();
      nav_goal_pose.pose.orientation.y = ref_q.y();
      nav_goal_pose.pose.orientation.z = ref_q.z();
      nav_goal_pose.pose.orientation.w = ref_q.w();

      nav_goal_pub_.publish(nav_goal_pose);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Failure %s", ex.what());
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_listener_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> trolley_pose_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2_filter_;

  geometry_msgs::PoseStamped trolley_pose_cam_, trolley_pose_map_;
  geometry_msgs::PoseStamped nav_goal_pose_;
  ros::Publisher nav_goal_pub_;

  double offset_x_, offset_z_, offset_yaw_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vision_servoing_trolley");
  ros::NodeHandle nh("~");

  PoseTrolley pose_trolley;
  ros::spin();

  return 0;
}

