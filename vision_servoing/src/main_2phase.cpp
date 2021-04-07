

#include "vision_servoing/AprilTagServo.h"

//-------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vision_servoing_2phase");
  ros::NodeHandle nh("~");
  vision_servoing::AprilTagServo aprilTagServo(nh);

  ros::spin();

  return 0;
}
