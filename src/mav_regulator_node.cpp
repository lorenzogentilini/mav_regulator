#include <mav_regulator/mav_regulator.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "mav_regulator_node");
  ros::NodeHandle nh("~");

  ros::MultiThreadedSpinner spinner(2);
  MpcRegulator regulator(nh);

  spinner.spin();
  return 0;
}