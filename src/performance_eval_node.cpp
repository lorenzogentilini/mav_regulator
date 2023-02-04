#include <mav_regulator/performance_eval.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "evaluator_node");
  ros::NodeHandle nh("~");

  Evaluator eval(nh);

  ros::spin();

  return 0;
}