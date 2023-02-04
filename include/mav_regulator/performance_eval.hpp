/* Performance Evaluator                       */
/* Author: Lorenzo Gentilini                   */
/* E-Mail: lorenzo.gentilini6@unibo.it         */
/* Date: May 2022                              */
/* File: performance_eval.hpp                  */

#include <ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mav_regulator/BezierMultiArray.h>
#include <mav_regulator/SetPoint.h>

#define FREQ 20

typedef enum{ARM_WAIT, OFFBOARD_WAIT, TAKEOFF, WAIT_TAKEOFF, MOVE, IDLE} flightState;

class Evaluator{
    public:
    // Constructor
    Evaluator(ros::NodeHandle& nh);

    // Destructor
    ~Evaluator(){;};

    private:
    // Attribute
    double takeoffAltitude, goalThreshold, executionTime;
    Eigen::Matrix<double, 3, 1> actualPosition, currentSetpoint;
    flightState _state = ARM_WAIT;
    std::string pathToSettings;

    // Usefull Flags
    bool stateReceived = false, odomReceived = false;

    // Message Structs
    mavros_msgs::State currentState;
    mav_regulator::BezierMultiArray multiTrajectory;

    // Subscribers
    ros::Subscriber mavrosState_sub, odometry_sub;

    // Publisher
    ros::Publisher trajectoryMulti_pub, referencePoint_pub;

    // Timers
    ros::Timer loopTimer;

    void supervise(const ros::TimerEvent& e);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
};