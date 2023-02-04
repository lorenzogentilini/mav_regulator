#pragma once
//#define FLIGHTMARE
#define MAVROS

#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <chrono>
#include <thread>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <mav_regulator/SetPoint.h>
#include <mav_regulator/BezierMultiArray.h>
#include <mav_regulator/BSpline.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#ifdef MAVROS
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#endif

#ifdef FLIGHTMARE
#include <geometry_msgs/Vector3.h>
#include <quadrotor_msgs/ControlCommand.h>
#endif

#include <acado_auxiliary_functions.h>
#include <acado_common.h>
#include <mav_regulator/common_functions.hpp>

#define N   ACADO_N     // Number of Samples
#define NX  ACADO_NX    // Number of States
#define NY  ACADO_NY    // Number of Reference States
#define NYN ACADO_NYN   // Number of End Reference States
#define NU  ACADO_NU    // Number of Inputs
#define NOD ACADO_NOD   // Number of Online Data

#define refState    acadoVariables.y
#define refEndState acadoVariables.yN
#define initState   acadoVariables.x0
#define state       acadoVariables.x
#define input       acadoVariables.u
#define onlineData  acadoVariables.od
#define weight      acadoVariables.W
#define endWeight   acadoVariables.WN
#define lowerBoundU acadoVariables.lbValues
#define upperBoundU acadoVariables.ubValues
#define lowerBoundS acadoVariables.lbAValues
#define upperBoundS acadoVariables.ubAValues

typedef enum{ USE_POSITION,
              USE_VELOCITY,
              USE_ACCELERATION,
              USE_MIXED,
              USE_TRAJECTORY,
              USE_SPLINE} typeReference;

class LowPassFilter{
    public:
    double A, B, C;
    double x = 0;
    ros::Time previousTime = ros::Time::now();

    LowPassFilter(double cutoff_ff){
        double cutoff_w = cutoff_ff*2*M_PI;
        A = -cutoff_w;
        B = cutoff_w;
        C = 1.0;
    };

    void setInitialCondition(double x_){
        previousTime = ros::Time::now();
        x = x_/C;
    };

    double update(double u){
        double dt = (ros::Time::now() - previousTime).toSec();
        previousTime = ros::Time::now();

        x = x + (A*x + B*u)*dt;

        return C*x;
    };
};

// MPC Wrapper Class
class MpcRegulator{
    public:
    // Constructor
    MpcRegulator(ros::NodeHandle& nh);

    // Destructor
    ~MpcRegulator(){
        if(use_lp_filter){
            for(uint ii = 0; ii < lp_pp.size(); ii++){
                delete lp_pp[ii];
            }

            for(uint ii = 0; ii < lp_vv.size(); ii++){
                delete lp_vv[ii];
            }

            for(uint ii = 0; ii < lp_aa.size(); ii++){
                delete lp_aa[ii];
            }

            for(uint ii = 0; ii < lp_jj.size(); ii++){
                delete lp_jj[ii];
            }

            for(uint ii = 0; ii < lp_yy.size(); ii++){
                delete lp_yy[ii];
            }

            for(uint ii = 0; ii < lp_ww.size(); ii++){
                delete lp_ww[ii];
            }
        }
    };

    private:
    // Variables
    double q_px = 0.0, q_py = 0.0, q_pz = 0.0,
           q_qw = 0.0, q_qx = 0.0, q_qy = 0.0, q_qz = 0.0,
           q_vx = 0.0, q_vy = 0.0, q_vz = 0.0,
           r_th = 0.0, r_wx = 0.0, r_wy = 0.0, r_wz = 0.0;

    double g = 0.0, dt = 0.0, T_max = 0.0, T_min = 0.0,
           w_xy_max = 0.0, w_z_max = 0.0, v_max = 0.0, a_max = 0.0,
           startTime = 0.0;

    double last_referenceYaw = 0.0;

    bool position_valid      = false,
         velocity_valid      = false,
         quaternion_valid    = false,
         reference_valid     = false,
         globalPose_received = false,
         odom_valid          = false,
         is_rotating         = false,
         is_calling          = false;
    unsigned short seq = 0;

    bool use_lp_filter;
    double ff_pp, ff_vv, ff_aa, ff_jj, ff_yy, ff_ww;

    // Structures
    Eigen::Matrix<float, 1, NX> fullEstState    = Eigen::MatrixXf::Zero(1, NX);
    Eigen::Matrix<float, 1, NY> hoverRefState   = Eigen::MatrixXf::Zero(1, NY);
    Eigen::Matrix<float, 1, NX> hoverState      = Eigen::MatrixXf::Zero(1, NX);
    Eigen::Matrix<float, 1, NU> hoverInput      = Eigen::MatrixXf::Zero(1, NU);
    Eigen::Matrix<float, 1, NU> ubInput         = Eigen::MatrixXf::Zero(1, NU);
    Eigen::Matrix<float, 1, NU> lbInput         = Eigen::MatrixXf::Zero(1, NU);
    Eigen::Matrix<float, NY, NY> W              = Eigen::MatrixXf::Zero(NY, NY);

    // For References
    Eigen::Matrix<float, 1, 3> actualRef_forIntegral = Eigen::MatrixXf::Zero(1, 3);
    Eigen::Matrix<float, 1, 3> actualError = Eigen::MatrixXf::Zero(1, 3);
    Eigen::Matrix<float, 1, 3> positionReference = Eigen::MatrixXf::Zero(1, 3);
    Eigen::Matrix<float, 1, 3> positionInitial = Eigen::MatrixXf::Zero(1, 3);

    // For Bezier Reference
    std::vector<std::vector<float>> time_controlPoints;
    std::vector<float> executionTimes;
    std::vector<bool> is_timeScaled;
    std::vector<bool> is_invertedTime;

    // For Spline Reference
    std::vector<std::vector<float>> p_spline_knots;
    std::vector<std::vector<float>> y_spline_knots;

    std::vector<std::vector<float>> p_spline_knots_rotated;
    std::vector<std::vector<float>> y_spline_knots_rotated;

    std::vector<std::vector<float>> p_spline_knots_actual;
    std::vector<std::vector<float>> y_spline_knots_actual;

    // Common
    std::vector<std::vector<Eigen::Matrix<float, 1, 3>>> position_controlPoints;
    std::vector<std::vector<float>> orientation_controlPoints;

    std::vector<std::vector<Eigen::Matrix<float, 1, 3>>> position_controlPoints_rotated;
    std::vector<std::vector<float>> orientation_controlPoints_rotated;

    std::vector<std::vector<Eigen::Matrix<float, 1, 3>>> position_controlPoints_actual;
    std::vector<std::vector<float>> orientation_controlPoints_actual;

    std::vector<std::string> actualFrames;
    std::vector<bool> is_yawFollowing;

    std::vector<bool> is_yawFollowing_actual;
    std::vector<bool> is_yawFollowing_rotated;

    std::vector<LowPassFilter*> lp_pp;
    std::vector<LowPassFilter*> lp_vv;
    std::vector<LowPassFilter*> lp_aa;
    std::vector<LowPassFilter*> lp_jj;
    std::vector<LowPassFilter*> lp_yy;
    std::vector<LowPassFilter*> lp_ww;

    std::mutex mtx;
    ros::Time last_refRotation = ros::Time::now();

    std::string actualFrame;
    typeReference actualReference;
    float yawReference, yawInitial, executionTime;
    double committedTime = (ros::Time::now()).toSec();

    #ifdef FLIGHTMARE
    quadrotor_msgs::ControlCommand controlAction;
    #endif

    #ifdef MAVROS
    mavros_msgs::State currentState;
    mavros_msgs::AttitudeTarget controlAction;
    #endif

    // Ros Stuffs
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;
    tf2::Transform mapToOdom_transformation;

    ros::Subscriber stateSub;
    ros::Subscriber poseSub;
    ros::Subscriber velSub;
    ros::Subscriber imuSub;
    ros::Subscriber odomSub;
    ros::Subscriber setpointSub;
    ros::Subscriber trajSub;
    ros::Subscriber splineSub;
    ros::Publisher controlPub;
    ros::Timer updateTimer;
    ros::Timer controlTimer;

    // Helper Functions
    void setReference();
    void rotateReference(bool blocking);
    void run();
    void reset();
    void dynamicInversion(Eigen::Matrix<float, 1, 3> p, Eigen::Matrix<float, 1, 3> v,
                          Eigen::Matrix<float, 1, 3> a, Eigen::Matrix<float, 1, 3> j,
                          float yaw, float w,
                          Eigen::Matrix<float, 1, 4>& q, Eigen::Matrix<float, 1, 4>& u);

    // Callbacks
    #ifdef MAVROS
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    #endif

    #ifdef FLIGHTMARE
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    #endif

    void setPointCallback(const mav_regulator::SetPoint::ConstPtr& msg);
    void trajectoryCallback(const mav_regulator::BezierMultiArray::ConstPtr& msg);
    void sendControl(const ros::TimerEvent& e);
    void updateReference(const ros::TimerEvent& e);
	void splineCallback(const mav_regulator::BSpline::ConstPtr& msg);

    void checkRef_pp(Eigen::Matrix<float, 1, 3>& pp);
    void checkRef_vv(Eigen::Matrix<float, 1, 3>& vv);
    void checkRef_aa(Eigen::Matrix<float, 1, 3>& aa);
    void checkRef_jj(Eigen::Matrix<float, 1, 3>& jj);
    void checkRef_yy(float& yy);
    void checkRef_ww(float& ww);

    void lpReset(Eigen::Matrix<float, 1, 3> pp_recovery, Eigen::Matrix<float, 1, 3> vv_recovery,
                 Eigen::Matrix<float, 1, 3> aa_recovery, Eigen::Matrix<float, 1, 3> jj_recovery,
                 float yy_recovery, float ww_recovery);

    double to_PI(double yy){
		while(yy < -M_PI){
			yy = yy + 2*M_PI;
		}
		
		while(yy > M_PI){
			yy = yy - 2*M_PI;
		}

		return yy;
	};
};