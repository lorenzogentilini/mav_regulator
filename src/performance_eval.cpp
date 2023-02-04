#include <mav_regulator/performance_eval.hpp>

Evaluator::Evaluator(ros::NodeHandle& nh):
    mavrosState_sub(nh.subscribe("/mavros/state", 1, &Evaluator::stateCallback, this)),
    odometry_sub(nh.subscribe("/mavros/local_position/odom", 1, &Evaluator::odometryCallback, this)),
    trajectoryMulti_pub(nh.advertise<mav_regulator::BezierMultiArray>("/trajectory", 1)),
    referencePoint_pub(nh.advertise<mav_regulator::SetPoint>("/setpoint", 1)),
    loopTimer(nh.createTimer(ros::Duration(1/(double)FREQ), &Evaluator::supervise, this)){

    nh.param("execution_time", executionTime, 5.0);
    nh.param("takeoff_altitude", takeoffAltitude, 2.0);
    nh.param("goal_thr", goalThreshold, 0.2);
    nh.param("trajectory", pathToSettings, std::string("/home"));

    // Load Trajectories
    YAML::Node conf = YAML::LoadFile(pathToSettings);

    multiTrajectory.action = 1;
    for(uint jj = 0; jj < conf["traj_pieces"].size(); jj++){
        mav_regulator::Bezier actualTrajectory;
        for(uint kk = 0; kk < conf["traj_pieces"][jj].size(); kk++){
            actualTrajectory.x.push_back(conf["traj_pieces"][jj][kk]["x"].as<double>());
            actualTrajectory.y.push_back(conf["traj_pieces"][jj][kk]["y"].as<double>());
            actualTrajectory.z.push_back(conf["traj_pieces"][jj][kk]["z"].as<double>());
            actualTrajectory.yaw.push_back(0.0);
        }

        actualTrajectory.totalTime = executionTime;
        actualTrajectory.frame = "odom";
        actualTrajectory.is_timeScaled = false;
        actualTrajectory.is_timeInverted = false;

        multiTrajectory.trajectory.push_back(actualTrajectory);
    }
}

void Evaluator::supervise(const ros::TimerEvent& e){
    if(!currentState.connected || !stateReceived || !odomReceived)
        return;

    // State Machine
    switch(_state){
        case ARM_WAIT:{
            if(currentState.armed){
                ROS_INFO("Armed!");
                _state = OFFBOARD_WAIT;
            }

            break;
        }

        case OFFBOARD_WAIT:{
            if(currentState.mode == "OFFBOARD" && currentState.armed){    
                ROS_INFO("Offboard Enabled");
                _state = TAKEOFF;
            }

            if(!currentState.armed){
                ROS_INFO("Dearmed");
                _state = ARM_WAIT;
            }

            break;
        }

        case TAKEOFF:{
            currentSetpoint << actualPosition(0), actualPosition(1), actualPosition(2)+takeoffAltitude;
            
            // Takeoff Triplets
            mav_regulator::SetPoint msg;
            msg.action = mav_regulator::SetPoint::USE_POSITION;
            msg.x = currentSetpoint(0);
            msg.y = currentSetpoint(1);
            msg.z = currentSetpoint(2);
            msg.yaw = 0.0;
            msg.time = 5.0;

            referencePoint_pub.publish(msg);

            _state = WAIT_TAKEOFF;

            if(currentState.mode != "OFFBOARD"){
                ROS_INFO("Lost Offboard");
                _state = OFFBOARD_WAIT;
            }

            if(!currentState.armed){
                ROS_INFO("Dearmed");
                _state = ARM_WAIT;
            }

            break;
        }

        case WAIT_TAKEOFF:{
            if((actualPosition - currentSetpoint).squaredNorm() < goalThreshold){
                ROS_INFO("Takeoff Done!");
                _state = MOVE;
            }

            if(currentState.mode != "OFFBOARD"){
                ROS_INFO("Lost Offboard");
                _state = OFFBOARD_WAIT;
            }

            if(!currentState.armed){
                ROS_INFO("Dearmed");
                _state = ARM_WAIT;
            }

            break;
        }

        case MOVE:{
            trajectoryMulti_pub.publish(multiTrajectory);
            _state = IDLE;

            if(currentState.mode != "OFFBOARD"){
                ROS_INFO("Lost Offboard");
                _state = OFFBOARD_WAIT;
            }

            if(!currentState.armed){
                ROS_INFO("Dearmed");
                _state = ARM_WAIT;
            }

            break;
        }

        case IDLE:{
            if(currentState.mode != "OFFBOARD"){
                ROS_INFO("Lost Offboard");
                _state = OFFBOARD_WAIT;
            }

            if(!currentState.armed){
                ROS_INFO("Dearmed");
                _state = ARM_WAIT;
            }

            break;
        }
    }
}

void Evaluator::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
    stateReceived = true;
}

void Evaluator::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    // Memorize Position
    actualPosition << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odomReceived = true;
}