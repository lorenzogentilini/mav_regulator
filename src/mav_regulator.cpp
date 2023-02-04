#include <mav_regulator/mav_regulator.hpp>

const double run_freq = 0.005; // Regulator Update Frequency
const double spu_freq = 0.5; // Setpoint Update Frequency

const int spline_order = 7;
const int spline_order_yaw = 3;

// Global Variables Used by the Solver
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MpcRegulator::MpcRegulator(ros::NodeHandle& nh): _tfListener(_tfBuffer),
    #ifdef MAVROS
    stateSub(nh.subscribe("/mavros/state", 1, &MpcRegulator::stateCallback, this)),
    poseSub(nh.subscribe("/mavros/local_position/pose", 1, &MpcRegulator::positionCallback, this)),
    velSub(nh.subscribe("/mavros/local_position/velocity_local", 1, &MpcRegulator::velocityCallback, this)),
    imuSub(nh.subscribe("/mavros/imu/data", 1, &MpcRegulator::imuCallback, this)),
    #endif
    #ifdef FLIGHTMARE
    odomSub(nh.subscribe("/hummingbird/ground_truth/odometry", 1, &MpcRegulator::odometryCallback, this)),
    #endif
    setpointSub(nh.subscribe("/setpoint", 1, &MpcRegulator::setPointCallback, this)),
    trajSub(nh.subscribe("/trajectory", 1, &MpcRegulator::trajectoryCallback, this)),
    splineSub(nh.subscribe("/spline", 1, &MpcRegulator::splineCallback, this)),
    #ifdef MAVROS
    controlPub(nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1)),
    #endif
    #ifdef FLIGHTMARE
    controlPub(nh.advertise<quadrotor_msgs::ControlCommand>("/hummingbird/control_command", 1)),
    #endif
    updateTimer(nh.createTimer(ros::Duration(spu_freq), &MpcRegulator::updateReference, this)),
    controlTimer(nh.createTimer(ros::Duration(run_freq), &MpcRegulator::sendControl, this)){

    nh.param("mpc/q_px", q_px, 200.0);
    nh.param("mpc/q_py", q_py, 200.0);
    nh.param("mpc/q_pz", q_pz, 500.0);
    nh.param("mpc/q_qw", q_qw, 100.0);
    nh.param("mpc/q_qx", q_qx, 50.0);
    nh.param("mpc/q_qy", q_qy, 50.0);
    nh.param("mpc/q_qz", q_qz, 100.0);
    nh.param("mpc/q_vx", q_vx, 10.0);
    nh.param("mpc/q_vy", q_vy, 10.0);
    nh.param("mpc/q_vz", q_vz, 10.0);
    nh.param("mpc/q_th", r_th, 50.0);
    nh.param("mpc/q_wx", r_wx, 100.0);
    nh.param("mpc/q_wy", r_wy, 100.0);
    nh.param("mpc/q_wz", r_wz, 50.0);
    nh.param("mpc/dt", dt, 0.05);
    nh.param("mav/gravity", g, 9.8055);
    nh.param("mav/T_max", T_max, 20.0);
    nh.param("mav/T_min", T_min, 2.0);
    nh.param("mav/w_xy_max", w_xy_max, 3.0);
    nh.param("mav/w_z_max", w_z_max, 2.0);

    nh.param("mav/use_low_pass", use_lp_filter, false);
    nh.param("mav/ff_pp", ff_pp, 10.0);
    nh.param("mav/ff_vv", ff_vv, 10.0);
    nh.param("mav/ff_aa", ff_aa, 10.0);
    nh.param("mav/ff_jj", ff_jj, 10.0);
    nh.param("mav/ff_yy", ff_yy, 10.0);
    nh.param("mav/ff_ww", ff_ww, 10.0);

    // Solver Memory
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    // Variables Initialization
    hoverRefState(3) = 1;
    hoverState(3) = 1;
    hoverInput(0) = g;

    // Set Input Limits
    ubInput << T_max, w_xy_max, w_xy_max, w_z_max; 
    lbInput << T_min, -w_xy_max, -w_xy_max, -w_z_max;

    // Initialize Q & R Matrices
    W(0,0) = q_px;
    W(1,1) = q_py;
    W(2,2) = q_pz;
    W(3,3) = q_qw;
    W(4,4) = q_qx;
    W(5,5) = q_qy;
    W(6,6) = q_qz;
    W(7,7) = q_vx;
    W(8,8) = q_vy;
    W(9,9) = q_vz;
    W(10,10) = r_th;
    W(11,11) = r_wx;
    W(12,12) = r_wy;
    W(13,13) = r_wz;

    // Initialize the Solver
    acado_initializeSolver();

    // Initialize ACADO Variables
    fillRawBuffer<Eigen::Matrix<float, 1, NX>>(initState, hoverState, 1);
    fillRawBuffer<Eigen::Matrix<float, 1, NX>>(state, hoverState, N+1);
    fillRawBuffer<Eigen::Matrix<float, 1, NU>>(input, hoverInput, N);
    fillRawBuffer<Eigen::Matrix<float, 1, NY>>(refState, hoverRefState, N);
    fillRawBuffer<Eigen::Matrix<float, 1, NX>>(refEndState, hoverState, 1);  
    fillRawBuffer<Eigen::Matrix<float, NY, NY>>(weight, W, N);
    fillRawBuffer<Eigen::Matrix<float, NX, NX>>(endWeight, W.block(0,0,NX,NX), 1);
    fillRawBuffer<Eigen::Matrix<float, 1, 4>>(upperBoundU, ubInput, N);
    fillRawBuffer<Eigen::Matrix<float, 1, 4>>(lowerBoundU, lbInput, N);

    // Initialize Solver
    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();

    if(use_lp_filter){
        lp_pp.resize(3);
        lp_vv.resize(3);
        lp_aa.resize(3);
        lp_jj.resize(3);

        lp_yy.resize(1);
        lp_ww.resize(1);

        for(uint ii = 0; ii < lp_pp.size(); ii++){
            lp_pp[ii] = new LowPassFilter(ff_pp);
        }

        for(uint ii = 0; ii < lp_vv.size(); ii++){
            lp_vv[ii] = new LowPassFilter(ff_vv);
        }

        for(uint ii = 0; ii < lp_aa.size(); ii++){
            lp_aa[ii] = new LowPassFilter(ff_aa);
        }

        for(uint ii = 0; ii < lp_jj.size(); ii++){
            lp_jj[ii] = new LowPassFilter(ff_jj);
        }

        for(uint ii = 0; ii < lp_yy.size(); ii++){
            lp_yy[ii] = new LowPassFilter(ff_yy);
        }

        for(uint ii = 0; ii < lp_ww.size(); ii++){
            lp_ww[ii] = new LowPassFilter(ff_ww);
        }
    }
}

void MpcRegulator::run(){
    // Read Transformation
    try{
        tf2::fromMsg(_tfBuffer.lookupTransform("map", "odom", ros::Time(0)).transform, mapToOdom_transformation);
        globalPose_received = true;
    } catch(tf2::TransformException &ex){
        // Nothing
        ;
    }

    // Set Actual Reference
    setReference();

    actualError += (fullEstState.block(0,0,1,3) - actualRef_forIntegral)*run_freq;

    Eigen::Matrix<float, N+1, NOD> onlineData_values = Eigen::MatrixXf::Zero(N+1, NOD);
    for(uint ii = 0; ii < N+1; ii++){
        onlineData_values(ii, 0) = actualError(0);
        onlineData_values(ii, 1) = actualError(1);
        onlineData_values(ii, 2) = actualError(2);
    }

    fillRawBuffer<Eigen::Matrix<float, N+1, NOD>>(onlineData, onlineData_values, 1);

    // Fill with State Estimation
    fillRawBuffer<Eigen::Matrix<float, 1, NX>>(initState, fullEstState, 1);

    // Run Control
    acado_feedbackStep();
    acado_preparationStep();

    // Update Control Action
    #ifdef MAVROS
    controlAction.thrust = input[0]/T_max;  // Normalize for Throttle
    controlAction.body_rate.x = input[1];
    controlAction.body_rate.y = input[2];
    controlAction.body_rate.z = input[3];
    #endif
    #ifdef FLIGHTMARE
    controlAction.collective_thrust = input[0];
    geometry_msgs::Vector3 br;
    br.x = input[1];
    br.y = input[2];
    br.z = input[3];
    controlAction.bodyrates = br;
    #endif
}

void MpcRegulator::setReference(){
    // mtx.lock();
    switch(actualReference){
        case USE_POSITION:{
            Eigen::Matrix<float, N, NY> committedStateRef = Eigen::MatrixXf::Zero(N, NY);
            Eigen::Matrix<float, 1, 3> positionReference_true = positionReference;
            float yawReference_true = yawReference;

            if(actualFrame == "map"){
                // Convert Setpoint in Map Coordinate
                tf2::Quaternion mapToRef_quaternion;
                mapToRef_quaternion.setRPY(0.0, 0.0, yawReference);

                tf2::Vector3 mapToRef_translation(positionReference(0), positionReference(1), positionReference(2));
                tf2::Transform mapToRef_transformation(mapToRef_quaternion, mapToRef_translation);
                tf2::Transform odomToRef_transformation = (mapToRef_transformation.inverse()*mapToOdom_transformation).inverse();
                tf2::Quaternion odomToRef_quaternion = odomToRef_transformation.getRotation();
                tf2::Vector3 odomToRef_translation = odomToRef_transformation.getOrigin();

                positionReference_true(0) = odomToRef_translation.x();
                positionReference_true(1) = odomToRef_translation.y();
                positionReference_true(2) = odomToRef_translation.z();

                tf2::Matrix3x3 m(odomToRef_quaternion);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                yawReference_true = yaw;
            }

            startTime = abs((ros::Time::now()).toSec() - committedTime);

            Eigen::Matrix<float, 1, 3> pp_recovery;
            Eigen::Matrix<float, 1, 3> vv_recovery;
            Eigen::Matrix<float, 1, 3> aa_recovery;
            Eigen::Matrix<float, 1, 3> jj_recovery;

            double yy_recovery;
            double ww_recovery;

            float referenceYaw, referenceOmega;
            Eigen::Matrix<float, 1, 3> referencePosition, referenceVelocity, referenceAcceleration, referenceJerk;
            for(uint ii = 0; ii < N; ii++){
                // Evaluate Bézier Curve
                referencePosition = evaluateBezierFromPoint<Eigen::Matrix<float, 1, 3>>(positionInitial, positionReference, executionTime, startTime + dt*ii);
                referenceYaw = evaluateBezierFromPoint<float>(yawInitial, yawReference, executionTime, startTime + dt*ii);
                referenceVelocity = evaluateBezierDerivativeFromPoint<Eigen::Matrix<float, 1, 3>>(positionInitial, positionReference, executionTime, startTime + dt*ii, 1);
                referenceAcceleration = evaluateBezierDerivativeFromPoint<Eigen::Matrix<float, 1, 3>>(positionInitial, positionReference, executionTime, startTime + dt*ii, 2);
                referenceJerk = evaluateBezierDerivativeFromPoint<Eigen::Matrix<float, 1, 3>>(positionInitial, positionReference, executionTime, startTime + dt*ii, 3);
                referenceOmega = evaluateBezierDerivativeFromPoint<float>(yawInitial, yawReference, executionTime, startTime + dt*ii, 1);

                checkRef_pp(referencePosition);
                checkRef_vv(referenceVelocity);
                checkRef_aa(referenceAcceleration);
                checkRef_jj(referenceJerk);
                checkRef_yy(referenceYaw);
                checkRef_ww(referenceOmega);

                if(ii == 0){
                    actualRef_forIntegral = referencePosition;
                    pp_recovery = referencePosition;
                    vv_recovery = referenceVelocity;
                    aa_recovery = referenceAcceleration;
                    jj_recovery = referenceJerk;
                    yy_recovery = referenceYaw;
                    ww_recovery = referenceOmega;
                }

                // Normalize Yaw
                referenceYaw = to_PI(referenceYaw);

                Eigen::Matrix<float, 1, 4> attitudeQuaternion, controlInput;
                dynamicInversion(referencePosition, referenceVelocity, referenceAcceleration, referenceJerk, referenceYaw, referenceOmega, attitudeQuaternion, controlInput);

                committedStateRef(ii, 0) = (float)referencePosition(0);
                committedStateRef(ii, 1) = (float)referencePosition(1);
                committedStateRef(ii, 2) = (float)referencePosition(2);

                committedStateRef(ii, 3) = (float)attitudeQuaternion(0);
                committedStateRef(ii, 4) = (float)attitudeQuaternion(1);
                committedStateRef(ii, 5) = (float)attitudeQuaternion(2);
                committedStateRef(ii, 6) = (float)attitudeQuaternion(3);
                
                committedStateRef(ii, 7) = (float)referenceVelocity(0);
                committedStateRef(ii, 8) = (float)referenceVelocity(1);
                committedStateRef(ii, 9) = (float)referenceVelocity(2);
                
                committedStateRef(ii, 10) = (float)controlInput(0);
                // committedStateRef(ii, 11) = (float)controlInput(1);
                // committedStateRef(ii, 12) = (float)controlInput(2);
                // committedStateRef(ii, 13) = (float)controlInput(3);

            }

            fillRawBuffer<Eigen::Matrix<float, N, NY>>(refState, committedStateRef, 1);
            fillRawBuffer<Eigen::Matrix<float, 1, NYN>>(refEndState, committedStateRef.block(N-1, 0, 1, NYN), 1);

            if(use_lp_filter){
                lpReset(pp_recovery, vv_recovery, aa_recovery, jj_recovery, yy_recovery, ww_recovery);
            }

            break;
        }

        case USE_TRAJECTORY:{
            Eigen::Matrix<float, N, NY> committedStateRef = Eigen::MatrixXf::Zero(N, NY);
            startTime = abs((ros::Time::now()).toSec() - committedTime);

            if(startTime > executionTimes[0] && executionTimes.size() > 1){
                committedTime = (ros::Time::now()).toSec();
                startTime = abs((startTime - executionTimes[0]) + (ros::Time::now()).toSec() - committedTime);

                executionTimes.erase(executionTimes.begin());
                position_controlPoints.erase(position_controlPoints.begin());
                orientation_controlPoints.erase(orientation_controlPoints.begin());
                actualFrames.erase(actualFrames.begin());
                is_timeScaled.erase(is_timeScaled.begin());
                time_controlPoints.erase(time_controlPoints.begin());
                is_invertedTime.erase(is_invertedTime.begin());
                is_yawFollowing.erase(is_yawFollowing.begin());

                // position_controlPoints_rotated.erase(position_controlPoints_rotated.begin());
                // orientation_controlPoints_rotated.erase(orientation_controlPoints_rotated.begin());

                rotateReference(true);

                position_controlPoints_actual = position_controlPoints_rotated;
                orientation_controlPoints_actual = orientation_controlPoints_rotated;
            } else if(startTime > executionTimes[0]){
                startTime = executionTimes[0];
            }

            Eigen::Matrix<float, 1, 3> pp_recovery;
            Eigen::Matrix<float, 1, 3> vv_recovery;
            Eigen::Matrix<float, 1, 3> aa_recovery;
            Eigen::Matrix<float, 1, 3> jj_recovery;

            double yy_recovery;
            double ww_recovery;

            float residualTime = startTime;
            uint newIdx = 0, actualIdx = 0;

            if(!is_rotating){
                position_controlPoints_actual = position_controlPoints_rotated;
                orientation_controlPoints_actual = orientation_controlPoints_rotated;
            }

            for(uint ii = 0; ii < N; ii++){
                // Evaluate Position & Orientation
                double mpcTime = residualTime + dt*(ii - newIdx);
                if(mpcTime > executionTimes[actualIdx] && executionTimes.size() > (actualIdx + 1)){
                    actualIdx ++;
                    newIdx = ii;
                    residualTime = mpcTime - executionTimes[actualIdx];
                    mpcTime = residualTime + dt*(ii - newIdx);
                }

                float referenceYaw, referenceOmega;
                Eigen::Matrix<float, 1, 3> referencePosition, referenceVelocity, referenceAcceleration, referenceJerk;
                if(is_timeScaled[actualIdx]){
                    referencePosition = evaluateBezier_timeScaled<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx]);
                    referenceVelocity = evaluateBezierDerivative_timeScaled<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 1);
                    referenceAcceleration = evaluateBezierDerivative_timeScaled<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 2);
                    referenceJerk = evaluateBezierDerivative_timeScaled<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 3);
                    referenceYaw = evaluateBezier_timeScaled<float>(orientation_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx]);
                    referenceOmega = evaluateBezierDerivative_timeScaled<float>(orientation_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 1);
                } else{
                    referencePosition = evaluateBezier<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx]);
                    referenceVelocity = evaluateBezierDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 1);
                    referenceAcceleration = evaluateBezierDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 2);
                    referenceJerk = evaluateBezierDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 3);
                    referenceYaw = evaluateBezier<float>(orientation_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx]);
                    referenceOmega = evaluateBezierDerivative<float>(orientation_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime, is_invertedTime[actualIdx], 1);
                }

                if(is_yawFollowing[actualIdx]){
                    Eigen::Matrix<float, 1, 3> referenceVelocity_new;
                    if(is_timeScaled[actualIdx]){
                        referenceVelocity_new = evaluateBezierDerivative_timeScaled<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], time_controlPoints[actualIdx], executionTimes[actualIdx], mpcTime+1.0, is_invertedTime[actualIdx], 1);
                    } else{
                        referenceVelocity_new = evaluateBezierDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], executionTimes[actualIdx], mpcTime+1.0, is_invertedTime[actualIdx], 1);
                    }

                    referenceYaw = atan2(referenceVelocity_new(1), referenceVelocity_new(0));
                    if(referenceVelocity_new.norm() < 0.1){
                        referenceYaw = last_referenceYaw;
                    }

                    last_referenceYaw = referenceYaw;
                    referenceOmega = 0.0;
                }

                checkRef_pp(referencePosition);
                checkRef_vv(referenceVelocity);
                checkRef_aa(referenceAcceleration);
                checkRef_jj(referenceJerk);
                checkRef_yy(referenceYaw);
                checkRef_ww(referenceOmega);

                if(ii == 0){
                    actualRef_forIntegral = referencePosition;
                    pp_recovery = referencePosition;
                    vv_recovery = referenceVelocity;
                    aa_recovery = referenceAcceleration;
                    jj_recovery = referenceJerk;
                    yy_recovery = referenceYaw;
                    ww_recovery = referenceOmega;
                }

                // Normalize Yaw
                referenceYaw = to_PI(referenceYaw);

                Eigen::Matrix<float, 1, 4> attitudeQuaternion, controlInput;
                dynamicInversion(referencePosition, referenceVelocity, referenceAcceleration, referenceJerk, referenceYaw, referenceOmega, attitudeQuaternion, controlInput);

                committedStateRef(ii, 0) = (float)referencePosition(0);
                committedStateRef(ii, 1) = (float)referencePosition(1);
                committedStateRef(ii, 2) = (float)referencePosition(2);

                committedStateRef(ii, 3) = (float)attitudeQuaternion(0);
                committedStateRef(ii, 4) = (float)attitudeQuaternion(1);
                committedStateRef(ii, 5) = (float)attitudeQuaternion(2);
                committedStateRef(ii, 6) = (float)attitudeQuaternion(3);
                
                committedStateRef(ii, 7) = (float)referenceVelocity(0);
                committedStateRef(ii, 8) = (float)referenceVelocity(1);
                committedStateRef(ii, 9) = (float)referenceVelocity(2);
                
                committedStateRef(ii, 10) = (float)controlInput(0);
                // committedStateRef(ii, 11) = (float)controlInput(1);
                // committedStateRef(ii, 12) = (float)controlInput(2);
                // committedStateRef(ii, 13) = (float)controlInput(3);
            }

            fillRawBuffer<Eigen::Matrix<float, N, NY>>(refState, committedStateRef, 1);
            fillRawBuffer<Eigen::Matrix<float, 1, NYN>>(refEndState, committedStateRef.block(N-1, 0, 1, NYN), 1);

            if(use_lp_filter){
                lpReset(pp_recovery, vv_recovery, aa_recovery, jj_recovery, yy_recovery, ww_recovery);
            }

            break;
        }

        case USE_SPLINE:{
            Eigen::Matrix<float, N, NY> committedStateRef = Eigen::MatrixXf::Zero(N, NY);
            startTime = abs((ros::Time::now()).toSec() - committedTime);

            if(startTime > p_spline_knots_actual[0][p_spline_knots_actual[0].size()-1] &&
               p_spline_knots_actual.size() > 1){

                committedTime = (ros::Time::now()).toSec();
                startTime = abs((startTime - p_spline_knots_actual[0][p_spline_knots_actual[0].size()-1]) + (ros::Time::now()).toSec() - committedTime);

                position_controlPoints.erase(position_controlPoints.begin());
                orientation_controlPoints.erase(orientation_controlPoints.begin());
                p_spline_knots.erase(p_spline_knots.begin());
                y_spline_knots.erase(y_spline_knots.begin());
                actualFrames.erase(actualFrames.begin());
                is_yawFollowing.erase(is_yawFollowing.begin());

                // orientation_controlPoints_rotated.erase(orientation_controlPoints_rotated.begin());
                // position_controlPoints_rotated.erase(position_controlPoints_rotated.begin());

                rotateReference(true);

                position_controlPoints_actual = position_controlPoints_rotated;
                orientation_controlPoints_actual = orientation_controlPoints_rotated;

                p_spline_knots_actual = p_spline_knots_rotated;
                y_spline_knots_actual = y_spline_knots_rotated;

                is_yawFollowing_actual = is_yawFollowing_rotated;
            } else if(startTime > p_spline_knots_actual[0][p_spline_knots_actual[0].size()-1]){
                startTime = p_spline_knots_actual[0][p_spline_knots_actual[0].size()-1];
            }

            Eigen::Matrix<float, 1, 3> pp_recovery;
            Eigen::Matrix<float, 1, 3> vv_recovery;
            Eigen::Matrix<float, 1, 3> aa_recovery;
            Eigen::Matrix<float, 1, 3> jj_recovery;

            double yy_recovery;
            double ww_recovery;

            float residualTime = startTime;
            uint newIdx = 0, actualIdx = 0;

            if(!is_rotating){
                position_controlPoints_actual = position_controlPoints_rotated;
                orientation_controlPoints_actual = orientation_controlPoints_rotated;

                p_spline_knots_actual = p_spline_knots_rotated;
                y_spline_knots_actual = y_spline_knots_rotated;

                is_yawFollowing_actual = is_yawFollowing_rotated;
            }

            for(uint ii = 0; ii < N; ii++){
                // Evaluate Position & Orientation
                double mpcTime = residualTime + dt*(ii - newIdx);
                if(mpcTime > p_spline_knots_actual[actualIdx][p_spline_knots_actual[0].size()-1] &&
                   p_spline_knots_actual.size() > (actualIdx + 1)){

                    actualIdx ++;
                    newIdx = ii;
                    residualTime = mpcTime - p_spline_knots_actual[actualIdx][p_spline_knots_actual[0].size()-1];
                    mpcTime = residualTime + dt*(ii - newIdx);
                }

                float referenceYaw, referenceOmega;
                Eigen::Matrix<float, 1, 3> referencePosition, referenceVelocity, referenceAcceleration, referenceJerk;
                referencePosition = evaluateBSpline<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], p_spline_knots_actual[actualIdx], mpcTime, spline_order);
                referenceVelocity = evaluateBSplineDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], p_spline_knots_actual[actualIdx], mpcTime, spline_order, 1);
                referenceAcceleration = evaluateBSplineDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], p_spline_knots_actual[actualIdx], mpcTime, spline_order, 2);
                referenceJerk = evaluateBSplineDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], p_spline_knots_actual[actualIdx], mpcTime, spline_order, 3);

                if(is_yawFollowing_actual[actualIdx]){
                    Eigen::Matrix<float, 1, 3> referenceVelocity_new;
                    referenceVelocity_new = evaluateBSplineDerivative<Eigen::Matrix<float, 1, 3>>(position_controlPoints_actual[actualIdx], p_spline_knots_actual[actualIdx], mpcTime+1.0, spline_order, 1);

                    referenceYaw = atan2(referenceVelocity_new(1), referenceVelocity_new(0));
                    if(referenceVelocity_new.norm() < 0.1){
                        referenceYaw = last_referenceYaw;
                    }

                    last_referenceYaw = referenceYaw;
                    referenceOmega = 0.0;
                } else{
                    referenceYaw = evaluateBSpline<float>(orientation_controlPoints_actual[actualIdx], y_spline_knots_actual[actualIdx], mpcTime, spline_order_yaw);
                    referenceOmega = evaluateBSplineDerivative<float>(orientation_controlPoints_actual[actualIdx], y_spline_knots_actual[actualIdx], mpcTime, spline_order_yaw, 1);
                }

                checkRef_pp(referencePosition);
                checkRef_vv(referenceVelocity);
                checkRef_aa(referenceAcceleration);
                checkRef_jj(referenceJerk);
                checkRef_yy(referenceYaw);
                checkRef_ww(referenceOmega);

                if(ii == 0){
                    actualRef_forIntegral = referencePosition;
                    pp_recovery = referencePosition;
                    vv_recovery = referenceVelocity;
                    aa_recovery = referenceAcceleration;
                    jj_recovery = referenceJerk;
                    yy_recovery = referenceYaw;
                    ww_recovery = referenceOmega;
                }

                // Normalize Yaw
                referenceYaw = to_PI(referenceYaw);

                Eigen::Matrix<float, 1, 4> attitudeQuaternion, controlInput;
                dynamicInversion(referencePosition, referenceVelocity, referenceAcceleration, referenceJerk, referenceYaw, referenceOmega, attitudeQuaternion, controlInput);

                committedStateRef(ii, 0) = (float)referencePosition(0);
                committedStateRef(ii, 1) = (float)referencePosition(1);
                committedStateRef(ii, 2) = (float)referencePosition(2);

                committedStateRef(ii, 3) = (float)attitudeQuaternion(0);
                committedStateRef(ii, 4) = (float)attitudeQuaternion(1);
                committedStateRef(ii, 5) = (float)attitudeQuaternion(2);
                committedStateRef(ii, 6) = (float)attitudeQuaternion(3);
                
                committedStateRef(ii, 7) = (float)referenceVelocity(0);
                committedStateRef(ii, 8) = (float)referenceVelocity(1);
                committedStateRef(ii, 9) = (float)referenceVelocity(2);
                
                committedStateRef(ii, 10) = (float)controlInput(0);
                // committedStateRef(ii, 11) = (float)controlInput(1);
                // committedStateRef(ii, 12) = (float)controlInput(2);
                // committedStateRef(ii, 13) = (float)controlInput(3);
            }

            fillRawBuffer<Eigen::Matrix<float, N, NY>>(refState, committedStateRef, 1);
            fillRawBuffer<Eigen::Matrix<float, 1, NYN>>(refEndState, committedStateRef.block(N-1, 0, 1, NYN), 1);

            if(use_lp_filter){
                lpReset(pp_recovery, vv_recovery, aa_recovery, jj_recovery, yy_recovery, ww_recovery);
            }

            break;
        }

        default:{
            ROS_ERROR("[REGULATOR]: Action Not Implemented Yet!!");
            break;
        }

        // mtx.unlock();
    }
}

void MpcRegulator::dynamicInversion(Eigen::Matrix<float, 1, 3> p,
                                    Eigen::Matrix<float, 1, 3> v,
                                    Eigen::Matrix<float, 1, 3> a,
                                    Eigen::Matrix<float, 1, 3> j,
                                    float yaw, float w,
                                    Eigen::Matrix<float, 1, 4>& q,
                                    Eigen::Matrix<float, 1, 4>& u){
    
    Eigen::Matrix<float, 1, 3> zb = a + Eigen::Matrix<float, 1, 3>(0.0, 0.0, g);
    u(0) = zb.norm();
    zb = zb.normalized();

    Eigen::Matrix<float, 1, 3> xc = Eigen::Matrix<float, 1, 3>(cos(yaw), sin(yaw), 0.0);
    Eigen::Matrix<float, 1, 3> yb = (zb.cross(xc)).normalized();
    Eigen::Matrix<float, 1, 3> xb = yb.cross(zb);

    // Recover Quaternion
    // We have R = [xb' yb' zb']
    tf2::Matrix3x3 m(xb(0), yb(0), zb(0), xb(1), yb(1), zb(1), xb(2), yb(2), zb(2));
    tf2::Quaternion qReference;
    m.getRotation(qReference);

    q(0) = qReference.w();
    q(1) = qReference.x();
    q(2) = qReference.y();
    q(3) = qReference.z();

    // Check if Estimated and Reference Quaternion Live in the Same Hemisphere
    Eigen::Matrix<float, 1, 4> estimatedQuaternion =  fullEstState.block(0,3,1,4);
    if(estimatedQuaternion.dot(q) < 0.0){
        fullEstState.block(0,3,1,4) = -fullEstState.block(0,3,1,4);
    }

    // Compute Omega
    // Use Actual Orientation for Compute Omega
    tf2::Quaternion qActual(fullEstState(4), fullEstState(5), fullEstState(6), fullEstState(3));
    tf2::Matrix3x3 mActual(qActual);
    tf2::Vector3 xbActual = m.getColumn(0);
    tf2::Vector3 ybActual = m.getColumn(1);
    tf2::Vector3 zbActual = m.getColumn(2);

    Eigen::Matrix<float, 1, 3> xb_ = Eigen::Matrix<float, 1, 3>(xbActual[0], xbActual[1], xbActual[2]);
    Eigen::Matrix<float, 1, 3> yb_ = Eigen::Matrix<float, 1, 3>(ybActual[0], ybActual[1], ybActual[2]);
    Eigen::Matrix<float, 1, 3> zb_ = Eigen::Matrix<float, 1, 3>(zbActual[0], zbActual[1], zbActual[2]);

    Eigen::Matrix<float, 1, 3> h = (j - (zb_.dot(j) * zb_))/u(0);

    u(1) = -h.dot(yb_);
    u(2) = h.dot(xb_);
    u(3) = w*zb_(2);
}

void MpcRegulator::sendControl(const ros::TimerEvent& e){
    #ifdef MAVROS
    controlAction.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    controlAction.header.stamp = ros::Time::now();

    if(currentState.mode != "OFFBOARD"){
        reference_valid = false;
        position_valid = false;
        velocity_valid = false;
        quaternion_valid = false;
    }

    bool stopControl = !currentState.armed || currentState.mode != "OFFBOARD" ||
                       !position_valid || !velocity_valid || !quaternion_valid ||
                       !reference_valid;
    #endif
    #ifdef FLIGHTMARE
    controlAction.control_mode = quadrotor_msgs::ControlCommand::BODY_RATES;
    controlAction.armed = true;
    controlAction.expected_execution_time = ros::Time::now();
    controlAction.header.stamp = ros::Time::now();

    bool stopControl = !odom_valid || !reference_valid;
    #endif

    if(stopControl){
        // Reset ACADO Variables and Start Again to Solve From Hovering
        fillRawBuffer<Eigen::Matrix<float, 1, NX>>(initState, hoverState, 1);
        fillRawBuffer<Eigen::Matrix<float, 1, NX>>(state, hoverState, N+1);
        fillRawBuffer<Eigen::Matrix<float, 1, NU>>(input, hoverInput, N);
        fillRawBuffer<Eigen::Matrix<float, 1, NY>>(refState, hoverRefState, N);
        fillRawBuffer<Eigen::Matrix<float, 1, NX>>(refEndState, hoverState, 1);
        actualError = Eigen::MatrixXf::Zero(1, 3);

        #ifdef MAVROS
        controlAction.thrust = 0.2;
        controlAction.body_rate.x = 0.0;
        controlAction.body_rate.y = 0.0;
        controlAction.body_rate.z = 0.0;
        #endif
        #ifdef FLIGHTMARE
        controlAction.collective_thrust = 0.2;
        geometry_msgs::Vector3 br;
        br.x = 0.0;
        br.y = 0.0;
        br.x = 0.0;
        controlAction.bodyrates = br;
        #endif
    } else{
        run();
    }
    
    controlAction.header.seq = seq;
    controlAction.header.stamp = ros::Time::now();
    controlPub.publish(controlAction);
    seq ++;
}

void MpcRegulator::updateReference(const ros::TimerEvent& e){
    #ifdef MAVROS
    if(!position_valid || !velocity_valid || !quaternion_valid){
    #endif
    #ifdef FLIGHTMARE
    if(!odom_valid){
    #endif
        return;
    }

    if((ros::Time::now() - last_refRotation).toSec() < 1.0){
        return;
    }

    rotateReference(false);
}

void MpcRegulator::checkRef_pp(Eigen::Matrix<float, 1, 3>& pp){
    if(!use_lp_filter){
        return;
    }

    for(uint ii = 0; ii < 3; ii++){
        pp(ii) = lp_pp[ii]->update(pp(ii));
    }
}
void MpcRegulator::checkRef_vv(Eigen::Matrix<float, 1, 3>& vv){
    if(!use_lp_filter){
        return;
    }

    for(uint ii = 0; ii < 3; ii++){
        vv(ii) = lp_vv[ii]->update(vv(ii));
    }
}

void MpcRegulator::checkRef_aa(Eigen::Matrix<float, 1, 3>& aa){
    if(!use_lp_filter){
        return;
    }

    for(uint ii = 0; ii < 3; ii++){
        aa(ii) = lp_aa[ii]->update(aa(ii));
    }
}

void MpcRegulator::checkRef_jj(Eigen::Matrix<float, 1, 3>& jj){
    if(!use_lp_filter){
        return;
    }

    for(uint ii = 0; ii < 3; ii++){
        jj(ii) = lp_jj[ii]->update(jj(ii));
    }
}

void MpcRegulator::checkRef_yy(float& yy){
    if(!use_lp_filter){
        return;
    }

    yy = lp_yy[0]->update(yy);
}

void MpcRegulator::checkRef_ww(float& ww){
    if(!use_lp_filter){
        return;
    }

    ww = lp_ww[0]->update(ww);
}

void MpcRegulator::lpReset(Eigen::Matrix<float, 1, 3> pp_recovery,
                           Eigen::Matrix<float, 1, 3> vv_recovery,
                           Eigen::Matrix<float, 1, 3> aa_recovery,
                           Eigen::Matrix<float, 1, 3> jj_recovery,
                           float yy_recovery, float ww_recovery){
    
    for(uint ii = 0; ii < 3; ii++){
        lp_pp[ii]->setInitialCondition(pp_recovery(ii));
    }
    for(uint ii = 0; ii < 3; ii++){
        lp_vv[ii]->setInitialCondition(vv_recovery(ii));
    }
    for(uint ii = 0; ii < 3; ii++){
        lp_aa[ii]->setInitialCondition(aa_recovery(ii));
    }
    for(uint ii = 0; ii < 3; ii++){
        lp_jj[ii]->setInitialCondition(jj_recovery(ii));
    }

    lp_yy[0]->setInitialCondition(yy_recovery);
    lp_ww[0]->setInitialCondition(ww_recovery);
}

// Callback Functions
#ifdef MAVROS
void MpcRegulator::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    fullEstState.block(0,0,1,3) = Eigen::Matrix<float, 1, 3>(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    position_valid = true;
}

void MpcRegulator::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    fullEstState.block(0,7,1,3) = Eigen::Matrix<float, 1, 3>(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);;
    velocity_valid = true;
}

void MpcRegulator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    fullEstState.block(0,3,1,4) = Eigen::Matrix<float, 1, 4>(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

    // Canonalize Quaternion
    // if(fullEstState(3) < 0.0){
    //     fullEstState.block(0,3,1,4) = -fullEstState.block(0,3,1,4);
    // }

    quaternion_valid = true;
}
#endif
#ifdef FLIGHTMARE
void MpcRegulator::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    tf2::Vector3 v_translation(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    tf2::Quaternion v_quaternion(0,0,0,1);
    tf2::Transform v_transformation(v_quaternion, v_translation);
    tf2::Vector3 v2_translation(0,0,0);
    tf2::Quaternion v2_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Transform v2_transformation(v2_quaternion, v2_translation);

    tf2::Transform vv_transformation = v2_transformation*v_transformation;
    tf2::Vector3 vv_translation = vv_transformation.getOrigin();
    
    fullEstState.block(0,0,1,3) = Eigen::Matrix<float, 1, 3>(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    fullEstState.block(0,7,1,3) = Eigen::Matrix<float, 1, 3>(vv_translation.x(), vv_translation.y(), vv_translation.z());
    fullEstState.block(0,3,1,4) = Eigen::Matrix<float, 1, 4>(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    
    // Canonalize Quaternion
    if(fullEstState(3) < 0.0){
        fullEstState.block(0,3,1,4) = -fullEstState.block(0,3,1,4);
    }

    odom_valid = true;
}
#endif

void MpcRegulator::setPointCallback(const mav_regulator::SetPoint::ConstPtr& msg){
    #ifdef MAVROS
    if(!position_valid || !velocity_valid || !quaternion_valid){
    #endif
    #ifdef FLIGHTMARE
    if(!odom_valid){
    #endif
        ROS_ERROR("[REGULATOR]: Rejecting Reference, No Odometry Yet");
        return;
    }

    if(msg->reset){
        reset();
    }

    // mtx.lock();
    positionReference = Eigen::Matrix<float, 1, 3>(msg->x, msg->y, msg->z);
    yawReference = msg->yaw;
    executionTime = msg->time;
    actualFrame = msg->frame;

    committedTime = (ros::Time::now()).toSec();

    tf2::Quaternion q(fullEstState(4), fullEstState(5), fullEstState(6), fullEstState(3));
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    positionInitial = fullEstState.block(0,0,1,3);
    yawInitial = yaw;

    // todo: adjust yaw setpoint
    reference_valid = true;
    startTime = 0.0;

    switch(msg->action){
        case(mav_regulator::SetPoint::USE_POSITION):{
            actualReference = USE_POSITION;
            break;
        }

        case(mav_regulator::SetPoint::USE_VELOCITY):{
            actualReference = USE_VELOCITY;
            break;
        }

        case(mav_regulator::SetPoint::USE_ACCELERATION):{
            actualReference = USE_ACCELERATION;
            break;
        }

        case(mav_regulator::SetPoint::USE_MIXED):{
            actualReference = USE_MIXED;
            break;
        }

        default:{
            actualReference = USE_POSITION;
            break;
        }
    }

    if(use_lp_filter){
        Eigen::Matrix<float, 1, 4> q = fullEstState.block(0,3,1,4);
        tf2::Quaternion qq(q(1), q(2), q(3), q(0));
        tf2::Matrix3x3 m(qq);

	    double r, p, y;
	    m.getRPY(r,p,y);

        Eigen::Matrix<float, 1, 3> pp = fullEstState.block(0,0,1,3);
        Eigen::Matrix<float, 1, 3> vv = fullEstState.block(0,7,1,3);
        Eigen::Matrix<float, 1, 3> zero = Eigen::MatrixXf::Zero(1,3);
        lpReset(pp, vv, zero, zero, y, 0.0);
    }

    // mtx.unlock();
}

void MpcRegulator::trajectoryCallback(const mav_regulator::BezierMultiArray::ConstPtr& msg){
    #ifdef MAVROS
    if(!position_valid || !velocity_valid || !quaternion_valid){
    #endif
    #ifdef FLIGHTMARE
    if(!odom_valid){
    #endif
        ROS_ERROR("[REGULATOR]: Rejecting Reference, No Odometry Yet");
        return;
    }

    // mtx.lock();

    if(msg->action == mav_regulator::BezierMultiArray::REPLACE || actualReference != USE_TRAJECTORY){
        position_controlPoints.clear();
        orientation_controlPoints.clear();
        time_controlPoints.clear();
        actualFrames.clear();
        executionTimes.clear();
        is_timeScaled.clear();
        is_invertedTime.clear();
        is_yawFollowing.clear();

        position_controlPoints.shrink_to_fit();
        orientation_controlPoints.shrink_to_fit();
        time_controlPoints.shrink_to_fit();
        actualFrames.shrink_to_fit();
        executionTimes.shrink_to_fit();
        is_timeScaled.shrink_to_fit();
        is_invertedTime.shrink_to_fit();
        is_yawFollowing.shrink_to_fit();

        committedTime = (ros::Time::now()).toSec();
        startTime = 0.0;

        if(use_lp_filter){
            Eigen::Matrix<float, 1, 4> q = fullEstState.block(0,3,1,4);
            tf2::Quaternion qq(q(1), q(2), q(3), q(0));
            tf2::Matrix3x3 m(qq);

            double r, p, y;
            m.getRPY(r,p,y);

            Eigen::Matrix<float, 1, 3> pp = fullEstState.block(0,0,1,3);
            Eigen::Matrix<float, 1, 3> vv = fullEstState.block(0,7,1,3);
            Eigen::Matrix<float, 1, 3> zero = Eigen::MatrixXf::Zero(1,3);
            lpReset(pp, vv, zero, zero, y, 0.0);
        }
    }

    for(uint ii = 0; ii < msg->trajectory.size(); ii++){
        std::vector<Eigen::Matrix<float, 1, 3>> position_controlPoints_accumulator;
        std::vector<float> orientation_controlPoints_accumulator;
        std::vector<float> time_controlPoints_accumulator;

        for(uint jj = 0; jj < msg->trajectory[ii].x.size(); jj++){
            Eigen::Matrix<float, 1, 3> actualPosition = Eigen::Matrix<float, 1, 3>(msg->trajectory[ii].x[jj],
                                                                                   msg->trajectory[ii].y[jj],
                                                                                   msg->trajectory[ii].z[jj]);
            position_controlPoints_accumulator.push_back(actualPosition);
        }

        if(position_controlPoints_accumulator.size() < 8){
            ROS_WARN("[REGULATOR]: Provided Trajectory Order Less Than 7");
        }

        for(uint jj = 0; jj < msg->trajectory[ii].yaw.size(); jj++){
            orientation_controlPoints_accumulator.push_back(msg->trajectory[ii].yaw[jj]);
        }

        for(uint jj = 0; jj < msg->trajectory[ii].u.size(); jj++){
            time_controlPoints_accumulator.push_back(msg->trajectory[ii].u[jj]);
        }

        position_controlPoints.push_back(position_controlPoints_accumulator);
        orientation_controlPoints.push_back(orientation_controlPoints_accumulator);
        time_controlPoints.push_back(time_controlPoints_accumulator);

        actualFrames.push_back(msg->trajectory[ii].frame);
        executionTimes.push_back(msg->trajectory[ii].totalTime);
        is_timeScaled.push_back(msg->trajectory[ii].is_timeScaled);
        is_invertedTime.push_back(msg->trajectory[ii].is_timeInverted);
        is_yawFollowing.push_back(msg->trajectory[ii].is_yawFollowing);
    }

    rotateReference(true);

    position_controlPoints_actual = position_controlPoints_rotated;
    orientation_controlPoints_actual = orientation_controlPoints_rotated;

    actualReference = USE_TRAJECTORY;
    reference_valid = true;
    // mtx.unlock();
}

void MpcRegulator::splineCallback(const mav_regulator::BSpline::ConstPtr& msg){
    if(is_calling){
        return;
    }

    is_calling = true;

    #ifdef MAVROS
    if(!position_valid || !velocity_valid || !quaternion_valid){
    #endif
    #ifdef FLIGHTMARE
    if(!odom_valid){
    #endif
        ROS_ERROR("[REGULATOR]: Rejecting Reference, No Odometry Yet");
        is_calling = false;
        return;
    }

    if(msg->reset){
        reset();
    }

    // mtx.lock();

    std::vector<std::vector<float>> p_spline_knots_new;
    std::vector<std::vector<float>> y_spline_knots_new;
    std::vector<std::vector<Eigen::Matrix<float, 1, 3>>> position_controlPoints_new;
    std::vector<std::vector<float>> orientation_controlPoints_new;
    std::vector<std::string> actualFrames_new;
    std::vector<bool> is_yawFollowing_new;

    if(msg->action == mav_regulator::BSpline::ADD && actualReference == USE_SPLINE){
        p_spline_knots_new = p_spline_knots;
        y_spline_knots_new = y_spline_knots;
        position_controlPoints_new = position_controlPoints;
        orientation_controlPoints_new = orientation_controlPoints;
        actualFrames_new = actualFrames;
        is_yawFollowing_new = is_yawFollowing;

    } else{
        if(use_lp_filter){
            Eigen::Matrix<float, 1, 4> q = fullEstState.block(0,3,1,4);
            tf2::Quaternion qq(q(1), q(2), q(3), q(0));
            tf2::Matrix3x3 m(qq);

            double r, p, y;
            m.getRPY(r,p,y);

            Eigen::Matrix<float, 1, 3> pp = fullEstState.block(0,0,1,3);
            Eigen::Matrix<float, 1, 3> vv = fullEstState.block(0,7,1,3);
            Eigen::Matrix<float, 1, 3> zero = Eigen::MatrixXf::Zero(1,3);
            lpReset(pp, vv, zero, zero, y, 0.0);
        }

        if(msg->action != mav_regulator::BSpline::REPLANNED){
            committedTime = (ros::Time::now()).toSec();
        }
    }

    std::vector<Eigen::Matrix<float, 1, 3>> position_controlPoints_accumulator;
    std::vector<float> orientation_controlPoints_accumulator;
    std::vector<float> p_knots_accumulator;
    std::vector<float> y_knots_accumulator;
    for(uint ii = 0; ii < msg->p_controlPoints.size(); ii++){
        Eigen::Matrix<float, 1, 3> actualPosition = Eigen::Matrix<float, 1, 3>(msg->p_controlPoints[ii].x,
                                                                               msg->p_controlPoints[ii].y,
                                                                               msg->p_controlPoints[ii].z);
        position_controlPoints_accumulator.push_back(actualPosition);                                                                          
    }
    for(uint ii = 0; ii < msg->p_knots.size(); ii++){
        p_knots_accumulator.push_back(msg->p_knots[ii]);
    }
    for(uint ii = 0; ii < msg->y_controlPoints.size(); ii++){
        orientation_controlPoints_accumulator.push_back(msg->y_controlPoints[ii]);
    }
    for(uint ii = 0; ii < msg->y_knots.size(); ii++){
        y_knots_accumulator.push_back(msg->y_knots[ii]);
    }

    // Check Spline Order
    int np = p_knots_accumulator.size() - position_controlPoints_accumulator.size() - 1;
    int ny = y_knots_accumulator.size() - orientation_controlPoints_accumulator.size() - 1;

    if(np != spline_order){
        ROS_ERROR("[REGULATOR]: Error in Spline Order");
    }

    if(orientation_controlPoints_accumulator.size() > 0 && ny != spline_order_yaw){
        ROS_ERROR("[REGULATOR]: Error in Spline Order - Yaw");
    }

    position_controlPoints_new.push_back(position_controlPoints_accumulator);
    p_spline_knots_new.push_back(p_knots_accumulator);
    orientation_controlPoints_new.push_back(orientation_controlPoints_accumulator);
    y_spline_knots_new.push_back(y_knots_accumulator);
    actualFrames_new.push_back(msg->frame);
    is_yawFollowing_new.push_back(msg->is_yawFollowing);

    // Cloning Computed Reference to Vectors Used by Regulator
    while(is_rotating){
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    position_controlPoints = position_controlPoints_new;
    orientation_controlPoints = orientation_controlPoints_new;
    p_spline_knots = p_spline_knots_new;
    y_spline_knots = y_spline_knots_new;
    actualFrames = actualFrames_new;
    is_yawFollowing = is_yawFollowing_new;

    if(msg->action != mav_regulator::BSpline::REPLANNED){
        rotateReference(true);
        position_controlPoints_actual = position_controlPoints_rotated;
        orientation_controlPoints_actual = orientation_controlPoints_rotated;
        p_spline_knots_actual = p_spline_knots_rotated;
        y_spline_knots_actual = y_spline_knots_rotated;
        is_yawFollowing_actual = is_yawFollowing_rotated;

    } else{
        rotateReference(false);
    }

    actualReference = USE_SPLINE;
    reference_valid = true;
    startTime = 0.0;

    // Compute Real Start-Time During Replanning
    // if(msg->action == mav_regulator::BSpline::REPLANNED){
    //     // We need to Set committedTime Accordingly
    //     // Search Close to the actual committedTime

    //     // Search Optimal Time
    //     double dt_search = 5.0;
    //     double min_dist = INFINITY;
    //     double committedTime_opt = 0.0;

    //     Eigen::Matrix<float, 1, 3> referencePosition;
    //     for(double tt = -dt_search; tt <= dt_search; tt += 0.05){
    //         double committedTime_tt = committedTime + tt;
    //         committedTime_tt = committedTime_tt < 0.0 ? 0.0 : committedTime_tt;

    //         double actual_tt = abs((ros::Time::now()).toSec() - committedTime_tt);

    //         referencePosition = evaluateBSpline<Eigen::Matrix<float, 1, 3>>(position_controlPoints_rotated[0], p_spline_knots[0], actual_tt, spline_order);
    //         double dd = (referencePosition-fullEstState.block(0,0,1,3)).squaredNorm();

    //         if(dd < min_dist){
    //             min_dist = dd;
    //             committedTime_opt = committedTime_tt;
    //         }
    //     }

    //     committedTime = committedTime_opt;
    // }

    // mtx.unlock();

    is_calling = false;
}

void MpcRegulator::rotateReference(bool blocking){
    if(is_rotating && !blocking){
        return;
    }

    if(blocking){
        while(is_rotating){
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    is_rotating = true;

    position_controlPoints_rotated.clear();
    position_controlPoints_rotated.shrink_to_fit();
    orientation_controlPoints_rotated.clear();
    orientation_controlPoints_rotated.shrink_to_fit();

    if(actualFrames.size() == 0){
        is_rotating = false;
        return;
    }

    position_controlPoints_rotated = position_controlPoints;
    orientation_controlPoints_rotated = orientation_controlPoints;

    for(uint ii = 0; ii < actualFrames.size(); ii++){
        if(actualFrames[ii] == "odom"){
            position_controlPoints_rotated[ii] = position_controlPoints[ii];
            orientation_controlPoints_rotated[ii] = orientation_controlPoints[ii];

        } else if(actualFrames[ii] == "map"){
            // Rotate it In ODOM Coordinate
            if(position_controlPoints[ii].size() != 0){
                for(uint kk = 0; kk < position_controlPoints[ii].size(); kk++){
                    Eigen::Matrix<float, 1, 3> cp = position_controlPoints[ii][kk];

                    tf2::Vector3 mapToRef_translation(cp(0), cp(1), cp(2));
                    tf2::Quaternion mapToRef_quaternion(0.0, 0.0, 0.0, 1.0);

                    tf2::Transform mapToRef_transformation(mapToRef_quaternion, mapToRef_translation);
                    tf2::Transform odomToRef_transformation = (mapToRef_transformation.inverse()*mapToOdom_transformation).inverse();
                    tf2::Vector3 odomToRef_translation = odomToRef_transformation.getOrigin();

                    cp(0) = odomToRef_translation.x();
                    cp(1) = odomToRef_translation.y();
                    cp(2) = odomToRef_translation.z();

                    position_controlPoints_rotated[ii][kk] = cp;

                }
            }

            if(orientation_controlPoints[ii].size() != 0){
                for(uint kk = 0; kk < orientation_controlPoints[ii].size(); kk++){
                    // tf2::Vector3 mapToRef_translation(0.0, 0.0, 0.0);
                    // tf2::Quaternion mapToRef_quaternion;
                    // mapToRef_quaternion.setRPY(0.0, 0.0, orientation_controlPoints[ii][kk]);

                    // tf2::Transform mapToRef_transformation(mapToRef_quaternion, mapToRef_translation);
                    // tf2::Transform odomToRef_transformation = (mapToRef_transformation.inverse()*mapToOdom_transformation).inverse();
                    // tf2::Quaternion odomToRef_quaternion = odomToRef_transformation.getRotation();

                    // tf2::Matrix3x3 m(odomToRef_quaternion);
                    // double roll, pitch, yaw;
                    // m.getRPY(roll, pitch, yaw);

                    tf2::Quaternion mapToOdom_quaternion = mapToOdom_transformation.getRotation();
                    tf2::Matrix3x3 m(mapToOdom_quaternion);
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);

                    orientation_controlPoints_rotated[ii][kk] = orientation_controlPoints[ii][kk] - yaw;
                }
            }
        } else{
            ROS_ERROR("[Regulator]: Unknown Reference Frame!!");
        }
    }

    // Save Knots
    y_spline_knots_rotated.clear();
    y_spline_knots_rotated.shrink_to_fit();

    p_spline_knots_rotated.clear();
    p_spline_knots_rotated.shrink_to_fit();

    y_spline_knots_rotated = y_spline_knots;
    p_spline_knots_rotated = p_spline_knots;

    is_yawFollowing_rotated.clear();
    is_yawFollowing_rotated.shrink_to_fit();

    is_yawFollowing_rotated = is_yawFollowing;

    last_refRotation = ros::Time::now();
    is_rotating = false;
}

#ifdef MAVROS
void MpcRegulator::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}
#endif

void MpcRegulator::reset(){
    actualError = Eigen::MatrixXf::Zero(1, 3);
}