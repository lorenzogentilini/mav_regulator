#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>

int main(){
  USING_NAMESPACE_ACADO
  const bool CODE_GEN = true;

  // System variables
  DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  OnlineData            d_x, d_y, d_z;
  Control               T, w_x, w_y, w_z;
  DifferentialEquation  f;

  Function              h, hN;
  
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 1.5;       // Time horizon [s]
  const double dt = 0.05;         // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]

  // Constraints Reset Online
  const double w_max_yaw = 3;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 6;      // Maximal pitch and roll rate [rad/s]
  const double maxThrust = 20;    // Maximal thrust [N]
  const double minThrust = 2;     // Minimal thrust [N]

  // System Dynamics
  f << dot(p_x) == v_x;
  f << dot(p_y) == v_y;
  f << dot(p_z) == v_z;
  f << dot(q_w) == 0.5*(-w_x*q_x - w_y*q_y - w_z*q_z);
  f << dot(q_x) == 0.5*(w_x*q_w + w_z*q_y - w_y*q_z);
  f << dot(q_y) == 0.5*(w_y*q_w - w_z*q_x + w_x*q_z);
  f << dot(q_z) == 0.5*(w_z*q_w + w_y*q_x - w_x*q_y);
  f << dot(v_x) == 2*(q_w*q_y + q_x*q_z)*T + d_x;
  f << dot(v_y) == 2*(q_y*q_z - q_w*q_x)*T + d_y;
  f << dot(v_z) == (1 - 2*q_x*q_x - 2*q_y*q_y)*T - g_z + d_z;

  // Cost Function
  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << T << w_x << w_y << w_z;

  // End Cost Function
  hN << p_x << p_y << p_z
     << q_w << q_x << q_y << q_z
     << v_x << v_y << v_z;

  // Cost Q Matrix
  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 200;    // x
  Q(1,1) = 200;    // y
  Q(2,2) = 500;    // z
  Q(3,3) = 100;    // qw
  Q(4,4) = 100;    // qx
  Q(5,5) = 100;    // qy
  Q(6,6) = 100;    // qz
  Q(7,7) = 100;    // vx
  Q(8,8) = 100;    // vy
  Q(9,9) = 100;    // vz
  Q(10,10) = 10;   // T
  Q(11,11) = 10;   // wx
  Q(12,12) = 10;   // wy
  Q(13,13) = 10;   // wz

  DMatrix QN(hN.getDim(), hN.getDim());
  QN.setIdentity();
  QN(0,0) = Q(0,0);   // x
  QN(1,1) = Q(1,1);   // y
  QN(2,2) = Q(2,2);   // z
  QN(3,3) = Q(3,3);   // qw
  QN(4,4) = Q(4,4);   // qx
  QN(5,5) = Q(5,5);   // qy
  QN(6,6) = Q(6,6);   // qz
  QN(7,7) = Q(7,7);   // vx
  QN(8,8) = Q(8,8);   // vy
  QN(9,9) = Q(9,9);   // vz

  // Set a reference for the analysis (if CODE_GEN is false).
  DVector r(h.getDim());
  r.setZero();
  r(0) = 2.0;
  r(1) = 4.0;
  r(2) = 1.0;
  r(3) = 1.0;

  DVector rN(hN.getDim());
  rN.setZero();
  rN(0) = r(0);
  rN(1) = r(1);
  rN(2) = r(2);
  rN(3) = r(3);

  // DEFINE AN OPTIMAL CONTROL PROBLEM:
  OCP ocp(t_start, t_end, N);
  if(!CODE_GEN){
    // For analysis, set references.
    ocp.minimizeLSQ(Q, h, r);
    ocp.minimizeLSQEndTerm(QN, hN, rN);

  } else{
    // For code generation, references are set during run time.
    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();

    ocp.minimizeLSQ(Q_sparse, h);
    ocp.minimizeLSQEndTerm(QN_sparse, hN);
  }

  // Add system dynamics
  ocp.subjectTo(f);

  // Add constraints
  ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo(minThrust <= T <= maxThrust);

  if(!CODE_GEN){
    // Set initial state
    ocp.subjectTo(AT_START, p_x ==  0.0);
    ocp.subjectTo(AT_START, p_y ==  0.0);
    ocp.subjectTo(AT_START, p_z ==  0.0);
    ocp.subjectTo(AT_START, q_w ==  1.0);
    ocp.subjectTo(AT_START, q_x ==  0.0);
    ocp.subjectTo(AT_START, q_y ==  0.0);
    ocp.subjectTo(AT_START, q_z ==  0.0);
    ocp.subjectTo(AT_START, v_x ==  0.0);
    ocp.subjectTo(AT_START, v_y ==  0.0);
    ocp.subjectTo(AT_START, v_z ==  0.0);
    ocp.subjectTo(AT_START, w_x ==  0.0);
    ocp.subjectTo(AT_START, w_y ==  0.0);
    ocp.subjectTo(AT_START, w_z ==  0.0);
    ocp.subjectTo(AT_START, d_x ==  0.0);
    ocp.subjectTo(AT_START, d_y ==  0.0);
    ocp.subjectTo(AT_START, d_z ==  0.0);

    // Setup some visualization
    GnuplotWindow stateWindow(PLOT_AT_EACH_ITERATION);
    stateWindow.addSubplot(p_x,"position x");
    stateWindow.addSubplot(p_y,"position y");
    stateWindow.addSubplot(p_z,"position z");
    stateWindow.addSubplot(v_x,"verlocity x");
    stateWindow.addSubplot(v_y,"verlocity y");
    stateWindow.addSubplot(v_z,"verlocity z");

    GnuplotWindow inputWindow(PLOT_AT_EACH_ITERATION);
    inputWindow.addSubplot(w_x,"omega x");
    inputWindow.addSubplot(w_y,"omega y");
    inputWindow.addSubplot(w_z,"omega z"); 
    inputWindow.addSubplot(T,"Thrust");


    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set(INTEGRATOR_TOLERANCE, 1e-6);
    algorithm.set(KKT_TOLERANCE, 1e-3);
    algorithm << stateWindow;
    algorithm << inputWindow;
    algorithm.solve();

  }else{
    // For code generation, we can set some properties.
    OCPexport mpc(ocp);
    mpc.set(HESSIAN_APPROXIMATION,            GAUSS_NEWTON);        // is robust, stable
    mpc.set(DISCRETIZATION_TYPE,              MULTIPLE_SHOOTING);   // good convergence
    mpc.set(SPARSE_QP_SOLUTION,               FULL_CONDENSING_N2);  // due to qpOASES
    mpc.set(INTEGRATOR_TYPE,                  INT_IRK_GL4);         // accurate
    mpc.set(NUM_INTEGRATOR_STEPS,             N);
    mpc.set(QP_SOLVER,                        QP_QPOASES);          // free, source code
    mpc.set(HOTSTART_QP,                      YES);
    mpc.set(CG_USE_OPENMP,                    YES);                 // paralellization
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);                  // set on runtime
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);                 // time-varying costs
    mpc.set(USE_SINGLE_PRECISION,             YES);                 // single precision

    // Do not generate tests, makes or matlab-related interfaces.
    mpc.set(GENERATE_TEST_FILE,          NO);
    mpc.set(GENERATE_MAKE_FILE,          NO);
    mpc.set(GENERATE_MATLAB_INTERFACE,   NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Finally, export everything.
    if(mpc.exportCode("quadrotor_mpc_model_codegen") != SUCCESSFUL_RETURN)
      exit(EXIT_FAILURE);
      
    mpc.printDimensionsQP( );
  }

  return EXIT_SUCCESS;
}