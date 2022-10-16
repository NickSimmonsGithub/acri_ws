// ControlSystem.cpp
// Written by: Nicholas Simmons, C3330381
// Last modified: 24/09/2022
//
// This file (along with ControlSystem.h) implements the ACRI trackside robotic device's observer and controller modules.
// It also implements a full state space representation of the tank for debugging.

#include <string>
#include <Eigen/Core>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <ros/console.h>
#include <boost/numeric/odeint.hpp>
#include "ControlSystem.h"
#include "ros/ros.h"
#include "acri_controller/State.h"
#include "acri_controller/System.h"
#include "acri_controller/Control.h"
#include "acri_controller/ObserverInit.h"
#include "riccati_solver.h"

namespace odeint = boost::numeric::odeint;

// ### ODE WRAPPER. NOT PART OF CONTROLSYSTEM CLASS ###

//[ ode_wrapper
template< class Obj , class Mem >
class ode_wrapper
{
    Obj m_obj;
    Mem m_mem;

public:

    ode_wrapper( Obj obj , Mem mem ) : m_obj( obj ) , m_mem( mem ) { }

    template< class State , class Deriv , class Time >
    void operator()( const State &x , Deriv &dxdt , Time t )
    {
        (m_obj.*m_mem)( x , dxdt , t );
    }
};

template< class Obj , class Mem >
ode_wrapper< Obj , Mem > make_ode_wrapper( Obj obj , Mem mem )
{
    return ode_wrapper< Obj , Mem >( obj , mem );
}
//]



// ### CONSTRUCTORS ###

ControlSystem::ControlSystem(double _v)
{
    // Sets the velocity member only. Used for odeint.
    v = _v;
}

ControlSystem::ControlSystem(int argc, char** argv, Eigen::Vector3d & x_init, double theta_bar)
{

    ros::init(argc, argv, "controlSystem");

    // Calculate the remaining equillibrium point members from theta_bar.
    xbarCalc(0);

    // Calculate A, B, C, G and H.
    lineariseSystem();

    // Insert initial state to the state vector.
    x     = x_init;
    x_obs = x_init;

    // Set the initial reference point at theta = 0.
    setReference(-0.2);

    // Construct all MPC matrices.
    initialiseMPC();

    // Construct the linear observer.
    InitialiseObserver();
}


ControlSystem::ControlSystem(int argc, char** argv, Eigen::Vector3d & x_init, Eigen::Vector3d & x_obs_init, double theta_bar)
{

}


// ### DESTRUCTOR ###

ControlSystem::~ControlSystem()
{
    // Empty destructor
}


// ### LINEARISATION PUBLIC INTERFACE ###

void ControlSystem::xbarCalc(double theta_bar)
{
    double z_bar = -d * std::tan(theta_bar);                // Calculate z_bar from theta_bar.
    x_bar << theta_bar,                                     // Construct the equillibrium point vector.
                     0,
                 z_bar;

}

void ControlSystem::lineariseSystem()
{
    // Check that the sample time is larger than zero.
    assert(dt > 0);

    // Extract the equillbrium point components from xbar.
    double theta_bar = x_bar(0);
    double z_bar     = x_bar(2);                       

    // Calculate A.
    double df1dx1 = 0.0;
    double df1dx2 = 1.0 / (m * (d*d + z_bar*z_bar) + J);
    double df1dx3 = 0.0;
    double df2dx1 = m * g * d * std::cos(theta_bar)  -  m * g * z_bar * std::sin(theta_bar);
    double df2dx2 = 0.0;
    double df2dx3 = m * g * std::cos(theta_bar);
    double df3dx1 = 0.0;
    double df3dx2 = 0.0;
    double df3dx3 = 0.0;

    A << df1dx1, df1dx2, df1dx3,
         df2dx1, df2dx2, df2dx3,
         df3dx1, df3dx2, df3dx3;

    // Calculate B.
    double df1du = -(m * d)/(m * (d*d + z_bar*z_bar) + J);
    double df2du = 0.0;
    double df3du = 1.0;

    B << df1du,
         df2du,
         df3du;

    // Calculate C.
    C << 1, 0, 0;

}      


// ### CONTROLLER PUBLIC INTERFACE ###

void ControlSystem::setReference(double theta_ref)
{
    double z_ref = -d * std::tan(theta_ref);                // Calculate z_ref from theta_ref.
    x_ref << theta_ref,                                     // Construct the reference point vector.
                     0,
                 z_ref;
}

void ControlSystem::initialiseMPC()
{
    // Populate tuning parameters.
    double q1   =  1e7;
    double q2   =  1e-8;
    double q3   =  1e3;
    double q4   =  1e6;
    delta_u_max =  0.03;
    delta_u_min = -0.03;
    u_max       =  1.2;
    u_min       = -1.2;


    Q = Eigen::MatrixXd::Zero(4, 4);
    Q(0, 0) = q1;
    Q(1, 1) = q2;
    Q(2, 2) = q3;
    Q(3, 3) = q4;

    R << 1.0;

    // Augment A and B matrices for integral action.
    A_int << A, Eigen::VectorXd::Zero(3),
             C,                        0;
    B_int << B,
             D;

    // Define N for discretisation.
    N << 0,
         0,
         0,
         0;

    // Initialise integral action term.
    z << 0;

    // Discretise system.
    dlqr_cost();

    std::cout << "Ad: " << std::endl << Ad << std::endl;
    std::cout << "Bd: " << std::endl << Bd << std::endl;

    // Compute terminal cost. 
    solveRiccatiIterationD(Ad, Bd, Qd, Rd, Q_f, 1e-12, 100000);

    // Populate A_MPC, B_MPC, Q_MPC, R_MPC and N_MPC terms.
    int nx = A_int.cols();
    int nu = B_int.cols();
    A_MPC = Eigen::MatrixXd::Zero(nx*(N_horizon + 1),                 nx);
    B_MPC = Eigen::MatrixXd::Zero(nx*(N_horizon + 1),       N_horizon*nu);
    Q_MPC = Eigen::MatrixXd::Zero(nx*(N_horizon + 1), nx*(N_horizon + 1));
    R_MPC = Eigen::MatrixXd::Zero(      N_horizon*nu,       N_horizon*nu);
    N_MPC = Eigen::MatrixXd::Zero(nx*(N_horizon + 1),       N_horizon*nu);

    Eigen::MatrixXd cumulativePowTerm = Eigen::MatrixXd::Identity(nx, nx);
    A_MPC.block(           0,            0, nx, nx) = cumulativePowTerm;
    Q_MPC.block(nx*N_horizon, nx*N_horizon, nx, nx) = Q_f;
    for(int i = 1; i <= N_horizon; i++)
    {
        cumulativePowTerm = cumulativePowTerm * Ad;
        A_MPC.block(      i*nx,          0, nx, nx) = cumulativePowTerm;
        Q_MPC.block((i - 1)*nx, (i - 1)*nx, nx, nx) = Qd;
        R_MPC.block((i - 1)*nu, (i - 1)*nu, nu, nu) = Rd;
        for(int j = 1; j <= i; j++)
        {
            Eigen::MatrixXd powTerm = power(Ad, i, j);
            B_MPC.block(i*nx, (j - 1)*nu, nx, nu) = powTerm * Bd;
        }
    }

    // Populate H_MPC and fbar_MPC terms.
    H_MPC    = 2.0 * (B_MPC.transpose()*Q_MPC*B_MPC + B_MPC.transpose()*N_MPC + N_MPC.transpose()*B_MPC + R_MPC);
    H_MPC    = (H_MPC + H_MPC.transpose()) / 2.0;
    // H_MPC.transposeInPlace();
    // std::cout << "H_MPC: " << std::endl << H_MPC << std::endl;

    fbar_MPC = 2.0 * (B_MPC.transpose()*Q_MPC + N_MPC.transpose())*A_MPC;
    // std::cout << "fbar_MPC: " << std::endl << fbar_MPC << std::endl;

    // Populate the CONST_INEQUAL_A Term.
    int stepSize = 1;
    Eigen::MatrixXd DiagonalSubTerm = Eigen::MatrixXd::Zero(N_horizon, N_horizon);
    for(int i = 0; i < N_horizon - stepSize; i++)
    {
        DiagonalSubTerm(i + stepSize, i) = 1;
    }
    Eigen::MatrixXd W;
    W = Eigen::MatrixXd::Identity(N_horizon, N_horizon) - DiagonalSubTerm;
    CONST_INEQUAL_A.resize(2*N_horizon, N_horizon);
    CONST_INEQUAL_A <<  W,
                       -W;
    CONST_INEQUAL_A.transposeInPlace();

    // Populate the DeltaUMin and DeltaUMax matrices.
    DeltaUMax = Eigen::VectorXd::Ones(N_horizon) * delta_u_max;                 // Assumes u0 = 0.
    DeltaUMin = Eigen::VectorXd::Ones(N_horizon) * delta_u_min;
    CONST_INEQUAL_B.resize(2*N_horizon);
    CONST_INEQUAL_B <<  DeltaUMax,
                       -DeltaUMin;

    // Populate upper and lower bound matrices.
    CONST_BOUND_UPPER = Eigen::VectorXd::Ones(N_horizon) * u_max;
    CONST_BOUND_LOWER = Eigen::VectorXd::Ones(N_horizon) * u_min;

    // Populate all c-style matrices for qpas_sub_noblas.
    std::cout << "H: ";
    for(int i = 0; i < CTRL_N_HORIZON; i++)
    {
        std::cout << std::endl;
        for(int j = 0; j < CTRL_N_HORIZON; j++)
        {
            ctrl_H[CTRL_N_HORIZON*i + j] = H_MPC(i, j);
            std::cout << ctrl_H[CTRL_N_HORIZON*i + j] << ", ";
        }
    }

    std::cout << "A: ";
    for(int i = 0; i < CTRL_N_HORIZON; i++)
    {
        std::cout << std::endl;
        for(int j = 0; j < CTRL_N_INEQ_CONST; j++)
        {
            ctrl_A[CTRL_N_INEQ_CONST*i + j] = CONST_INEQUAL_A(i, j);
            std::cout << ctrl_A[CTRL_N_INEQ_CONST*i + j] << ", ";
        }
    }

    std::cout << "b: " << std::endl;
    for(int i = 0; i < CTRL_N_INEQ_CONST; i++)
    {
        ctrl_b[i] = CONST_INEQUAL_B(i);
        std::cout << ctrl_b[i] << std::endl;
    }

    std::cout << "xl: " << std::endl;
    for(int i = 0; i < CTRL_N_LB_CONST; i++)
    {
        ctrl_xl[i] = CONST_BOUND_LOWER(i);
        std::cout << ctrl_xl[i] << std::endl;
    }

    std::cout << "xu: " << std::endl;
    for(int i = 0; i < CTRL_N_UB_CONST; i++)
    {
        ctrl_xu[i] = CONST_BOUND_UPPER(i);
        std::cout << ctrl_xu[i] << std::endl;
    }

    for(int i = 0; i < CTRL_N_HORIZON; i++)
    {
        ctrl_Ustar[i] = 0;
    }

    // Calculate Integral Action Matrices.
    Az << dt * C, 1.0;
    Bz << dt * D;

    
}

bool ControlSystem::calculateControlService(acri_controller::Control::Request& req, acri_controller::Control::Response& res)
{
    Eigen::Vector3d x;
    x << req.x_in.theta,
         req.x_in.P_theta,
         req.x_in.z;

    // compute f_MPC as a c-style array.
    double ctrl_f[CTRL_N_HORIZON];
    double ctrl_lm[CTRL_N_EQ_CONST + CTRL_N_INEQ_CONST + CTRL_N_LB_CONST + CTRL_N_UB_CONST];
    for(int i = 0; i < N_horizon; i++)
    {
        double firstTerm  = fbar_MPC(i, 0)*(x(0) - x_ref(0));
        double secondTerm = fbar_MPC(i, 1)*(x(1) - x_ref(1));
        double thirdTerm  = fbar_MPC(i, 2)*(x(2) - x_ref(2));
        double fourthTerm = fbar_MPC(i, 3)*z(0, 0);
        ctrl_f[i] = firstTerm + secondTerm + thirdTerm + fourthTerm;
    }

    ctrl_b[0] = ctrl_Ustar[0] + delta_u_max;
    ctrl_b[CTRL_N_HORIZON] = -ctrl_Ustar[0] - delta_u_min;

    // other qpas_sub_noblas variables.
    int numits, numadd, numdrop;

    qpas_sub_noblas(CTRL_N_HORIZON, CTRL_N_EQ_CONST, CTRL_N_INEQ_CONST, CTRL_N_LB_CONST, CTRL_N_UB_CONST, ctrl_H, ctrl_f, ctrl_A, ctrl_b, ctrl_xl, ctrl_xu, ctrl_Ustar, ctrl_lm, 0, &numits, &numadd, &numdrop);
    res.v = ctrl_Ustar[0];

    // Update the integral action state.
    Eigen::Vector4d integralActionTerm;
    integralActionTerm << x(0) - x_ref(0),
                          x(1) - x_ref(1),
                          x(2) - x_ref(2),
                          z;

    z = Az * integralActionTerm;                                                // Neglected Bz term to remove unnecessary computation.    

    return true;      
}

Eigen::MatrixXd ControlSystem::power(Eigen::MatrixXd in, int i, int j)
{
    assert(i >= j);
    Eigen::MatrixXd out = Eigen::MatrixXd::Identity(in.rows(), in.cols());
    int numRepetitions = i - j;
    for(int k = 0; k < numRepetitions; k++)
    {
        out = out * in;
    }
    return out;
}

void ControlSystem::setVelocity(double _v)
{
    // TODO: Add velocity constraint assertions here.
    v = _v;
}


// ### CONTROLLER PRIVATE INTERFACE ###

void ControlSystem::dlqr_cost()
{
    int Nx = A_int.rows();
    int Nu = B_int.cols();
    int n  = Nx + Nu;

    Eigen::MatrixXd Za = Eigen::MatrixXd::Zero(Nx, Nx);
    Eigen::MatrixXd Zb = Eigen::MatrixXd::Zero(Nx, Nu);
    Eigen::MatrixXd Zu = Eigen::MatrixXd::Zero(Nu, Nu);

    Eigen::MatrixXd M(2*n, 2*n);

    // Rows 1 -> Nx
    M.block(        0,         0, Nx, Nx) = -A_int.transpose();
    M.block(        0,        Nx, Nx, Nu) = Zb;
    M.block(        0,   Nx + Nu, Nx, Nx) = Q;
    M.block(        0, 2*Nx + Nu, Nx, Nu) = N;

    // Rows Nx -> Nx + Nu
    M.block(       Nx,         0, Nu, Nx) = -B_int.transpose();
    M.block(       Nx,        Nx, Nu, Nu) = Zu;
    M.block(       Nx,   Nx + Nu, Nu, Nx) = N.transpose();
    M.block(       Nx, 2*Nx + Nu, Nu, Nu) = R;

    // Rows Nx + Nu -> 2Nx + Nu
    M.block(  Nx + Nu,         0, Nx, Nx) = Za;
    M.block(  Nx + Nu,        Nx, Nx, Nu) = Zb;
    M.block(  Nx + Nu,   Nx + Nu, Nx, Nx) = A_int;
    M.block(  Nx + Nu, 2*Nx + Nu, Nx, Nu) = B_int;

    // Rows 2Nx + Nu -> 2(Nx + Nu)
    M.block(2*Nx + Nu,         0, Nu, Nx) = Zb.transpose();
    M.block(2*Nx + Nu,        Nx, Nu, Nu) = Zu;
    M.block(2*Nx + Nu,   Nx + Nu, Nu, Nx) = Zb.transpose();
    M.block(2*Nx + Nu, 2*Nx + Nu, Nu, Nu) = Zu;

    // Convert the M matrix to discrete time.
    Eigen::MatrixXd expTerm = M*dt;
    Eigen::MatrixXd phi     = expTerm.exp();
    Eigen::MatrixXd phi12   = phi.block(0, n, n, n);
    Eigen::MatrixXd phi22   = phi.block(n, n, n, n);
    Eigen::MatrixXd QQ      = phi22.transpose() * phi12;
    QQ                      = (QQ + QQ.transpose()) / 2.0;

    // Extract Ad, Bd, Qd, Rd, Nd.
    Ad = phi22.block( 0,  0, Nx, Nx);
    Bd = phi22.block( 0, Nx, Nx, Nu);
    Qd =    QQ.block( 0,  0, Nx, Nx);
    Rd =    QQ.block(Nx, Nx, Nu, Nu);
    Nd =    QQ.block( 0, Nx, Nx, Nu);
}

// ### OBSERVER PUBLIC INTERFACE ###

bool ControlSystem::updateObserverService(acri_controller::SystemObs::Request& req, acri_controller::SystemObs::Response& res)
{
    // Extract the variables from the req object.
    acri_controller::State x_in = req.x_in;
    x_obs(0) = x_in.theta;
    x_obs(1) = x_in.P_theta;
    x_obs(2) = x_in.z;
    double y = req.y;
    double v = req.v;
    setVelocity(v);

    // Construct the matrix of measurements of the states.
    Eigen::Vector3d y_obs;
    y_obs << y,
             0,
             0;

    // Update x_obs.
    x_obs = (G - Lc*C)*x_obs + H*v + Lc*C*y_obs;
    
    // Construct the response object using the update x_obs.
    res.x_out.theta   = x_obs(0);
    res.x_out.P_theta = x_obs(1);
    res.x_out.z       = x_obs(2);
    
    return true;
}


// ### OBSERVER PRIVATE INTERFACE ###

void ControlSystem::InitialiseObserver()
{
    // Copy the relevant components of Ad and Bd into G and H.
    G << Ad(0, 0), Ad(0, 1), Ad(0, 2),
         Ad(1, 0), Ad(1, 1), Ad(1, 2),
         Ad(2, 0), Ad(2, 1), Ad(2, 2);
    H << Bd(0),
         Bd(1),
         Bd(2);

    // Calculate the 3 observer poles.
    double pole1 = std::exp(-4 * dt);
    double pole2 = std::exp(-3 * dt);
    double pole3 = std::exp(-6 * dt);

    // Calculate Observer Gain.
    acri_controller::ObserverInit srv;
    srv.request.G_00  = G(0, 0);
    srv.request.G_01  = G(0, 1);
    srv.request.G_02  = G(0, 2);
    srv.request.G_10  = G(1, 0);
    srv.request.G_11  = G(1, 1);
    srv.request.G_12  = G(1, 2);
    srv.request.G_20  = G(2, 0);
    srv.request.G_21  = G(2, 1);
    srv.request.G_22  = G(2, 2);
    srv.request.C_0   = C(0, 0);
    srv.request.C_1   = C(0, 1);
    srv.request.C_2   = C(0, 2);
    srv.request.pole1 = pole1;
    srv.request.pole2 = pole2;
    srv.request.pole3 = pole3;
    ros::NodeHandle n_obs;
    ros::ServiceClient client = n_obs.serviceClient<acri_controller::ObserverInit>("calculateObserverGain");
    if(client.call(srv))
    {
        Lc << srv.response.Lc_0,
              srv.response.Lc_1,
              srv.response.Lc_2;
        std::cout << "Lc: " << std::endl << Lc << std::endl;
        
    }
    else
    {
        ROS_ERROR("Failed to call service calculateObserverGain");
    }

}

// ### STATE DYNAMICS PUBLIC INTERFACE ###

bool ControlSystem::updateStateService(acri_controller::System::Request& req, acri_controller::System::Response& res)
{
    // Extract the variables from the req object.
    acri_controller::State x_in = req.x_in;
    x(0) = x_in.theta;
    x(1) = x_in.P_theta;
    x(2) = x_in.z;
    double v = req.v;
    setVelocity(v);

    // Run the runge-kutta solver.
    updateState();

    // Add the updated state to the res object.
    res.x_out.theta   = x(0);
    res.x_out.P_theta = x(1);
    res.x_out.z       = x(2);

    return true;
}

void ControlSystem::updateState()
{
    // Construct the std::vector state vector from the eigen member variable.
    std::vector<double> x_vec;
    x_vec.push_back(x(0));
    x_vec.push_back(x(1));
    x_vec.push_back(x(2));

    // Define the runge-kutta solver and any extra member variables it requires.
    double t_RK = 0.0;              // This value is not important - t is not used in the calculation of the system dynamics.
    odeint::runge_kutta_dopri5<std::vector<double>> rk;

    // Perform runge-kutta step.
    rk.do_step(make_ode_wrapper(ControlSystem(v), &ControlSystem::systemDynamics), x_vec, t_RK, dt);

    // Update the eigen state vector with the results of the runge kutta step.
    x(0) = x_vec[0];
    x(1) = x_vec[1];
    x(2) = x_vec[2];
}

Eigen::Vector3d ControlSystem::getState()
{
    return x;
}



// ### STATE DYNAMICS PRIVATE INTERFACE ###

void ControlSystem::systemDynamics(const std::vector<double> & x, std::vector<double> & dx, double t)
{
    dx.clear();

    // Extract states from the state vector.
    double theta   = x[0];
    double P_theta = x[1];
    double z       = x[2];

    // Calculate the state derivatives.
    dx.push_back((P_theta - m*d*v) / (m * (d*d + z*z) + J));
    dx.push_back(m*g*(z*std::cos(theta) + d*std::sin(theta)));
    dx.push_back(v);
}


