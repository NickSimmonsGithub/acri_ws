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

ControlSystem::ControlSystem(Eigen::Vector3d & x_init, double theta_bar)
{
    // Calculate the remaining equillibrium point members from theta_bar.
    xbarCalc(theta_bar);

    // Calculate A, B, C, G and H.
    lineariseSystem();

    // Insert initial state to the state vector.
    x     = x_init;
    x_obs = x_init;

    initialiseMPC();
}


ControlSystem::ControlSystem(Eigen::Vector3d & x_init, Eigen::Vector3d & x_obs_init, double theta_bar)
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

    // Calculate G.
    Eigen::Matrix3d A_dt(3, 3);
    A_dt = dt * A;
    G = A_dt.exp();

    std::cout << "G matrix: " << std::endl << G << std::endl;

}      


// ### CONTROLLER PUBLIC INTERFACE ###

void ControlSystem::initialiseMPC()
{
    // Populate tuning parameters.
    double q1 = 1e7;
    double q2 = 1e-8;
    double q3 = 1e3;
    double q4 = 1e6;

    Q = Eigen::MatrixXd::Zero(4, 4);
    Q(0, 0) = q1;
    Q(1, 1) = q2;
    Q(2, 2) = q3;
    Q(3, 3) = q4;

    R << 1.0;

    N_horizon = 50;

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

    // Discretise system.
    dlqr_cost();

    // Compute terminal cost. TODO: Ask Joel about this gain term.
    dare();

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
    std::cout << "H_MPC: " << std::endl << H_MPC << std::endl;

    fbar_MPC = 2.0 * (B_MPC.transpose()*Q_MPC + N_MPC.transpose())*A_MPC;
    std::cout << "fbar_MPC: " << std::endl << fbar_MPC << std::endl;

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

void ControlSystem::dare()                          
{
    // Initialise recursive parameters.
    /*
    Eigen::MatrixXd Ak = Ad;
    Eigen::MatrixXd Gk = Bd * Rd.colPivHouseholderQr().solve(Bd.transpose());
    Eigen::MatrixXd Hk = Qd;
    Eigen::MatrixXd invTerm = Eigen::MatrixXd::Identity(Bd.rows(), Bd.rows()) - Gk*Hk;
    Eigen::MatrixXd Ak_plus1 = Ak*invTerm.colPivHouseholderQr().solve(Ak);
    Eigen::MatrixXd Gk_plus1 = Gk + Ak*invTerm.colPivHouseholderQr().solve(Gk)*Ak.transpose();
    Eigen::MatrixXd Hk_plus1 = Hk + Ak.transpose()*Hk*invTerm.colPivHouseholderQr().solve(Ak);
    Eigen::MatrixXd Hk_plus1_minus_Hk = Hk_plus1 - Hk;
    double epsilon = 0.000001;
    while(Hk_plus1_minus_Hk.norm() / Hk_plus1.norm() > epsilon)
    {
        Ak = Ak_plus1;
        Gk = Gk_plus1;
        Hk = Hk_plus1;
        invTerm = Eigen::MatrixXd::Identity(Bd.rows(), Bd.rows()) - Gk*Hk;
        Ak_plus1 = Ak*invTerm.colPivHouseholderQr().solve(Ak);
        Gk_plus1 = Gk + Ak*invTerm.colPivHouseholderQr().solve(Gk)*Ak.transpose();
        Hk_plus1 = Hk + Ak.transpose()*Hk*invTerm.colPivHouseholderQr().solve(Ak);
        Hk_plus1_minus_Hk = Hk_plus1 - Hk;
    }
    */
    #define EPS 0.000001

    Eigen::MatrixXd At = Ad.transpose();
    Eigen::MatrixXd Bt = Bd.transpose();
    Eigen::MatrixXd X = Qd;

    double d = EPS;
    for (std::size_t ii=0; ii < 100000 && d >= EPS; ++ii)
    {
        Eigen::MatrixXd Xp = X;
        
        X = Q + At*X*Ad - At*X*Bd*(Bt*X*Bd+Rd).inverse()*Bt*X*Ad;
        d = (X - Xp).array().abs().sum();
    }
    
    Q_f = X;
}

// ### STATE DYNAMICS PUBLIC INTERFACE ###

bool ControlSystem::updateStateService(acri_controller::System::Request& req, acri_controller::System::Response& res)
{
    // Extract the variables from the req object.
    acri_controller::State x_in = req.x_in;
    x(0) = x_in.theta;
    x(1) = x_in.P_theta;
    x(2) = x_in.z;
    setVelocity(req.v);

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


