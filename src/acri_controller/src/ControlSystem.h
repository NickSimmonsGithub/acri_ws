// ControlSystem.h
// Written by: Nicholas Simmons, C3330381
// Last modified: 24/09/2022
//
// This file (along with ControlSystem.cpp) implements the ACRI trackside robotic device's observer and controller modules.
// It also implements a full state space representation of the tank for tuning the observer.

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <Eigen/Core>
#include <boost/numeric/odeint.hpp>
#include "qpas_sub_noblas.h"
#include "ros/ros.h"
#include "acri_controller/State.h"
#include "acri_controller/System.h"
#include "acri_controller/Control.h"
#include "acri_controller/SystemObs.h"

namespace odeint = boost::numeric::odeint;

class ControlSystem
{
    public:

        // ### CONSTRUCTORS ###

        ControlSystem(double _v);


        ControlSystem(int argc, char** argv, Eigen::Vector3d & x_init, double theta_bar = 0);
        // Precondition:  None.
        // Postcondition: A new ControlSystem object is created, with x and x_obs initial conditions set to x_init and the equillibrium point set using theta_bar.

        ControlSystem(int argc, char** argv, Eigen::Vector3d & x_init, Eigen::Vector3d & x_obs_init, double theta_bar = 0);
        // Precondition:  None.
        // Postcondition: A new ControlSystem object is created, with x and x_obs initial conditions set to x_init and x_obs_init respectively and equillibrium point set using theta_bar.


        // ### DESTRUCTOR ###

        ~ControlSystem();
        // Precondition:  A valid ControlSystem object exists.
        // Postcondition: The ControlSystem object is deleted.


        // ### LINEARISATION PUBLIC INTERFACE ###

        void xbarCalc(double theta_bar);
        // Precondition:  None.
        // Postcondition: x_bar is set using theta_bar.

        void lineariseSystem();
        // Precondition:  x_bar has been set.
        // Postcondition: A, B, G and H matrices are calculated using the system parameters and x_bar.


        // ### CONTROLLER PUBLIC INTERFACE ### 

        void initialiseMPC();
        // Precondition:  None.
        // Postcondition: All matrices required to run MPC function are initialised.

        void setReference(double theta_ref);
        // Precondition:  None.
        // Postcondition: The controller reference point is set.

        // double calculateControl(bool _isObservedValueUsed);
        // Precondition:  InitialiseMPC has been run.
        // Postcondition: The optimal control is returned and the integral action term is updated.

        bool calculateControlService(acri_controller::Control::Request& req, acri_controller::Control::Response& res);
        // Precondition:  InitialiseMPC has been run.
        // Postcondition:  

        


        // ### OBSERVER PUBLIC INTERFACE ###

        bool updateObserverService(acri_controller::SystemObs::Request& req, acri_controller::SystemObs::Response& res);

        void updateObserverState();
        // Preconditions: x_obs contains a valid state estimate, x_obs(k).
        //                G contains a valid discrete time version of A.
        //                H contains a valid discrete time version of B.
        // Postcondition: x_obs is replaced with an updated state estimate, x_obs(k + 1).

        Eigen::Vector3d getObserverState();
        // Precondition:  x_obs contains a valid state estimate, x_obs(k).
        // Postcondition: returns the contents of x_obs(k).


        // ### STATE DYNAMICS PUBLIC INTERFACE ###

        bool updateStateService(acri_controller::System::Request& req, acri_controller::System::Response& res);

        void updateState();
        // Preconditions: x contains a valid state, x(k).
        // Postcondition: x is replaced with an updated state estimate, x(k + 1).

        Eigen::Vector3d getState();
        // Precondition:  x contains a valid state, x(k).
        // Postcondition: returns the contents of x(k).



    private:

        // System parameters.
        double m = 540.0;
        double d = 0.45;
        double w = 0.9;
        double g = 9.81;
        double J = (1.0 / 12.0) * m * ((2.0*d)*(2.0*d) + (2.0*w)*(2.0*w));

        // State vectors.
        Eigen::Vector3d x;                                  // State vector.
        Eigen::Vector3d x_obs;                              // Observed state vector.

        // Equillibrium point.
        Eigen::Vector3d x_bar;

        // Reference point.
        Eigen::Vector3d x_ref;

        // Continuous time linearised system matrices.
        Eigen::Matrix3d A;                                  // Jacobian with respect to state.
        Eigen::Vector3d B;                                  // Jacobian with respect to the input.

        // Observer matrices.
        Eigen::Matrix3d G;                                  // Discrete time version of A.
        Eigen::Vector3d H;                                  // Discrete time version of B.
        Eigen::Vector3d Lc;                                 // Observer gain matrix.

        // Linearised measurement model matrices.
        Eigen::Matrix<double, 1, 3> C;                      // Maps state vector to output vector.
        double D = 0;                                       // Maps input vector to output vector.

        // timestep size.     
        double dt = 0.025;

        // Input.
        double u = 0;
        double v = 0;

        // MPC Variables.
        Eigen::Matrix4d A_int;                                              // A matrix, augmented for integral action.
        Eigen::Vector4d B_int;                                              // B matrix, augmented for integral action.
        Eigen::Vector4d N;
        Eigen::Matrix4d Ad;
        Eigen::Vector4d Bd;
        Eigen::Matrix4d Qd;
        Eigen::Matrix<double, 1, 1> Rd;                                     // dlqr_cost output variable.
        Eigen::Vector4d Nd;                                                 // dlqr_cost output variable.
        Eigen::MatrixXd Q_f;                                                // DARE output variable.
        Eigen::Vector3d N_x;                                                // State feedforward term.
        Eigen::Matrix<double, 1, 1> N_u;                                    // Input feedforward term.
        Eigen::MatrixXd A_MPC;
        Eigen::MatrixXd B_MPC;
        Eigen::MatrixXd Q_MPC;
        Eigen::MatrixXd R_MPC;
        Eigen::MatrixXd N_MPC;
        Eigen::MatrixXd H_MPC;
        Eigen::MatrixXd fbar_MPC;
        Eigen::MatrixXd f_MPC;
        Eigen::MatrixXd CONST_INEQUAL_A;
        Eigen::VectorXd DeltaUMin;
        Eigen::VectorXd DeltaUMax;
        Eigen::VectorXd CONST_INEQUAL_B;
        Eigen::VectorXd CONST_BOUND_UPPER;
        Eigen::VectorXd CONST_BOUND_LOWER;

        // MPC Tuning Parameters.
        const static int N_horizon = 50;                                    // MPC Horizon.
        Eigen::Matrix4d Q;                                                  // Q matrix.
        Eigen::Matrix<double, 1, 1> R;                                      // R matrix.
        double delta_u_max;                                                 // Upper slew rate constraint.
        double delta_u_min;                                                 // Lower slew rate constraint.
        double u_max;                                                       // Upper input constraint.
        double u_min;                                                       // Lower input constraint.

        // qpas_sub_noblas constants.
        const static int CTRL_N_INPUT = 1;
        const static int CTRL_N_STATE = 4;
        const static int CTRL_N_OUTPUT = 1;
        const static int CTRL_N_HORIZON = N_horizon;
        const static int CTRL_N_EQ_CONST = 0;
        const static int CTRL_N_INEQ_CONST = 2 * N_horizon;
        const static int CTRL_N_LB_CONST = N_horizon;
        const static int CTRL_N_UB_CONST = N_horizon;

        // qpas_sub_noblas c-style arrays.
        double ctrl_H[CTRL_N_HORIZON * CTRL_N_HORIZON];
        double ctrl_A[CTRL_N_INEQ_CONST * CTRL_N_HORIZON];
        double ctrl_b[CTRL_N_INEQ_CONST];
        double ctrl_xl[CTRL_N_LB_CONST];
        double ctrl_xu[CTRL_N_UB_CONST];
        double ctrl_Ustar[CTRL_N_HORIZON*CTRL_N_INPUT];

        // Integral Action Terms.
        Eigen::Matrix<double, 1, 1> z;
        Eigen::Matrix<double, 1, 4> Az;
        Eigen::Matrix<double, 1, 1> Bz;

        // ### CONTROLLER PRIVATE INTERFACE ###

        void dlqr_cost();
        // Precondition:  
        // Postcondition: 

        void dare();
        // Precondition:  
        // Postcondition: 

        Eigen::MatrixXd power(Eigen::MatrixXd in, int i, int j);
        // Precondition:  
        // Postcondition: 

        void setVelocity(double _v);
        // Precondition:  None.
        // Postcondition: The tank velocity member variable is updated.


        // ### OBSERVER PRIVATE INTERFACE ### 

        void InitialiseObserver();
        // Precondition:  
        // Postcondition: 


        // ### STATE DYNAMICS PRIVATE INTERFACE ###
       
       void systemDynamics(const std::vector<double> & x, std::vector<double> & dx, double t);


};

#endif
