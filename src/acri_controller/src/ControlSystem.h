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
#include "ros/ros.h"
#include "acri_controller/State.h"
#include "acri_controller/System.h"

namespace odeint = boost::numeric::odeint;

class ControlSystem
{
    public:

        // ### CONSTRUCTORS ###

        ControlSystem(double _v);


        ControlSystem(Eigen::Vector3d & x_init, double theta_bar = 0);
        // Precondition:  None.
        // Postcondition: A new ControlSystem object is created, with x and x_obs initial conditions set to x_init and the equillibrium point set using theta_bar.

        ControlSystem(Eigen::Vector3d & x_init, Eigen::Vector3d & x_obs_init, double theta_bar = 0);
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

        void setVelocity(double _v);
        // Precondition:  None.
        // Postcondition: The tank velocity member variable is updated.

        void initialiseMPC();
        // Precondition:  None.
        // Postcondition: 


        // ### OBSERVER PUBLIC INTERFACE ###

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
        double m = 600.0;
        double d = 0.45;
        double w = 0.9;
        double g = 9.81;
        double J = (1.0 / 12.0) * m * ((2.0*d)*(2.0*d) + (2.0*w)*(2.0*w));

        // State vectors.
        Eigen::Vector3d x;                                  // State vector.
        Eigen::Vector3d x_obs;                              // Observed state vector.

        // Equillibrium point.
        Eigen::Vector3d x_bar;

        // Continuous time linearised system matrices.
        Eigen::Matrix3d A;                                  // Jacobian with respect to state.
        Eigen::Vector3d B;                                  // Jacobian with respect to the input.

        // Discrete time linearised system matrices.
        Eigen::Matrix3d G;                                  // Discrete time version of A.
        Eigen::Vector3d H;                                  // Discrete time version of B.

        // Linearised measurement model matrices.
        Eigen::Matrix<double, 1, 3> C;                      // Maps state vector to output vector.
        double D = 0;                                       // Maps input vector to output vector.

        // timestep size.     
        double dt = 0.1;

        // Input.
        double v = 0;

        // MPC Variables.
        Eigen::Matrix4d A_int;                              // A matrix, augmented for integral action.
        Eigen::Vector4d B_int;                              // B matrix, augmented for integral action.
        Eigen::Matrix4d Q;                                  // Q matrix.
        Eigen::Matrix<double, 1, 1> R;                      // R matrix. 
        int N_horizon;                                      // MPC Horizon.
        Eigen::Vector4d N;                                  // N term.
        Eigen::Matrix4d Ad;                                 // dlqr_cost output variable.
        Eigen::Vector4d Bd;                                 // dlqr_cost output variable.
        Eigen::Matrix4d Qd;                                 // dlqr_cost output variable.
        Eigen::Matrix<double, 1, 1> Rd;                     // dlqr_cost output variable.
        Eigen::Vector4d Nd;                                 // dlqr_cost output variable.
        Eigen::Matrix4d Q_f;                                // DARE output variable.
        Eigen::Vector3d N_x;                                // State feedforward term.
        Eigen::Matrix<double, 1, 1> N_u;                    // Input feedforward term.
        Eigen::MatrixXd A_MPC;
        Eigen::MatrixXd B_MPC;
        Eigen::MatrixXd Q_MPC;
        Eigen::MatrixXd R_MPC;
        Eigen::MatrixXd N_MPC;
        Eigen::MatrixXd H_MPC;
        Eigen::MatrixXd fbar_MPC;


        // ### CONTROLLER PRIVATE INTERFACE ###

        void dlqr_cost();
        // Precondition:  
        // Postcondition: 

        void dare();
        // Precondition:  
        // Postcondition: 

        Eigen::MatrixXd power(Eigen::MatrixXd in, int i, int j);


        // ### STATE DYNAMICS PRIVATE INTERFACE ###
       
       void systemDynamics(const std::vector<double> & x, std::vector<double> & dx, double t);


};





#endif
