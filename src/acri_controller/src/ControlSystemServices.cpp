#include "Eigen/Core"
#include "ros/ros.h"
#include "ControlSystem.h"

int main(int argc, char **argv)
{
    // Initialise the control system node.
    ros::init(argc, argv, "ControlSystemServer");
    ros::NodeHandle nh;

    // Instantiate the ControlSystem class.
    Eigen::Vector3d x_init;
    x_init << 0.0,
              0.0,
              0.0;
    double theta_bar = 0;
    ControlSystem controlSystem(argc, argv, x_init, theta_bar);

    // Initialise the UpdateState service.
    ros::ServiceServer updateStateService = nh.advertiseService("update_state", &ControlSystem::updateStateService, &controlSystem);

    // Initialise the CalculateControl service.
    ros::ServiceServer calculateControlService = nh.advertiseService("calculate_control", &ControlSystem::calculateControlService, &controlSystem);

    // Initialise the UpdateObserver service.
    ros::ServiceServer updateObserverService = nh.advertiseService("update_observer", &ControlSystem::updateObserverService, &controlSystem);

    // Spin the ros node.
    ros::spin();

}