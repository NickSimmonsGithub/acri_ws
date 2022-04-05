/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include "acri_gazebo/track_velocity.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/transport/transport.hh"
#include "TrackedVehiclePluginROS.hh"

using namespace gazebo;

namespace gazebo
{
  bool trackedVehiclePoseWarningIssued = false;
}

/// \brief Private data class
class gazebo::TrackedVehiclePluginROSPrivate
{
  /// \brief Pointer to model containing plugin.
  public: physics::ModelPtr model;

  /// \brief SDF for this plugin;
  public: sdf::ElementPtr sdf;

  /// \brief Pointer to a node with robot prefix.
  public: transport::NodePtr robotNode;

  /// \brief linear velocity command Gazebo subscriber.
  public: transport::SubscriberPtr velocityPoseSub;

  /// \brief angular velocity command Gazebo subscriber.
  public: transport::SubscriberPtr velocityTwistSub;
  
  /// \brief Publisher of the track velocities.
  public: transport::PublisherPtr tracksVelocityPub;

  /// \brief A node use for ROS transport
  public: ros::NodeHandle trackPoseRosNode;

  /// \brief A ROS subscriber
  public: ros::Subscriber trackPoseRosSub;

  /// \brief A ROS callbackqueue that helps process messages
  public: ros::CallbackQueue trackPoseQueue;

  /// \brief A thread the keeps running the rosQueue
  public: std::thread trackPoseQueueThread;


  /// \brief Distance between the centers of the tracks.
  public: double tracksSeparation = 0.1;

  /// \brief Steering efficiency coefficient (between 0.0 and 1.0).
  public: double steeringEfficiency = 0.5;

  /// \brief Max linear velocity in m/s. Also max track velocity.
  public: double maxLinearSpeed = 1.0;

  /// \brief Max angular speed in rad/s.
  public: double maxAngularSpeed = 1.0;

  /// \brief Friction coefficient in the first friction direction.
  public: boost::optional<double> trackMu;

  /// \brief Friction coefficient in the second friction direction.
  public: boost::optional<double> trackMu2;

  /// \brief Namespace used as a prefix for gazebo topic names.
  public: std::string robotNamespace;
  
};

TrackedVehiclePluginROS::TrackedVehiclePluginROS()
  : dataPtr(new TrackedVehiclePluginROSPrivate)
{
  this->trackNames[Tracks::LEFT] = "left";
  this->trackNames[Tracks::RIGHT] = "right";
}

TrackedVehiclePluginROS::~TrackedVehiclePluginROS() = default;

void TrackedVehiclePluginROS::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "TrackedVehiclePluginROS _model pointer is NULL");
  this->dataPtr->model = _model;

  GZ_ASSERT(_sdf, "TrackedVehiclePluginROS _sdf pointer is NULL");
  this->dataPtr->sdf = _sdf;

  // Load parameters from SDF plugin contents.
  this->LoadParam(_sdf, "robot_namespace", this->dataPtr->robotNamespace,
                  _model->GetName());
  this->LoadParam(_sdf, "steering_efficiency",
                  this->dataPtr->steeringEfficiency, 0.5);
  this->LoadParam(_sdf, "tracks_separation",
                  this->dataPtr->tracksSeparation, 0.4);
  this->LoadParam(_sdf, "max_linear_speed",
                  this->dataPtr->maxLinearSpeed, 1.);
  this->LoadParam(_sdf, "max_angular_speed",
                  this->dataPtr->maxAngularSpeed, 1.);

  if (_sdf->HasElement("track_mu"))
  {
    double mu;
    this->LoadParam(_sdf, "track_mu", mu, 2.0);
    this->dataPtr->trackMu = mu;
  }

  if (_sdf->HasElement("track_mu2"))
  {
    double mu2;
    this->LoadParam(_sdf, "track_mu2", mu2, 0.5);
    this->dataPtr->trackMu2 = mu2;
  }

  if (this->dataPtr->steeringEfficiency <= 0.)
    throw std::runtime_error("Steering efficiency must be positive");
  if (this->dataPtr->tracksSeparation <= 0.)
    throw std::runtime_error("Tracks separation must be positive");
  if (this->dataPtr->maxLinearSpeed <= 0.)
    throw std::runtime_error("Maximum linear speed must be positive");
  if (this->dataPtr->maxAngularSpeed < 0.)
    throw std::runtime_error("Maximum angular speed must be non-negative");
   
  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }  

  this->dataPtr->trackPoseRosSub = this->dataPtr->trackPoseRosNode.subscribe("acri_sim/track_velocity", 1, &TrackedVehiclePluginROS::OnRosMsgTrackVel, this);
  ros::AsyncSpinner spinner(1, &this->dataPtr->trackPoseQueue);
  this->dataPtr->trackPoseRosNode.setCallbackQueue(&this->dataPtr->trackPoseQueue);
  spinner.start();
  
}

void TrackedVehiclePluginROS::Init()
{
  // Initialize transport nodes.

  // Prepend world name to robot namespace if it isn't absolute.
  auto robotNamespace = this->GetRobotNamespace();
  if (!robotNamespace.empty() && robotNamespace.at(0) != '/')
  {
    robotNamespace = this->dataPtr->model->GetWorld()->Name() +
      "/" + robotNamespace;
  }
  this->dataPtr->robotNode = transport::NodePtr(new transport::Node());
  this->dataPtr->robotNode->Init(robotNamespace);

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

  this->dataPtr->velocityPoseSub =
      this->dataPtr->robotNode->Subscribe<msgs::Pose, TrackedVehiclePluginROS>(
          "~/cmd_vel", &TrackedVehiclePluginROS::OnVelMsg, this);

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

  this->dataPtr->velocityTwistSub =
      this->dataPtr->robotNode->Subscribe<msgs::Twist, TrackedVehiclePluginROS>(
          "~/cmd_vel_twist", &TrackedVehiclePluginROS::OnVelMsg, this);

  this->dataPtr->tracksVelocityPub =
    this->dataPtr->robotNode->Advertise<msgs::Vector2d>("~/tracks_speed", 1000);
    

    
}

void TrackedVehiclePluginROS::OnRosMsgTrackVel(const acri_gazebo::track_velocity& trackVelocity)
{
  this->SetTrackVelocity(trackVelocity.left, trackVelocity.right);
}

void TrackedVehiclePluginROS::Reset()
{
  this->SetTrackVelocity(0., 0.);

  ModelPlugin::Reset();
}

void TrackedVehiclePluginROS::SetTrackVelocity(double _left, double _right)
{
  // Apply the max track velocity limit.

  const auto left = ignition::math::clamp(_left,
                                          -this->dataPtr->maxLinearSpeed,
                                          this->dataPtr->maxLinearSpeed);
  const auto right = ignition::math::clamp(_right,
                                           -this->dataPtr->maxLinearSpeed,
                                           this->dataPtr->maxLinearSpeed);

  // Call the descendant custom handler of the subclass.
  this->SetTrackVelocityImpl(left, right);

  // Publish the resulting track velocities to anyone who is interested.
  auto speedMsg = msgs::Vector2d();
  speedMsg.set_x(left);
  speedMsg.set_y(right);
  this->dataPtr->tracksVelocityPub->Publish(speedMsg);
}

void TrackedVehiclePluginROS::SetBodyVelocity(
    const double _linear, const double _angular)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Compute effective linear and angular speed.
  const auto linearSpeed = ignition::math::clamp(
    _linear,
    -this->dataPtr->maxLinearSpeed,
    this->dataPtr->maxLinearSpeed);

  const auto angularSpeed = ignition::math::clamp(
    _angular,
    -this->dataPtr->maxAngularSpeed,
    this->dataPtr->maxAngularSpeed);

  // Compute track velocities using the tracked vehicle kinematics model.
  const auto leftVelocity = linearSpeed + angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  const auto rightVelocity = linearSpeed - angularSpeed *
    this->dataPtr->tracksSeparation / 2 / this->dataPtr->steeringEfficiency;

  // Call the track velocity handler (which does the actual vehicle control).
  this->SetTrackVelocity(leftVelocity, rightVelocity);
}

void TrackedVehiclePluginROS::OnVelMsg(ConstPosePtr &_msg)
{
  if (!trackedVehiclePoseWarningIssued)
  {
    gzwarn << "Controlling tracked vehicles via Pose messages is deprecated. "
              "Use the Twist API via ~/cmd_vel_twist." << std::endl;
    trackedVehiclePoseWarningIssued = true;
  }
  const auto yaw = msgs::ConvertIgn(_msg->orientation()).Euler().Z();
  this->SetBodyVelocity(_msg->position().x(), yaw);
}

void TrackedVehiclePluginROS::OnVelMsg(ConstTwistPtr &_msg)
{
  this->SetBodyVelocity(_msg->linear().x(), _msg->angular().z());
}




void TrackedVehiclePluginROS::TrackQueueThread()
{
  static const double timeout = 0.01;
  while (this->dataPtr->trackPoseRosNode.ok())
  {
    this->dataPtr->trackPoseQueue.callAvailable(ros::WallDuration(timeout));
  }
}


std::string TrackedVehiclePluginROS::GetRobotNamespace()
{
  return this->dataPtr->robotNamespace;
}

double TrackedVehiclePluginROS::GetSteeringEfficiency()
{
  return this->dataPtr->steeringEfficiency;
}

void TrackedVehiclePluginROS::SetSteeringEfficiency(double _steeringEfficiency)
{
  this->dataPtr->steeringEfficiency = _steeringEfficiency;
  this->dataPtr->sdf->GetElement("steering_efficiency")
    ->Set(_steeringEfficiency);
}

double TrackedVehiclePluginROS::GetTracksSeparation()
{
  return this->dataPtr->tracksSeparation;
}

boost::optional<double> TrackedVehiclePluginROS::GetTrackMu()
{
  return this->dataPtr->trackMu;
}

void TrackedVehiclePluginROS::SetTrackMu(double _mu)
{
  this->dataPtr->trackMu = _mu;
  this->dataPtr->sdf->GetElement("track_mu")->Set(_mu);
  this->UpdateTrackSurface();
}

boost::optional<double> TrackedVehiclePluginROS::GetTrackMu2()
{
  return this->dataPtr->trackMu2;
}

void TrackedVehiclePluginROS::SetTrackMu2(double _mu2)
{
  this->dataPtr->trackMu2 = _mu2;
  this->dataPtr->sdf->GetElement("track_mu2")->Set(_mu2);
  this->UpdateTrackSurface();
}

void TrackedVehiclePluginROS::SetLinkMu(const physics::LinkPtr &_link)
{
  if (!this->GetTrackMu().is_initialized() &&
    !this->GetTrackMu2().is_initialized())
  {
    return;
  }

  for (auto const &collision : _link->GetCollisions())
    {
      auto frictionPyramid = collision->GetSurface()->FrictionPyramid();
      if (frictionPyramid == nullptr)
      {
        gzwarn << "This dynamics engine doesn't support setting mu/mu2 friction"
          " parameters. Use its dedicated friction setting mechanism to set the"
          " wheel friction." << std::endl;
        break;
      }


      if (this->GetTrackMu().is_initialized())
      {
        double mu = this->GetTrackMu().get();
        if (!ignition::math::equal(frictionPyramid->MuPrimary(), mu, 1e-6))
        {
          gzdbg << "Setting mu (friction) of link '" << _link->GetName() <<
                "' from " << frictionPyramid->MuPrimary() << " to " <<
                mu << std::endl;
        }
        frictionPyramid->SetMuPrimary(mu);
      }

      if (this->GetTrackMu2().is_initialized())
      {
        double mu2 = this->GetTrackMu2().get();
        if (!ignition::math::equal(frictionPyramid->MuSecondary(), mu2, 1e-6))
        {
          gzdbg << "Setting mu2 (friction) of link '" << _link->GetName() <<
                "' from " << frictionPyramid->MuSecondary() << " to " <<
                mu2 << std::endl;
        }
        frictionPyramid->SetMuSecondary(mu2);
      }
    }
}
