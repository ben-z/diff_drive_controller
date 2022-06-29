/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the PAL Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Bence Magyar
 * Author: Enrique Fernández
 */

#include "diff_drive_controller/diff_drive_controller_utils.h"

namespace diff_drive_controller
{

double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2)
{
  return std::sqrt(std::pow(vec1.x-vec2.x, 2) +
                   std::pow(vec1.y-vec2.y, 2) +
                   std::pow(vec1.z-vec2.z, 2));
}

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
bool isCylinder(const urdf::LinkConstSharedPtr& link)
{
  if (!link)
  {
    ROS_ERROR("Link == NULL.");
    return false;
  }

  if (!link->collision)
  {
    ROS_ERROR_STREAM("Link " << link->name <<
        " does not have collision description. "
        "Add collision description for link to urdf.");
    return false;
  }

  if (!link->collision->geometry)
  {
    ROS_ERROR_STREAM("Link " << link->name <<
        " does not have collision geometry description. "
        "Add collision geometry description for link to urdf.");
    return false;
  }

  if (link->collision->geometry->type != urdf::Geometry::CYLINDER)
  {
    ROS_ERROR_STREAM("Link " << link->name <<
        " does not have cylinder geometry");
    return false;
  }

  return true;
}

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
bool getWheelRadius(
    const urdf::LinkConstSharedPtr& wheel_link,
    double& wheel_radius)
{
  if (!isCylinder(wheel_link))
  {
    ROS_ERROR_STREAM("Wheel link " << wheel_link->name <<
        " is NOT modeled as a cylinder!");
    return false;
  }

  wheel_radius = (static_cast<urdf::Cylinder*>(
        wheel_link->collision->geometry.get()))->radius;
  return true;
}

void resize(trajectory_msgs::JointTrajectoryPoint& msg,
    const std::size_t size)
{
  msg.positions.resize(size);
  msg.velocities.resize(size);
  msg.accelerations.resize(size);
  msg.effort.resize(size);
}

void resize(DiffDriveControllerState& msg, const std::size_t size)
{
  msg.joint_names.resize(size);

  resize(msg.desired         , size);
  resize(msg.actual          , size);
  resize(msg.limited         , size);
  resize(msg.error           , size);
  resize(msg.actual_estimated, size);
  resize(msg.error_estimated , size);

  resize(msg.actual_side_average          , 2);
  resize(msg.error_side_average           , 2);
  resize(msg.actual_estimated_side_average, 2);
  resize(msg.error_estimated_side_average , 2);
}

void error(
    trajectory_msgs::JointTrajectoryPoint& err,
    const trajectory_msgs::JointTrajectoryPoint& desired,
    const trajectory_msgs::JointTrajectoryPoint& actual)
{
  error(err.positions    , desired.positions    , actual.positions);
  error(err.velocities   , desired.velocities   , actual.velocities);
  error(err.accelerations, desired.accelerations, actual.accelerations);
  error(err.effort       , desired.effort       , actual.effort);

  err.time_from_start = actual.time_from_start;
}

void error(DiffDriveControllerState& msg)
{
  error(msg.error          , msg.desired, msg.actual);
  error(msg.error_estimated, msg.desired, msg.actual_estimated);

  error(msg.error_side_average,
      msg.desired, msg.actual_side_average);
  error(msg.error_estimated_side_average,
      msg.desired, msg.actual_estimated_side_average);

  msg.control_period_error =
      msg.control_period_desired - msg.control_period_actual;
  msg.control_period_error_estimated =
      msg.control_period_desired - msg.control_period_actual_estimated;
}

}  // namespace diff_drive_controller
