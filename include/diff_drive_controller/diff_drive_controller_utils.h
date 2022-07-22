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

#pragma once

#include <diff_drive_controller/DiffDriveControllerState.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/assert.h>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <vector>
#include <string>

namespace diff_drive_controller
{

double euclideanOfVectors(const urdf::Vector3& vec1, const urdf::Vector3& vec2);

/*
 * \brief Check if the link is modeled as a cylinder
 * \param link Link
 * \return true if the link is modeled as a Cylinder; false otherwise
 */
bool isCylinder(const urdf::LinkConstSharedPtr& link);

/*
 * \brief Get the wheel radius
 * \param [in]  wheel_link   Wheel link
 * \param [out] wheel_radius Wheel radius [m]
 * \return true if the wheel radius was found; false otherwise
 */
bool getWheelRadius(
    const urdf::LinkConstSharedPtr& wheel_link,
    double& wheel_radius);

void resize(trajectory_msgs::JointTrajectoryPoint& msg,
    const std::size_t size);

void resize(DiffDriveControllerState& msg, const std::size_t size);

template <typename T>
void error(
    std::vector<T>& err,
    const std::vector<T>& desired,
    const std::vector<T>& actual)
{
  ROS_ASSERT(err.size() == desired.size());
  ROS_ASSERT(err.size() == actual.size());

  // Compute: error = desired - actual
  std::transform(desired.begin(), desired.end(), actual.begin(),
      err.begin(), std::minus<T>());
}

void error(
    trajectory_msgs::JointTrajectoryPoint& err,
    const trajectory_msgs::JointTrajectoryPoint& desired,
    const trajectory_msgs::JointTrajectoryPoint& actual);

void error(DiffDriveControllerState& msg);

}  // namespace diff_drive_controller
