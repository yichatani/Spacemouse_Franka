/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Core>
#include <pybind11/pybind11.h>
#include "motion/time_optimal/path.h"

namespace py = pybind11;

namespace motion {
namespace time_optimal {

class Trajectory {
 public:
  /// @brief Generates a time-optimal trajectory
  Trajectory(const Path& path, const Eigen::VectorXd& max_velocity,
             const Eigen::VectorXd& max_acceleration, double time_step = 0.001);

  ~Trajectory();

  /** @brief Call this method after constructing the object to make sure the
     trajectory generation succeeded without errors. If this method returns
     false, all other methods have undefined behavior. **/
  bool isValid() const;

  /// @brief Returns the optimal duration of the trajectory
  double getDuration() const;

  size_t getTrajectorySegmentIndex(double time);

  /** @brief Return the position/configuration vector for a given point in time
   */
  Eigen::VectorXd getPosition(double time) const;
  /** @brief Return the velocity vector for a given point in time */
  Eigen::VectorXd getVelocity(double time) const;
  /** @brief Return the acceleration vector for a given point in time */
  Eigen::VectorXd getAcceleration(double time) const;

 private:
  struct TrajectoryStep {
    TrajectoryStep() {}
    TrajectoryStep(double path_pos, double path_vel)
        : path_pos_(path_pos), path_vel_(path_vel) {}
    double path_pos_;
    double path_vel_;
    double time_;
  };

  bool getNextSwitchingPoint(double path_pos,
                             TrajectoryStep& next_switching_point,
                             double& before_acceleration,
                             double& after_acceleration);
  bool getNextAccelerationSwitchingPoint(double path_pos,
                                         TrajectoryStep& next_switching_point,
                                         double& before_acceleration,
                                         double& after_acceleration);
  bool getNextVelocitySwitchingPoint(double path_pos,
                                     TrajectoryStep& next_switching_point,
                                     double& before_acceleration,
                                     double& after_acceleration);
  bool integrateForward(std::list<TrajectoryStep>& trajectory,
                        double acceleration);
  void integrateBackward(std::list<TrajectoryStep>& start_trajectory,
                         double path_pos, double path_vel, double acceleration);
  double getMinMaxPathAcceleration(double path_position, double path_velocity,
                                   bool max);
  double getMinMaxPhaseSlope(double path_position, double path_velocity,
                             bool max);
  double getAccelerationMaxPathVelocity(double path_pos) const;
  double getVelocityMaxPathVelocity(double path_pos) const;
  double getAccelerationMaxPathVelocityDeriv(double path_pos);
  double getVelocityMaxPathVelocityDeriv(double path_pos);

  std::list<TrajectoryStep>::const_iterator getTrajectorySegment(
      double time) const;

  Path path_;
  Eigen::VectorXd max_velocity_;
  Eigen::VectorXd max_acceleration_;
  unsigned int joint_num_;
  bool valid_;
  std::list<TrajectoryStep> trajectory_;
  std::list<TrajectoryStep>
      end_trajectory_;  // non-empty only if the trajectory generation failed.

  const double time_step_;

  mutable double cached_time_;
  mutable std::list<TrajectoryStep>::const_iterator cached_trajectory_segment_;
  py::object logger_;
};

}  // namespace time_optimal
}  // namespace motion