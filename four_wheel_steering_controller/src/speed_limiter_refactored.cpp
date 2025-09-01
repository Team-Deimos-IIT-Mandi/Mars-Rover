/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, PAL Robotics, S.L.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the PAL Robotics nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Enrique Fernández
 */

#include <algorithm>
#include <four_wheel_steering_controller/speed_limiter.h>

namespace four_wheel_steering_controller
{

  /**
   * @brief Constructs a SpeedLimiter object.
   * @param has_velocity_limits A flag indicating if velocity limits should be applied.
   * @param has_acceleration_limits A flag indicating if acceleration limits should be applied.
   * @param has_jerk_limits A flag indicating if jerk limits should be applied.
   * @param min_velocity The minimum velocity limit.
   * @param max_velocity The maximum velocity limit.
   * @param min_acceleration The minimum acceleration limit.
   * @param max_acceleration The maximum acceleration limit.
   * @param min_jerk The minimum jerk limit.
   * @param max_jerk The maximum jerk limit.
   */
  SpeedLimiter::SpeedLimiter(
    bool has_velocity_limits,
    bool has_acceleration_limits,
    bool has_jerk_limits,
    double min_velocity,
    double max_velocity,
    double min_acceleration,
    double max_acceleration,
    double min_jerk,
    double max_jerk
  )
  : has_velocity_limits_(has_velocity_limits),
    has_acceleration_limits_(has_acceleration_limits),
    has_jerk_limits_(has_jerk_limits),
    min_velocity_(min_velocity),
    max_velocity_(max_velocity),
    min_acceleration_(min_acceleration),
    max_acceleration_(max_acceleration),
    min_jerk_(min_jerk),
    max_jerk_(max_jerk)
  {
  }

  /**
   * @brief Applies all limits (jerk, acceleration, and velocity) sequentially.
   * @param v The velocity value to be limited (passed by reference).
   * @param v0 The previous velocity value.
   * @param v1 The velocity value from two steps ago.
   * @param dt The time step.
   * @return The ratio of the new velocity to the original velocity.
   */
  double SpeedLimiter::limit(double& v, double v0, double v1, double dt)
  {
    const double original_v = v;

    limit_jerk(v, v0, v1, dt);
    limit_acceleration(v, v0, dt);
    limit_velocity(v);

    return original_v != 0.0 ? v / original_v : 1.0;
  }

  /**
   * @brief Limits the velocity to the configured min/max range.
   * @param v The velocity value to be limited (passed by reference).
   * @return The ratio of the new velocity to the original velocity.
   */
  double SpeedLimiter::limit_velocity(double& v)
  {
    const double original_v = v;

    if (has_velocity_limits_)
    {
      v = std::clamp(v, min_velocity_, max_velocity_);
    }

    return original_v != 0.0 ? v / original_v : 1.0;
  }

  /**
   * @brief Limits the acceleration to the configured min/max range.
   * @param v The velocity value to be limited (passed by reference).
   * @param v0 The previous velocity value.
   * @param dt The time step.
   * @return The ratio of the new velocity to the original velocity.
   */
  double SpeedLimiter::limit_acceleration(double& v, double v0, double dt)
  {
    const double original_v = v;

    if (has_acceleration_limits_)
    {
      const double dv_min = min_acceleration_ * dt;
      const double dv_max = max_acceleration_ * dt;

      const double dv = std::clamp(v - v0, dv_min, dv_max);
      v = v0 + dv;
    }

    return original_v != 0.0 ? v / original_v : 1.0;
  }

  /**
   * @brief Limits the jerk (change in acceleration) to the configured min/max range.
   * @param v The velocity value to be limited (passed by reference).
   * @param v0 The previous velocity value.
   * @param v1 The velocity value from two steps ago.
   * @param dt The time step.
   * @return The ratio of the new velocity to the original velocity.
   */
  double SpeedLimiter::limit_jerk(double& v, double v0, double v1, double dt)
  {
    const double original_v = v;

    if (has_jerk_limits_)
    {
      const double dv  = v - v0;
      const double dv0 = v0 - v1;
      const double dt2 = dt * dt;

      const double da_min = min_jerk_ * dt2;
      const double da_max = max_jerk_ * dt2;

      const double da = std::clamp(dv - dv0, da_min, da_max);
      v = v0 + dv0 + da;
    }

    return original_v != 0.0 ? v / original_v : 1.0;
  }

} // namespace four_wheel_steering_controller