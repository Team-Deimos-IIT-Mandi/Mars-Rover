/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
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
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
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
 * Author: Mark Naeem
 */

#include <swerve_steering_controller/utils.h>
#include <swerve_steering_controller/interval.h>
#include <cmath>
#include <algorithm>

class wheel
{
    double theta_ = 0;
    double command_omega_ = 0;
    double steering_angle_= 0;
    int omega_direc_ = 1;
    bool limitless_ = false;
    interval limits_;

    public:
        wheel(const double radius = 0.0, const std::array<double,2>& position = {0.0,0.0}, bool limitless=false , const std::array<double,2>& rotation_limits = {-M_PI_2,M_PI_2});


        void set_rotation_limits(const std::array<double,2>& rotation_limits = {-M_PI_2,M_PI_2}, bool limitless = false);

        void   set_current_angle(const double& angle);    
        double get_current_angle() const;    

        bool set_command_angle(const double& target);
        double get_command_angle() const;    

        void set_command_velocity(const double& velocity);    
        double get_command_velocity() const;    

        void set_position(const std::array<double,2>& position);

        void set_limitless(const bool limitless);
        bool get_limitless() const;

        int get_omega_direction() const;
        //for the controller use only. THis class wont use them by itself
        double offset;
        double radius;
        std::array<double,2> position;

};