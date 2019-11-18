/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 Suyash Yeotikar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  @file turtlebot_walker.cpp
 *  @date Nov 17, 2019
 *  @author Suyash Yeotikar
 *  @brief main file
 *  @mainpage project page
 */


#include "turtlebot_walk.hpp"

void TurtlebotWalk::DetectObstacleCallback(const sensor_msgs::
                                           ImageConstPtr &msD) {
    cv_bridge::CvImageConstPtr cv_ptrD;
    cv_ptrD = cv_bridge::toCvShare(msD);
    for ( int i = 0; i < cv_ptrD->image.rows; i++ ) {
        for ( int j = 0; j < cv_ptrD->image.cols; j++ ) {
            if ( cv_ptrD->image.at<ushort>(i, j)
                    < distanceThreshold ) {
                obstacleDetected = true;
                break;
            }
        }
    }
}

geometry_msgs::Twist TurtlebotWalk::TurtlebotPlanner() {
    geometry_msgs::Twist velCommand;
    if ( obstacleDetected ) {
        velCommand.angular.z = robotAngularVelocity;
        velCommand.linear.x = 0;
        velCommand.linear.y = 0;
        velCommand.linear.z = 0;
        velCommand.angular.x = 0;
        velCommand.angular.y = 0;
    } else {
        velCommand.angular.z = 0;
        velCommand.angular.x = 0;
        velCommand.angular.y = 0;
        velCommand.linear.x = robotLinearVelocity;
        velCommand.linear.y = 0;
        velCommand.linear.z = 0;
    }
    return velCommand;
}

