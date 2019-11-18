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
 *  @file turtlebot_walk.hpp
 *  @date Nov 17, 2019
 *  @author Suyash Yeotikar
 *  @brief main file
 *  @mainpage project page
 *  Please refer the talker.cpp file in file section
 *  and function members sections for detailed documentation
 */

#ifndef INCLUDE_TURTLEBOT_WALK_TURTLEBOT_WALK_HPP_
#define INCLUDE_TURTLEBOT_WALK_TURTLEBOT_WALK_HPP_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/core/core.hpp>

/*
 * @class TurtlebotWalk
 * @ingroup turtlebot_walk
 * @brief Class declaration for turtlebot walker
 */
class TurtlebotWalk {
 public:
    bool obstacleDetected = false;
    float distanceThreshold = 50;
    float robotLinearVelocity = 0.25;
    float robotAngularVelocity = 0.25;
    ros::NodeHandle n;
    ros::Subscriber depthImageSubscriber;
    ros::Publisher velPublisher;
 private:
    /*
     * @brief method to detect obstacles using depth camera
     * @param msD Depth image message
     * @return none
     */
    void DetectObstacleCallback(const sensor_msgs::ImageConstPtr &msD);


    /*
     * @brief method to make the turtlebot move forward and turn around if obstacle is found
     * @param none
     * @return none
     */
    void TurtlebotPlanner();
};
#endif  // INCLUDE_TURTLEBOT_WALK_TURTLEBOT_WALK_HPP_
