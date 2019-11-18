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
 *  @file main.cpp
 *  @date Nov 17, 2019
 *  @author Suyash Yeotikar
 *  @brief main file
 *  @mainpage project page
 */

#include "turtlebot_walk.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");
    TurtlebotWalk tbw;
    tbw.depthImageSubscriber = tbw.n.subscribe("/camera/depth/image_raw",
                                 1, &TurtlebotWalk::DetectObstacleCallback,
                                 &tbw);
    tbw.velPublisher = tbw.n.advertise<geometry_msgs::
            Twist>("/mobile_base/commands/velocity", 1);
    ros::Rate loopRate(10);
    while ( ros::ok() ) {
        tbw.velPublisher.publish(tbw.TurtlebotPlanner());
        ros::spinOnce();
        loopRate.sleep();
    }
}
