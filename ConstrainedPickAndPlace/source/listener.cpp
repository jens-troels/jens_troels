/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Cmd.h"
#include <iostream>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_client");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<wsg_50_common::Move>("wsg_50_driver/move");

    wsg_50_common::Move move_cmd;


    move_cmd.request.width = 20;
    move_cmd.request.speed = 50;


    if (client.call(move_cmd))
    {
      cout << "Succesfully called the request" << endl;
    }
    else
    {
      ROS_ERROR("Failed to call service");
      return 1;
    }

//    ros::ServiceClient homing_client = n.serviceClient<wsg_50_common::Cmd>("wsg_50_driver/homing");
//    wsg_50_common::Cmd cmd;
//    homing_client.call(cmd);


  return 0;
}
// %EndTag(FULLTEXT)%

