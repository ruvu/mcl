/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Open Source Robotics Foundation
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include "./bag_player.hpp"

#include <string>
#include <vector>

#include "ros/init.h"
#include "rosbag/view.h"

BagPlayer::BagPlayer(const std::string & filename)
{
  bag.open(filename, rosbag::bagmode::Read);
  ros::Time::init();
  rosbag::View v(bag);
  bag_start_ = v.getBeginTime();
  bag_end_ = v.getEndTime();
  last_message_time_ = ros::Time(0);
  playback_speed_ = 1.0;
}

void BagPlayer::set_playback_speed(double scale)
{
  if (scale > 0.0) playback_speed_ = scale;
}

ros::Time BagPlayer::real_time(const ros::Time & msg_time) const
{
  return play_start_ + (msg_time - bag_start_) * (1 / playback_speed_);
}

void BagPlayer::start_play()
{
  std::vector<std::string> topics;
  for (const auto & cb : cbs_) topics.push_back(cb.first);

  rosbag::View view(bag, rosbag::TopicQuery(topics), bag_start_, bag_end_);
  play_start_ = ros::Time::now();

  for (rosbag::MessageInstance const & m : view) {
    if (!ros::ok()) break;

    if (cbs_.find(m.getTopic()) == cbs_.end()) continue;

    ros::Time::sleepUntil(real_time(m.getTime()));
    ros::spinOnce();  // handle dynamic reconfigure calls

    last_message_time_ = m.getTime(); /* this is the recorded time */
    auto cb = cbs_[m.getTopic()];
    cb(m);
  }
}
