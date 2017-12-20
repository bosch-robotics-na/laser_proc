/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/* 
 * Author: Chad Rockey
 */

#include <laser_proc/LaserProcROS.h>
#include <rclcpp/node.hpp>


namespace laser_proc
{

class LaserProcNodelet : public rclcpp::Node
{
public:
  LaserProcNodelet()
    :
    rclcpp::Node("laser_proc_nodelet")  {};

  ~LaserProcNodelet() {}

private:
  virtual void onInit()
  {
    auto pn = rclcpp::Node::make_shared("laser_proc_nodelet");
    lp.reset(new LaserProcROS(rclcpp::Node::SharedPtr(this), pn));
  };
  
  boost::shared_ptr<LaserProcROS> lp;
};

}

//#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(laser_proc, LaserProcNodelet, laser_proc::LaserProcNodelet, nodelet::Nodelet);
#include "class_loader/class_loader_register_macro.h"
CLASS_LOADER_REGISTER_CLASS(laser_proc::LaserProcNodelet, rclcpp::Node);

