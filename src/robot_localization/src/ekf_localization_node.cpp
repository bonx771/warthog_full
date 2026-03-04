/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ros_filter_types.h" // Header này định nghĩa: RosEkf, RosUkf, các typedef liên quan filter, liên kết giữa ROS wrapper và EKF core

#include <cstdlib>

#include <ros/ros.h>
//argc = argument count; Số lượng tham số truyền vào khi chạy chương trình.
//argv = argument vector, Là mảng các chuỗi ký tự (string) chứa các tham số dòng lệnh.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_navigation_node");
  // Đăng ký node với ROS Master
  // Tên node là: ekf_navigation_node
  // Sau lệnh này mới có thể publish/subscribe.
  ros::NodeHandle nh;
  // Dùng để: Subscribe topic thường, Publish topic thường, Lấy param global
  ros::NodeHandle nh_priv("~");
  // NodeHandle private namespace
  RobotLocalization::RosEkf ekf(nh, nh_priv);
  // Object này chứa toàn bộ hệ thống EKF.
  ekf.initialize();
  // Trong hàm này sẽ: Load parameters, Set process noise
  // Set measurement configs, Setup subscribers, Setup timers
  ros::spin(); // vong lap

  return EXIT_SUCCESS;
}
