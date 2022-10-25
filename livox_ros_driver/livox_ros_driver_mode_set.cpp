//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "include/livox_ros_driver.h"

using namespace livox_ros;

const int32_t kSdkVersionMajorLimit = 2;

LivoxNode::LivoxNode(int argc, char **argv)
{
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "livox_lidar_publisher");
  ros::NodeHandle nh;

  ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
  signal(SIGINT, LivoxNode::SignalHandler);
  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit)
  {
    ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
             _sdkversion.minor, _sdkversion.patch);
    return ;
  }

  /** Init default system parameter */
  LddcConfig config;
  nh.getParam("xfer_format", config.xfer_format);
  nh.getParam("multi_topic", config.multi_topic);
  nh.getParam("data_src", config.data_src);
  nh.getParam("publish_freq", config.publish_freq);
  nh.getParam("output_data_type", config.output_type);
  nh.getParam("frame_id", config.frame_id);
  nh.getParam("enable_lidar_bag", config.lidar_bag);
  nh.getParam("enable_imu_bag", config.imu_bag);
  if (config.publish_freq > 100.0)
  {
    config.publish_freq = 100.0;
  }
  else if (config.publish_freq < 0.1)
  {
    config.publish_freq = 0.1;
  }
  else
  {
    config.publish_freq = config.publish_freq;
  }

  nh.getParam("user_config_path", config.user_config_path);
  ROS_INFO("Config file : %s", config.user_config_path.c_str());

  std::string cmdline_bd_code;
  nh.getParam("cmdline_str", cmdline_bd_code);

  ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), config.bd_code_list);

  // livox_node.reset(new LivoxNode());
  // livox_node->ResetLidar(config, &nh);
  ResetLidar(config, &nh);
}
