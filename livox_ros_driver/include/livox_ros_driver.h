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

#ifndef LIVOX_ROS_DRIVER_INClUDE_LIVOX_ROS_DRIVER_H_
#define LIVOX_ROS_DRIVER_INClUDE_LIVOX_ROS_DRIVER_H_

#define LIVOX_ROS_DRIVER_VER_MAJOR 2
#define LIVOX_ROS_DRIVER_VER_MINOR 6
#define LIVOX_ROS_DRIVER_VER_PATCH 0

#define GET_STRING(n) GET_STRING_DIRECT(n)
#define GET_STRING_DIRECT(n) #n

#define LIVOX_ROS_DRIVER_VERSION_STRING                      \
  GET_STRING(LIVOX_ROS_DRIVER_VER_MAJOR)                     \
  "." GET_STRING(LIVOX_ROS_DRIVER_VER_MINOR) "." GET_STRING( \
      LIVOX_ROS_DRIVER_VER_PATCH)

#endif

#include <chrono>
#include <vector>
#include <csignal>

#include <ros/ros.h>
#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"

using namespace livox_ros;

struct LddcConfig
{
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag = false;

  std::string user_config_path;
  std::vector<std::string> bd_code_list;
};

class LivoxNode
{
public:
  std::shared_ptr<Lddc> lddc;
  LdsLidar *read_lidar;
  std::shared_ptr<std::thread> t_mode_seeting;

  LivoxNode(int argc, char **argv);
  ~LivoxNode() = default;

  inline void ResetLidar(LddcConfig &config, ros::NodeHandle *nh)
  {
    lddc.reset(new Lddc(config.xfer_format, config.multi_topic, config.data_src, config.output_type,
                        config.publish_freq, config.frame_id, config.lidar_bag, config.imu_bag));
    lddc->SetRosNode(nh);

    read_lidar = LdsLidar::GetInstance(1000 / config.publish_freq);
    lddc->RegisterLds(static_cast<Lds *>(read_lidar));
    int ret = read_lidar->InitLdsLidar(config.bd_code_list, config.user_config_path.c_str());

    // int ret = ResetLddc(config, &nh);

    if (!ret)
    {
      ROS_INFO("Init lds lidar success!");
    }
    else
    {
      ROS_ERROR("Init lds lidar fail!");
      return;
    }
    t_mode_seeting.reset(new std::thread(&LivoxNode::ModeSettingThread, this));
    sleep(5);

    ros::Time::init();
    double poll_freq = config.publish_freq;
    if (config.data_src == kSourceLvxFile)
    {
      poll_freq = 2000;
    }
    // ros::Rate r(poll_freq);
    ros::Rate r(1);

    int counter = 0;
    while (ros::ok())
    {
      if (lddc->lds_->IsModeRestarting())
      {
        // sleep(5);
        break;
      }
      lddc->DistributeLidarData();
      r.sleep();
      counter++;
      printf("counter %d \n", counter);
    }
  }

  inline void ModeSettingThread()
  {
    int command;
    printf("Please input target mode: \n N or n : NormalWorking\n P or p : PowerSaving\n S or s : Standby\n Q or q : Quit\n");
    while ((command = getchar()) != EOF && lddc != nullptr)
    // while ((command = getchar()) != EOF)
    {
      getchar();
      // LidarMode mode_target = kLidarModeNormal;
      if (command == 'N' || command == 'n')
      {
        // mode_target = kLidarModeNormal;
        lddc->lds_->SetLidarModeNormal();
        printf("Target mode: Normal \n");
      }
      else if (command == 'P' || command == 'p')
      {
        // mode_target = kLidarModePowerSaving;
        while (lddc->is_grab_)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        lddc->lds_->SetLidarModePowerSaving();
        printf("Target mode: PowerSaving \n");
      }
      else if (command == 'S' || command == 's')
      {
        printf("Target mode: Standby \n");
        // mode_target == kLidarModeStandby;
        lddc->lds_->SetLidarModeNormal();
      }
      else if (command == 'Q' || command == 'q')
      {
        break;
      }
      else
      {
        printf("Error command, please try again \n");
        continue;
      }
    }

    printf("ModeSettingThread stop\n");
  }

  inline static void SignalHandler(int signum)
  {
    printf("livox ros driver will exit\r\n");
    ros::shutdown();
    exit(signum);
  }

  inline void TestCb(livox_status status, uint8_t handle, uint8_t response, void *client_data)
  {
    printf("status: %d.\n", status);
    printf("handle: %d.\n", handle);
    printf("response: %d.\n", response);
  }
};
