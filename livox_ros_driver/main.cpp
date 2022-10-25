#include "include/livox_ros_driver.h"

using namespace livox_ros;

// const int32_t kSdkVersionMajorLimit = 2;


// inline void SignalHandler(int signum)
// {
//   printf("livox ros driver will exit\r\n");
//   // ros::shutdown();
//   exit(signum);
// }


int main(int argc, char **argv)
{
  // /** Ros related */
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //                                    ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  // ros::init(argc, argv, "livox_lidar_publisher");
  // ros::NodeHandle nh;

  // ROS_INFO("Livox Ros Driver Version: %s", LIVOX_ROS_DRIVER_VERSION_STRING);
  // signal(SIGINT, SignalHandler);
  // /** Check sdk version */
  // LivoxSdkVersion _sdkversion;
  // GetLivoxSdkVersion(&_sdkversion);
  // if (_sdkversion.major < kSdkVersionMajorLimit)
  // {
  //   ROS_INFO("The SDK version[%d.%d.%d] is too low", _sdkversion.major,
  //            _sdkversion.minor, _sdkversion.patch);
  //   return 0;
  // }

  // /** Init default system parameter */
  // LddcConfig config;
  // nh.getParam("xfer_format", config.xfer_format);
  // nh.getParam("multi_topic", config.multi_topic);
  // nh.getParam("data_src", config.data_src);
  // nh.getParam("publish_freq", config.publish_freq);
  // nh.getParam("output_data_type", config.output_type);
  // nh.getParam("frame_id", config.frame_id);
  // nh.getParam("enable_lidar_bag", config.lidar_bag);
  // nh.getParam("enable_imu_bag", config.imu_bag);
  // if (config.publish_freq > 100.0)
  // {
  //   config.publish_freq = 100.0;
  // }
  // else if (config.publish_freq < 0.1)
  // {
  //   config.publish_freq = 0.1;
  // }
  // else
  // {
  //   config.publish_freq = config.publish_freq;
  // }

  // nh.getParam("user_config_path", config.user_config_path);
  // ROS_INFO("Config file : %s", config.user_config_path.c_str());

  // std::string cmdline_bd_code;
  // nh.getParam("cmdline_str", cmdline_bd_code);

  // ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), config.bd_code_list);

  // livox_node.reset(new LivoxNode());
  // livox_node->ResetLidar(config, &nh);

  // ros::Time::init();
  // double poll_freq = config.publish_freq;
  // if (config.data_src == kSourceLvxFile)
  // {
  //   poll_freq = 2000;
  // }
  // // ros::Rate r(poll_freq);
  // ros::Rate r(1);
  // int counter = 0;
  // while (ros::ok())
  // {
  //   if (livox_node->lddc->lds_->IsModeRestarting())
  //   {
  //     livox_node.reset();
  //     sleep(5);
  //     livox_node.reset(new LivoxNode());
  //     livox_node->ResetLidar(config, &nh);
  //   }
  //   livox_node->lddc->DistributeLidarData();
  //   r.sleep();
  //   counter++;
  //   printf("counter %d \n", counter);
  std::shared_ptr<LivoxNode> livox_node;
  livox_node.reset(new LivoxNode(argc, argv));
  sleep(3);

  // int counter = 0;
  // while(true)
  // {
  //   // livox_node.reset();
  //   // sleep(3);
  //   livox_node.reset(new LivoxNode(argc, argv));
  //   // sleep(3);
  //   // counter++;
  //   // printf("livox node restart times: %d \n", counter);
  // }

  return 0;
}