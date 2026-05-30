#include "LIVMapper.h"
#include <iostream>

int main(int argc, char **argv)
{
  // 【新增】强制终端即时输出，防止日志被吞
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  {
      rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("laserMapping", options);
      image_transport::ImageTransport it_(nh);
      
      LIVMapper mapper(nh, "laserMapping", options);
      mapper.initializeSubscribersAndPublishers(nh, it_);
      
      std::cout << "[MAIN] Mapper started, waiting for Ctrl+C..." << std::endl;
      mapper.run(nh);
      std::cout << "[MAIN] Run loop finished." << std::endl;
      // mapper 会被自动销毁，触发 ~LIVMapper() 析构函数
  } 

  std::cout << "[MAIN] ROS shutdown." << std::endl;
  rclcpp::shutdown();
  return 0;
}