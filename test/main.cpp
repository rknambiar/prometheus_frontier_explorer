#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "prometheus");
  ros::NodeHandle node_handle;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();
  return RUN_ALL_TESTS();
}
