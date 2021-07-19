#include "./bag_buffer.hpp"

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_msgs/TFMessage.h"

ros::Duration duration_from_bag(const rosbag::Bag & bag)
{
  std::vector<ros::Time> ts;
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery("/scan"))) {
    auto scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (scan == nullptr) throw std::runtime_error("Received scan message with the wrong type");
    ts.push_back(scan->header.stamp);
  }

  ROS_INFO("Loaded %zu scans", ts.size());
  if (!ts.size()) throw std::runtime_error("Can't find any scans in the bagfile");

  return ts.back() - ts.front();
}

BagBuffer::BagBuffer(const rosbag::Bag & bag) : Buffer(duration_from_bag(bag))
{
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery("/tf"))) {
    auto tfs = msg.instantiate<tf2_msgs::TFMessage>();
    if (tfs == nullptr) throw std::runtime_error("Received tf message with the wrong type");
    for (auto & tf : tfs->transforms) {
      this->setTransform(tf, "bagfile");
    }
  }
}
