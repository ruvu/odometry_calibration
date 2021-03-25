#include <cstdio>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Duration duration_from_bag(const rosbag::Bag& bag)
{
  std::vector<ros::Time> ts;
  for (const auto& msg : rosbag::View(bag, rosbag::TopicQuery("/scan")))
  {
    auto scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (scan == nullptr)
      throw std::runtime_error("Received scan message with the wrong type");
    ts.push_back(scan->header.stamp);
  }

  ROS_INFO("Loaded %zu scans", ts.size());
  if (!ts.size())
    throw std::runtime_error("Can't find any scans in the bagfile");

  return ts.back() - ts.front();
}

class BagBuffer : public tf2_ros::Buffer
{
public:
  BagBuffer(const rosbag::Bag& bag) : Buffer(duration_from_bag(bag))
  {
    for (const auto& msg : rosbag::View(bag, rosbag::TopicQuery("/tf")))
    {
      auto tfs = msg.instantiate<tf2_msgs::TFMessage>();
      if (tfs == nullptr)
        throw std::runtime_error("Received tf message with the wrong type");
      for (auto& tf : tfs->transforms)
      {
        this->setTransform(tf, "bagfile");
      }
    }
  }
};

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    ROS_INFO("usage: %s BAGFILE", argv[0]);
    return 1;
  }

  ROS_INFO("loading %s", argv[1]);
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  BagBuffer buffer(bag);

  {
    for (const auto& msg : rosbag::View(bag, rosbag::TopicQuery("/scan")))
    {
      auto scan = msg.instantiate<sensor_msgs::LaserScan>();
      if (scan == nullptr)
        throw std::runtime_error("Received scan message with the wrong type");

      tf2::Stamped<tf2::Transform> measurement;
      try
      {
        tf2::fromMsg(buffer.lookupTransform("odom", "base_link", scan->header.stamp), measurement);
      }
      catch (const tf2::ExtrapolationException&)
      {
        ROS_WARN("skipping measurment because of unknown transform");
      }

      auto p = measurement.getOrigin();
      std::cout << p.getX() << " " << p.getY() << " " << p.getZ() << '\n';
    }
  }

  bag.close();

  return 0;
}
