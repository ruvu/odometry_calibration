#include <cstdio>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/TFMessage.h>

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

  printf("Loaded %zu scans", ts.size());
  if (!ts.size())
    throw std::runtime_error("Can't find any scans in the bagfile");

  return ts.back() - ts.front();
}

class BagBuffer : tf2_ros::Buffer
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
    printf("usage: %s BAGFILE", argv[0]);
    return 1;
  }

  printf("loading %s", argv[1]);
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  BagBuffer buffer(bag);

  bag.close();

  return 0;
}
