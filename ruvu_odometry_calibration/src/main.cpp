#include <cstdio>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <angles/angles.h>

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

struct Odometry
{
  double linear;
  double angular;
};

Odometry inverse_odometry(const tf2::Stamped<tf2::Transform>& a, const tf2::Stamped<tf2::Transform>& b)
{
  auto angular = angles::shortest_angular_distance(tf2::getYaw(a.getRotation()), tf2::getYaw(b.getRotation()));
  auto linear = (b.getOrigin() - a.getOrigin()).length();
  return Odometry{ linear, angular };
}

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

  // load a tf buffer with all transforms
  BagBuffer buffer(bag);

  // load all measurements from the bagfile
  std::vector<tf2::Stamped<tf2::Transform>> measurements;
  for (const auto& msg : rosbag::View(bag, rosbag::TopicQuery("/tf")))
  {
    auto tfs = msg.instantiate<tf2_msgs::TFMessage>();
    if (tfs == nullptr)
      throw std::runtime_error("Received tf message with the wrong type");

    for (const auto& tf : tfs->transforms)
    {
      if (tf.header.frame_id == "odom")
      {
        if (tf.child_frame_id != "base_link")
          throw std::runtime_error("Received odom tf with the wrong child_frame_id");

        tf2::Stamped<tf2::Transform> measurement;
        tf2::fromMsg(tf, measurement);
        auto p = measurement.getOrigin();
        measurements.push_back(measurement);
      }
    }
  }

  bag.close();

  // iterate in pairs over the measurements
  auto a = measurements.begin();
  auto b = a + 1;
  for (; b != measurements.end(); a = b++)
  {
    auto odometry = inverse_odometry(*a, *b);
    std::cout << odometry.linear << " " << odometry.angular << '\n';
  }

  // TODO

  //      tf2::Stamped<tf2::Transform> measurement;
  //      try
  //      {
  //        tf2::fromMsg(buffer.lookupTransform("odom", "base_link", scan->header.stamp), measurement);
  //      }
  //      catch (const tf2::ExtrapolationException&)
  //      {
  //        ROS_WARN("skipping measurment because of unknown transform");
  //      }

  //      auto p = measurement.getOrigin();
  //      std::cout << p.getX() << " " << p.getY() << " " << p.getZ() << '\n';

  return 0;
}
