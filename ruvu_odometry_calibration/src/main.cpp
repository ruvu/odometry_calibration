#include "./bag_buffer.hpp"
#include "angles/angles.h"
#include "ros/console.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_msgs/TFMessage.h"

struct Odometry
{
  double linear;
  double angular;
};

/**
 * Given two transforms, reverse calculate the odometry of the wheels for a differential drive robot
 * @return A tuple of translation and rotation
 */
Odometry inverse_odometry(const tf2::Transform & a, const tf2::Transform & b)
{
  auto angular =
    angles::shortest_angular_distance(tf2::getYaw(a.getRotation()), tf2::getYaw(b.getRotation()));
  auto linear = (b.getOrigin() - a.getOrigin()).length();
  return Odometry{linear, angular};
}

tf2::Transform lookupTransform(
  const tf2::BufferCore & buffer, const std::string & target_frame,
  const std::string & source_frame, const ros::Time & time)
{
  tf2::Stamped<tf2::Transform> tf;
  auto msg = buffer.lookupTransform(target_frame, source_frame, time);
  tf2::fromMsg(msg, tf);
  return tf;
}

struct DataPoint
{
  tf2::Transform measured_transformation;
  tf2::Transform reference_transformation;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    ROS_INFO("usage: %s BAGFILE", argv[0]);
    return 1;
  }

  ROS_INFO("loading %s", argv[1]);
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  // load a tf buffer with all transforms
  BagBuffer buffer(bag);

  // load all measurements from the bagfile
  std::vector<sensor_msgs::LaserScanConstPtr> scans;
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery("/scan"))) {
    auto scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (scan == nullptr) throw std::runtime_error("Received scan message with the wrong type");
    scans.push_back(scan);
  }

  bag.close();

  //  tf2::Stamped<tf2::Transform> measurement;
  //  tf2::fromMsg(tf, measurement);
  //  auto p = measurement.getOrigin();
  //  measurements.push_back(measurement);

  std::vector<DataPoint> data_points;
  // iterate in pairs over the measurements
  auto a = scans.begin();
  auto b = a + 1;
  for (; b != scans.end(); a = b++) {
    auto ts1 = (*a)->header.stamp;
    auto ts2 = (*b)->header.stamp;

    try {
      auto measurement1 = lookupTransform(buffer, "odom", "base_link", ts1);
      auto reference1 = lookupTransform(buffer, "map", "base_link", ts1);
      auto measurement2 = lookupTransform(buffer, "odom", "base_link", ts2);
      auto reference2 = lookupTransform(buffer, "map", "base_link", ts2);

      data_points.push_back(
        DataPoint{measurement1.inverseTimes(measurement2), reference1.inverseTimes(reference2)});

      // auto odometry = inverse_odometry(measurement1, measurement2);
      // std::cout << odometry.linear << " " << odometry.angular << '\n';
    } catch (const tf2::ExtrapolationException &) {
      ROS_WARN("skipping timestamp");
      continue;
    }
  }

  return 0;
}
