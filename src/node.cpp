/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2016 cocasema
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include "lidar_lite/driver.h"

#include <sensor_msgs/LaserScan.h>

#include <boost/lexical_cast.hpp>
#include <ros/ros.h>

namespace lidar_lite {

int
run(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_lite");

  std::string frame_id = "lidar_lite";
  int32_t i2c_bus = LidarLiteDriver::DEFAULT_I2C_BUS;
  uint8_t i2c_address = LidarLiteDriver::DEFAULT_I2C_ADDR;

  ros::NodeHandle nh;
  try {
    ros::NodeHandle nh_("~");
    nh_.param("frame_id", frame_id, frame_id);
    nh_.param("i2c_bus", i2c_bus, i2c_bus);

    std::string i2c_address_str = std::to_string(i2c_address);
    nh_.param("i2c_address", i2c_address_str, i2c_address_str);

    i2c_address = (uint8_t)std::stoul(i2c_address_str, nullptr, 0);

    ROS_INFO("Read params: {frame_id: %s, i2c_bus: %i, i2c_address: 0x%0x}",
             frame_id.c_str(), i2c_bus, i2c_address);
  }
  catch (boost::bad_lexical_cast const& ex) {
    ROS_ERROR("Failed to read params: %s", ex.what());
    return 1;
  }

  ros::Publisher publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 1024);

  LidarLiteDriver driver((uint8_t)i2c_bus, i2c_address);
  driver.configure(LidarLiteDriver::OperationMode::DEFAULT);

  while (ros::ok()) {
    auto distance = driver.distance(true);
    if (!distance) {
      continue;
    }

    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();

    float distance_in_meters = distance->value / 100.0;

    msg.angle_min = 0.;
    msg.angle_max = 0.;
    msg.angle_increment = 1.;
    msg.time_increment = 0.;
    //msg.scan_time = 0.;
    msg.range_min = 0.;
    msg.range_max = 40.;
    msg.ranges.push_back(distance_in_meters);
    msg.intensities.push_back(distance->value);

    publisher.publish(msg);
  }

  return 0;
}

} // namespace lidar_lite

int
main(int argc, char **argv)
{
  return lidar_lite::run(argc, argv);
}
