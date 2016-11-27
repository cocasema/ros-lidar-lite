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

#include <ros/ros.h>

namespace lidar_lite {

int
run(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_lite_node");
  ros::NodeHandle nh;

  std::string frame_id = "lidar_lite";
  int32_t i2c_bus = LidarLiteDriver::DEFAULT_I2C_BUS;
  int32_t i2c_address = LidarLiteDriver::DEFAULT_I2C_ADDR;

  nh.param("frame_id", frame_id, frame_id);
  nh.param("i2c_bus", i2c_bus, i2c_bus);
  nh.param("i2c_address", i2c_address, i2c_address);

  ros::Publisher publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 1024);

  LidarLiteDriver driver((uint8_t)i2c_bus, (uint8_t)i2c_address);

  while (ros::ok()) {
    auto distance = driver.distance(false);
    if (!distance) {
      continue;
    }

    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();

    msg.angle_min = 0.;
    msg.angle_max = 0.;
    msg.angle_increment = 1.;
    msg.time_increment = 0.;
    msg.range_min = 0.001;
    msg.range_max = 50. * 100;
    msg.ranges.push_back(distance->value);

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
