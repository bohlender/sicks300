/*
 *
 * sicks300.cpp
 *
 *
 * Copyright (C) 2010
 * Autonomous Intelligent Systems Group
 * University of Bonn, Germany
 *
 *
 * Authors: Andreas Hochrath, Torsten Fiolka
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * Origin:
 *  Player - One Hell of a Robot Server
 *  serialstream.cc & sicks3000.cc
 *  Copyright (C) 2003
 *     Brian Gerkey
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 */

#include "sicks300.h"

#include "termios.h"

SickS300::SickS300()
{

  ros::NodeHandle param_node("~");
  ros::NodeHandle nodeHandle("/");

  int param;
  double x, y, z;

  // reading transformation parameters from parameter server
  param_node.param(std::string("frame"), scan_data_.header.frame_id, std::string("base_laser_link"));
  param_node.param(std::string("send_transform"), param, 1);
  if (param)
  {
    send_transform_ = true;
  }
  else
  {
    send_transform_ = false;
  }
  param_node.param(std::string("tf_x"), x, 0.115);
  param_node.param(std::string("tf_y"), y, 0.0);
  param_node.param(std::string("tf_z"), z, 0.21);

  transform_vector_ = tf::Vector3(x, y, z);

  // Setting full field of view (270 degree) or reduced (180 degree)
  param_node.param(std::string("reduced_fov"), param, 0);
  if (param != 0)
  {
    reduced_FOV_ = true;
    ROS_INFO("INFO: Starting Sick300-Laser with reduced field ov view of 180 degree");
  }
  else
  {
    reduced_FOV_ = false;
  }

  if (!reduced_FOV_)
  {
    scan_data_.angle_min = -135.f / 180.f * M_PI;
    scan_data_.angle_max = 135.f / 180.f * M_PI;
    scan_data_.angle_increment = 0.5f / 180.f * M_PI;
    scan_data_.time_increment = 0;
    scan_data_.scan_time = 0.08;
    scan_data_.range_min = 0.1;
    scan_data_.range_max = 29.0;
    scan_data_.ranges.resize(541);
    scan_data_.intensities.resize(541);
  }
  else
  {
    scan_data_.angle_min = -90.f / 180.f * M_PI;
    scan_data_.angle_max = 90.f / 180.f * M_PI;
    scan_data_.angle_increment = 0.5f / 180.f * M_PI;
    scan_data_.time_increment = 0;
    scan_data_.scan_time = 0.08;
    scan_data_.range_min = 0.1;
    scan_data_.range_max = 29.0;
    scan_data_.ranges.resize(361);
    scan_data_.intensities.resize(361);
  }

  // Reading device parameters
  param_node.param(std::string("devicename"), device_name_, std::string("/dev/sick300"));
  param_node.param(std::string("baudrate"), baud_rate_, 500000);

  connected_ = serial_comm_.connect(device_name_, baud_rate_);

  scan_data_publisher_ = nodeHandle.advertise<sensor_msgs::LaserScan> ("laserscan", 10);

}

SickS300::~SickS300()
{
}

void SickS300::update()
{

  if (connected_ != 0)
  {
    connected_ = serial_comm_.connect(device_name_, baud_rate_);
  }

  if (connected_ == 0 && serial_comm_.readData() == 0)
  {

    float* ranges = serial_comm_.getRanges();
    unsigned int numRanges = serial_comm_.getNumRanges();
    if (!reduced_FOV_)
    {
      scan_data_.ranges.resize(numRanges);
      for (unsigned int i = 0; i < numRanges; i++)
        scan_data_.ranges[i] = ranges[i];
    }
    else
    {
      for (unsigned int i = 0; i < 361; i++)
        scan_data_.ranges[i] = ranges[i + 89];
    }
    scan_data_.header.stamp = ros::Time::now();

    scan_data_publisher_.publish(scan_data_);

  }

}

void SickS300::broadcast_transform()
{
  if (send_transform_)
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), transform_vector_),
                                                      ros::Time::now(), "base_link", "base_laser_link"));
  }

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "sicks300");
  ros::Time::init();
  ros::Rate loop_rate(20);

  ROS_INFO("Opening connection to Sick300-Laser...");

  SickS300 sickS300;

  ROS_INFO("Sick300 connected.");

  while (ros::ok())
  {

    sickS300.update();
    sickS300.broadcast_transform();

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Laser shut down.");

  return 0;
}

