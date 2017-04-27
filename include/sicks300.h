/*
 *
 * sicks300.h
 *
 *
 * Copyright (C) 2014
 * Software Engineering Group
 * RWTH Aachen University
 *
 *
 * Author: Dimitri Bohlender
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
 *  Copyright (C) 2010
 *     Andreas Hochrath, Torsten Fiolka
 *     Autonomous Intelligent Systems Group
 *     University of Bonn, Germany
 *
 *  Player - One Hell of a Robot Server
 *  serialstream.cc & sicks3000.cc
 *  Copyright (C) 2003
 *     Brian Gerkey
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 */

#ifndef __SICKS300_H__
#define __SICKS300_H__

#include "serialcomm_s300.h"

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

/**
 * \class SickS300
 * @brief uses SerialCommS300 to connect to Sick S300 laserscanner, read data and publishes
 * to a laserscan-topic. It also includes a transform publisher to send the appropriate
 * transform for the laser
 */

class SickS300 {
public:

  SickS300();
  ~SickS300();

  //! Sending laserscan
  void update();

  //! Broadcasting transform, if enabled
  void broadcast_transform();

protected:

  //! The underlying communications to the laser
  SerialCommS300 serial_comm_;

  sensor_msgs::LaserScan scan_data_;
  ros::Publisher scan_data_publisher_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::Vector3 transform_vector_;

  //! Scan field of view parameters
  unsigned int field_of_view_;
  unsigned int start_scan_;
  unsigned int end_scan_;

  ros::NodeHandle param_node_;
  ros::NodeHandle nodeHandle_;
  std::string connect_cmd_;

  //! Send Transform or not
  bool send_transform_;

  std::string device_name_;
  unsigned int baud_rate_;
  int connected_;
  std::string frame_id_;
};

#endif // __SICKS300_H__
