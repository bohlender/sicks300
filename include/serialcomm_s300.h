/*
 *
 * serialcomm_s300.h
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

#ifndef __SERIALCOMMS300_H__
#define __SERIALCOMMS300_H__

#include <string>
#include <cstddef>
#include <unistd.h>
#include <ros/ros.h>

#define RX_BUFFER_SIZE 4096
#define DEFAULT_SERIAL_PORT "/dev/sick300"
#define DEFAULT_BAUD_RATE 500000
#define PROTOCOL_1_02 0x0102
#define PROTOCOL_1_03 0x0103

/**
 * \class SerialCommS300
 * @brief connects to a Sick S300 laserscanner
 */

class SerialCommS300 {
public:

  SerialCommS300();
  ~SerialCommS300();

  // returns 0 if new laser data has arrived
  int readData();

  inline unsigned int getNumRanges() {
    return m_rangesCount;
  }

  inline float *getRanges() {
    return m_ranges;
  }

  inline unsigned int getScanNumber() {
    return m_scanNumber;
  }

  inline unsigned int getTelegramNumber() {
    return m_telegramNumber;
  }

  inline ros::Time getReceivedTime() {
    return m_receivedTime;
  }

  int connect(const std::string &deviceName, unsigned int baudRate = DEFAULT_BAUD_RATE);
  int disconnect();

private:

  void setFlags();
  int setBaudRate(int baudRate);
  int baudRateToBaudCode(int baudCode);

  unsigned short createCRC(unsigned char *data, ssize_t len);

  // discards read bytes from the buffer
  void discard_byte(unsigned int count = 1);

  // read bytes into buffer
  int read_byte(unsigned int count = 1);

protected:

  unsigned char m_rxBuffer[RX_BUFFER_SIZE];
  int m_fd;
  size_t m_rxCount;

  int zerobytesread_counter;

  float *m_ranges;
  unsigned int m_rangesCount;

  unsigned int m_scanNumber;
  unsigned int m_telegramNumber;
  ros::Time m_receivedTime;

};

#endif // __SERIALCOMMS300_H__

