/*
 *
 * serialcomm_s300.h
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

#ifndef __SERIALCOMMS300_H__
#define __SERIALCOMMS300_H__

#include <string>

#define RX_BUFFER_SIZE 4096
#define DEFAULT_SERIAL_PORT "/dev/sick300"
#define DEFAULT_BAUD_RATE 500000

/**
 * \class SerialCommS300
 * @brief connects to a Sick S300 laserscanner
 */

class SerialCommS300
{
public:

  SerialCommS300();
  ~SerialCommS300();

  // returns 0 if new laser data has arrived
  int readData();

  inline unsigned int getNumRanges()
  {
    return m_rangesCount;
  }
  inline float* getRanges()
  {
    return m_ranges;
  }

  int connect(const std::string& deviceName, unsigned int baudRate = DEFAULT_BAUD_RATE);
  int disconnect();

private:

  void setFlags();

  int setBaudRate(int baudRate);
  int baudRateToBaudCode(int baudCode);

  unsigned short createCRC(unsigned char* data, ssize_t len);

protected:

  unsigned char m_rxBuffer[RX_BUFFER_SIZE];

  int m_fd;

  int m_rxCount;

  float* m_ranges;
  unsigned int m_rangesCount;

};

#endif // __SERIALCOMMS300_H__

