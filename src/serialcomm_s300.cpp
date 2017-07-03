/*
 *
 * serialcomm_s300.cpp
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

#include "serialcomm_s300.h"

#include <assert.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <iostream>
#include <ros/console.h>

SerialCommS300::SerialCommS300() {
  m_rxCount = 0;
  m_ranges = NULL;
}

SerialCommS300::~SerialCommS300() {
  if (m_ranges) {
    delete[] m_ranges;
  }
  disconnect();
}

int SerialCommS300::connect(const std::string &deviceName, unsigned int baudRate) {

  m_fd = ::open(deviceName.c_str(), O_RDWR | O_NOCTTY);
  if (m_fd < 0) {
    ROS_ERROR_STREAM("SerialCommS300: unable to open serial port " << deviceName);
    return -1;
  }

  setFlags();
  if (setBaudRate(baudRateToBaudCode(baudRate))) {
    return -1;
  }

  zerobytesread_counter = 0;

  tcflush(m_fd, TCIOFLUSH);
  usleep(1000);
  tcflush(m_fd, TCIFLUSH);

  return 0;
}

void SerialCommS300::setFlags() {

  // set up new settings
  struct termios term;
  memset(&term, 0, sizeof(term));

  tcgetattr(m_fd, &term);

  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 5;
  term.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;
  term.c_iflag = INPCK;
  term.c_oflag = 0;
  term.c_lflag = 0;

  if (tcsetattr(m_fd, TCSANOW, &term) < 0)
    ROS_ERROR("SerialCommS300: failed to set tty device attributes");
  tcflush(m_fd, TCIOFLUSH);
}

int SerialCommS300::setBaudRate(int baudRate) {

  struct termios term;

  if (tcgetattr(m_fd, &term) < 0) {
    ROS_ERROR("SerialCommS300: unable to get device attributes");
    return 1;
  }

  if (cfsetispeed(&term, baudRate) < 0 || cfsetospeed(&term, baudRate) < 0) {
    ROS_ERROR_STREAM("SerialCommS300: failed to set serial baud rate " << baudRate);
    return 1;
  }

  if (tcsetattr(m_fd, TCSAFLUSH, &term) < 0) {
    ROS_ERROR("SerialCommS300: unable to set device attributes");
    return 1;
  }

  return 0;
}

int SerialCommS300::baudRateToBaudCode(int baudRate) {
  switch (baudRate) {
    case 38400:
      return B38400;
    case 115200:
      return B115200;
    case 500000:
      return B500000;
    default:
      return B500000;
  };
}

int SerialCommS300::disconnect() {
  ::close(m_fd);
  return 0;
}

static const unsigned short crc_table[256] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
                                              0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
                                              0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
                                              0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
                                              0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
                                              0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
                                              0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
                                              0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
                                              0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
                                              0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
                                              0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
                                              0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
                                              0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
                                              0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
                                              0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
                                              0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
                                              0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
                                              0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
                                              0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
                                              0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
                                              0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
                                              0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
                                              0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
                                              0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
                                              0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
                                              0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
                                              0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
                                              0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
                                              0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

unsigned short SerialCommS300::createCRC(unsigned char *data, ssize_t length) {

  unsigned short CRC_16 = 0xFFFF;
  unsigned short i;

  for (i = 0; i < length; i++) {
    CRC_16 = (CRC_16 << 8) ^ (crc_table[(CRC_16 >> 8) ^ (data[i])]);
  }

  return CRC_16;
}

//
// Data packet structure, as documented in
// https://www.sick.com/media/dox/2/92/892/Telegram_listing_S3000_Standard_Advanced_Professional_Remote_S300_Standard_Advanced_Professional_de_en_IM0022892.PDF
//
// Addr(dec)  Description
// <<<<       the packet begins here
// 0..3       0x00 00 00 00
// <<<<       the telegram size is measured from here when the version of the protocol is 1.02
// 4..5       0x00 00
// 6..7       telegram size, expressed in 2-byte words
// 8..9       0xff07 "cooordination flag, device address"
// 10..11     protocol version, 0x0301
// 12..13     status - 0x0000 normal, 0x0001 lockout
// <<<<       the telegram size is measured from here when the version of the protocol is 1.03
// 14..17     scan number
// 18..19     telegram number
// 20..21     type id of the block, twice (0xbb for continous data streaming block)
// 22..23     0x1111 - complete scan range
// <<<<       end of header
// 24..1105   541 x 2-word measurements
// 1106..1107 CRC-16
// <<<<       packet ends here

// therefore:
const unsigned int telegram_size_location = 6;
const unsigned int protocol_version_number_location = 10;
const unsigned int scan_number_location = 14;
const unsigned int telegram_number_location = 18;
const unsigned int block_type_id_location = 20;
const unsigned int header_size = 24;

// uncounted_header_part + telegram size = total packet size (1108)
const unsigned int uncounted_header_part_0103 = 14;
const unsigned int uncounted_header_part_0102 = 4;

// returns -1 if a packet has not been successfully read (soft failure, i.e. we need to try again
// when the rest of the bytes of the sensor message arrive); returns -2 on serious failure e.g.
// the interface handle is invalid and we should try reconnecting
int SerialCommS300::readData() {

  int available_bytes;
  ioctl(m_fd, FIONREAD, &available_bytes);
  if (available_bytes > 1200) {
    ROS_WARN("SerialCommS300: more than one packet is buffered, measurements are potentially stale!");
  }

  // if we do not have at least a header in buffer, read it
  if (m_rxCount < header_size) {
    ROS_DEBUG("SerialCommS300: Reading header bytes");
    // read up to a packet header
    int status = read_byte((unsigned int) (header_size - m_rxCount));
    m_receivedTime = ros::Time::now();
    if (status != 0) { // failure
      return status;
    }
  }

  while (m_rxCount >= header_size) {

    // find our continuous data header
    unsigned int packet_start;
    bool found_header = false;

    // stop search if there is less then a complete header in the buffer left
    for (packet_start = 0; packet_start <= m_rxCount - header_size && !found_header; ++packet_start) {
      if (memcmp(&m_rxBuffer[packet_start], "\0\0\0\0\0\0", 6) == 0 &&
          memcmp(&m_rxBuffer[packet_start + 8], "\xFF", 1) == 0) {

        if (packet_start > 0) {
          ROS_WARN("SerialCommS300: Warning: skipping %u bytes, header found", packet_start);
          discard_byte(packet_start);
        }
        found_header = true;
      }
    }

    if (!found_header) {
      ROS_WARN("SerialCommS300: Warning: skipping %u bytes, header not found", packet_start);

      // packet_start is now pointing at least header_size bytes before the end of the buffer
      discard_byte(packet_start);

      // give up for now
      return -1;
    }

    // get the relevant bits of the header
    // size includes all data from the data block number
    // through to the end of the packet including the checksum
    unsigned short size = short(2) * htons(*reinterpret_cast<unsigned short *> (&m_rxBuffer[telegram_size_location]));
    unsigned int total_packet_size;

    // determine which protocol is used
    unsigned short protocol = *reinterpret_cast<unsigned short *> (&m_rxBuffer[protocol_version_number_location]);

    // read & calculate the total packet size according to the protocol
    unsigned short packet_checksum, calc_checksum;
    if (protocol == PROTOCOL_1_02) {
      total_packet_size = size + uncounted_header_part_0102;
    } else if (protocol == PROTOCOL_1_03) {
      total_packet_size = size + uncounted_header_part_0103;
    } else {
      ROS_ERROR_STREAM("SerialCommS300: protocol version " << std::hex << protocol << std::dec << " is unsupported");
      // perhaps we can recover by discarding a byte?
      discard_byte();
      continue;
    }

    if (total_packet_size > RX_BUFFER_SIZE) {
      ROS_ERROR("SerialCommS300: Requested size of data is larger than the buffer size");
      discard_byte();
      continue;
    }

    // read the rest of the packet
    int status = read_byte(total_packet_size - m_rxCount);
    if (status == -2) {
      ROS_ERROR("SerialCommS300: Could not read the rest of the data packet!");
      return -2;
    }
    // give up for now if we don't have all bytes, return soft failure
    if (m_rxCount < total_packet_size) {
      ROS_DEBUG("SerialCommS300: Giving up for now, have only %lu bytes", m_rxCount);
      return -1;
    }

    // the checksum is contained in the last two bytes of the packet
    packet_checksum = *reinterpret_cast<unsigned short *> (&m_rxBuffer[total_packet_size - 2]);

    // excluded from checksumming: first four bytes in the header (zeroes), and the checksum bytes themselves
    calc_checksum = createCRC(&m_rxBuffer[4], total_packet_size - 2 - 4);

    ROS_DEBUG("SerialCommS300: Size: %d m_rxCount: %lu accessing checksum at m_rxBuffer[%u]", size, m_rxCount, total_packet_size - 2);

    if (packet_checksum != calc_checksum) {
      ROS_ERROR_STREAM("SerialCommS300: Checksums don't match (data packet size " << size << ")");
      discard_byte();
      continue;
    } else { // checksum valid
      ROS_DEBUG_STREAM("SerialCommS300: Checksums match (data packet size " << size << ")");

      if (m_rxBuffer[block_type_id_location] != m_rxBuffer[block_type_id_location + 1]) {
        ROS_ERROR("SerialCommS300: Bad type header bytes don't match");
      } else { // block type id bytes OK

        // skip the header to get to the beam measurement data
        uint8_t *beams = &m_rxBuffer[header_size];

        if (m_rxBuffer[block_type_id_location] == 0xAA) {
          ROS_ERROR("SerialCommS300: We got an I/O data packet, we don't know what to do with it");
        } else if (m_rxBuffer[block_type_id_location] == 0xBB) {
          // measurement data length: total packet size - header - checksums (2 bytes)
          // each measurement is 2 bytes
          unsigned int beam_count = (total_packet_size - header_size - 2) / 2;

          if (beam_count < 0) {
            ROS_ERROR_STREAM("SerialCommS300: bad beam count (" << beam_count << ")");
            discard_byte();
            continue;
          }

          // TODO remove this ugly memory allocation
          if (m_ranges) {
            delete[] m_ranges;
          }

          m_ranges = new float[beam_count];
          m_rangesCount = beam_count;

          for (int i = 0; i < beam_count; ++i) {
            unsigned short distance_cm = (*reinterpret_cast<unsigned short *> (&beams[2 * i]));
            distance_cm &= 0x1fff; // remove status bits
            m_ranges[i] = distance_cm / 100.0f;
          }

          // read the scan number (i.e. sensor timestamp - each scan takes 40 ms)
          m_scanNumber = *reinterpret_cast<unsigned int *> (&m_rxBuffer[scan_number_location]);
          m_telegramNumber = *reinterpret_cast<unsigned short *> (&m_rxBuffer[telegram_number_location]);

          discard_byte(total_packet_size);

          return 0;
        } else if (m_rxBuffer[block_type_id_location] == 0xCC) {
          ROS_ERROR("SerialCommS300: We got a reflector data packet, we don't know what to do with it");
        } else {
          ROS_ERROR("SerialCommS300: We got an unknown packet");
        }
      }
    }

    discard_byte();
    continue;
  }

  // we should not have reached here, but return soft failure just in case
  return -1;
}

int SerialCommS300::read_byte(unsigned int count) {
  ssize_t len = read(m_fd, &m_rxBuffer[m_rxCount], count);

  if (len < 0) {
    ROS_ERROR_STREAM("SerialCommS300: error reading from serial port: " << strerror(errno));
    return -2;
  } else if (len == 0) {
    ROS_WARN("SerialCommS300: received zero bytes!");
    if (zerobytesread_counter++ < 5) {
      return -1;
    } else {
      // if more than 5 times 0 bytes have been read, return error code -2
      // indicating a bad connection
      return -2;
    }
  }

  m_rxCount += len;

  return 0;
}

void SerialCommS300::discard_byte(unsigned int count) {
  memmove(m_rxBuffer, &m_rxBuffer[count], m_rxCount - count);
  m_rxCount -= count;
}
