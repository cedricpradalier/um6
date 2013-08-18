/**
 *
 *  \file
 *  \brief      Implementation of Comms class methods to handle reading and 
 *              writing to the UM6 serial interface.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to code@clearpathrobotics.com 
 *
 */

#include "um6/comms.h"

#include <arpa/inet.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <string>

#include "um6/registers.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "serial/serial.h"

#define DEBUG_COMMS

namespace um6 {


void Comms::serialThread(Registers* registers, int trigger, RegisterCallback  callback) {
    uint16_t checksum_calculated = 0;
    uint8_t data_length = 0;
    Message msg;
    ROS_INFO("Starting listening thread");
    uint8_t buffer[512];
    // First empty the buffer
    while (serial_.lowlevel_read(buffer,128)) {};

    while (!terminate && ros::ok()) {
        if (serial_.wait_data(500)) {
            // Wait a bit to make sur we have plenty to read
            ros::Duration(10e-3).sleep();
#ifdef DEBUG_COMMS
            ROS_DEBUG("Data!");
#endif
            // Now get it all and push it back in the buffer
            size_t n = 0;
            do {
                n = serial_.lowlevel_read(buffer,512);
                fifo_.push(buffer,n);
            } while (n);
#ifdef DEBUG_COMMS
            ROS_DEBUG("Pushing data: size %d",(int)fifo_.available());
#endif
            // Finally go through the data and extract all packets
            bool enough_data = true;
            while (enough_data) {
                switch (state) {
                    case WAITING_HEADER:
                        {
                            uint8_t hdr[3];
                            fifo_.discard_until('s');
                            if (fifo_.peek(hdr,3)==3) {
                                if ((hdr[0]=='s') && (hdr[1]=='n') && (hdr[2]=='p')) {
                                    state = WAITING_DATA_SIZE;
                                    msg.clear();
                                    // Discard header
                                    fifo_.pop(NULL,3);
                                    continue;
                                } else {
                                    // This was not an interesting 's', discard it
                                    fifo_.pop();
                                    continue;
                                }
                            }
                            enough_data = false;
                            break;
                        }
                    case WAITING_DATA_SIZE:
                        {
                            uint8_t data[2];
                            if (fifo_.peek(data,2) == 2) {
                                msg.type = data[0];
                                msg.address = data[1];
                                msg.has_data = msg.type & PACKET_HAS_DATA;
                                msg.is_batch = msg.type & PACKET_IS_BATCH;
                                if (msg.has_data) {
                                    data_length = 1;
                                    if (msg.is_batch) {
                                        data_length = (msg.type >> PACKET_BATCH_LENGTH_OFFSET) & PACKET_BATCH_LENGTH_MASK;
#ifdef DEBUG_COMMS
                                        ROS_DEBUG("Received packet %02x with batched (%d) data.", msg.address, data_length);
#endif
                                    } else {
#ifdef DEBUG_COMMS
                                        ROS_DEBUG("Received packet %02x with non-batched data.", msg.address);
#endif
                                    }
                                    data_length *= 4;
                                    state = WAITING_DATA;
                                    continue;
                                } else {
#ifdef DEBUG_COMMS
                                    ROS_DEBUG("Received packet %02x without data.", msg.address);
#endif
                                    data_length = 0;
                                    state = WAITING_DATA;
                                    continue;
                                }
                            }
                            enough_data = false;
                            break;
                        }
                    case WAITING_DATA:
                        {
                            if (fifo_.available()>=(data_length+4)) {
                                uint8_t data[data_length + 4];
                                std::string dbg = "snp ";
                                fifo_.pop(data,data_length+4);
                                checksum_calculated = 's' + 'n' + 'p';
                                for (size_t i=0;i<(size_t)(data_length+2);i++) {
                                    char tdbg[4];
                                    sprintf(tdbg,"%02X ",data[i]);
                                    dbg+=tdbg;
                                    checksum_calculated += data[i];

                                }
                                uint16_t checksum_transmitted = *reinterpret_cast<uint16_t*>(data+data_length+2);
                                checksum_transmitted = ntohs(checksum_transmitted);
                                if (checksum_transmitted != checksum_calculated) {
#ifdef DEBUG_COMMS
                                    ROS_DEBUG("Message data: %s",dbg.c_str());
#endif
                                    ROS_WARN("Discarding packet due to bad checksum %04X / %04X.",checksum_transmitted,checksum_calculated);
                                } else {
#ifdef DEBUG_COMMS
                                    ROS_DEBUG("Accepted data: %s",dbg.c_str());
#endif
                                    boost::mutex::scoped_lock lock(msg_mutex);
                                    // Copy data from checksum buffer into registers, if specified.
                                    // Note that byte-order correction (as necessary) happens at access-time.
                                    if ((data_length > 0) and registers) {
                                        registers->write_raw(msg.address, data+2,data_length);
                                        if (msg.address == trigger) {
                                            callback(*registers);
                                        }
                                    }
                                    if (waiting_ack) {
                                        msg.data.resize(data_length);
                                        for (size_t i=2;i<(size_t)(data_length+2);i++) {
                                            msg.data[i-2] = data[i];
                                        }
                                        messages.push_back(msg);
                                        msg_cond.notify_all();
                                    }
                                }
                                state = WAITING_HEADER;
                                continue;
                            }
                            enough_data = false;
                            break;
                        }
                    default:
                        enough_data = false;
                        break;
                }
            }
        } else {
            // Something weird, we might as well clear the buffer
            fifo_.clear();
            ROS_WARN("No data received in the last second. Is the sensor still connected");
        }
    }
}


const uint8_t Comms::PACKET_HAS_DATA = 1 << 7;
const uint8_t Comms::PACKET_IS_BATCH = 1 << 6;
const uint8_t Comms::PACKET_BATCH_LENGTH_MASK = 0x0F;
const uint8_t Comms::PACKET_BATCH_LENGTH_OFFSET = 2;

void Comms::startListeningThread(Registers* registers, int trigger, RegisterCallback callback) 
{
    receiver = boost::thread(&Comms::serialThread, this, registers,trigger,callback);
}


std::string Comms::checksum(const std::string& s) {
  uint16_t checksum = 0;
  BOOST_FOREACH(uint8_t ch, s)
    checksum += ch;
  checksum = htons(checksum);
#ifdef DEBUG_COMMS
  ROS_DEBUG("Computed checksum on string of length %ld as %04x.", (long int)s.length(), checksum);
#endif
  std::string out(2, 0);
  memcpy(&out[0], &checksum, 2);
  return out;
}

std::string Comms::message(uint8_t address, std::string data) {
  uint8_t type = 0;
  if (data.length() > 0)
    type |= PACKET_HAS_DATA;
  if (data.length() > 4) {
    type |= PACKET_IS_BATCH;
    type |= (data.length() / 4) << PACKET_BATCH_LENGTH_OFFSET;
  }

  std::stringstream ss(std::stringstream::out | std::stringstream::binary);
  ss << "snp" << type << address << data;
  std::string output = ss.str();
  std::string c = checksum(output);
  ss << c;
  output = ss.str();
#ifdef DEBUG_COMMS
  ROS_DEBUG("Generated message %02x of overall length %ld.", address, (long int)output.length());
#endif
  return output;
}

void Comms::send(const Accessor_& r) const {
  // uint8_t address = r.index;
  std::string data((char*)r.raw(), r.length * 4);
  serial_.write(message(r.index, data));
}

bool Comms::sendWaitAck(const Accessor_& r) {
  const uint8_t tries = 5;
  boost::mutex::scoped_lock lock(msg_mutex);
  waiting_ack = true;
  for (uint8_t t = 0; t < tries; t++) {
      send(r);
      const uint8_t listens = 20;
      for (uint8_t i = 0; i < listens; i++) {
          if (messages.empty()) {
              boost::system_time const timeout = boost::get_system_time() 
                  + boost::posix_time::milliseconds(100);
              if (!msg_cond.timed_wait(lock,timeout)) {
                  ROS_WARN("Serial read timed out waiting for ack. Attempting to retransmit.");
                  break;
              }
          } 
          Message m = messages.front();
          messages.pop_front();
          int16_t received = m.address;
          if (received == r.index) {
              waiting_ack = false;
              messages.clear();
#ifdef DEBUG_COMMS
              ROS_DEBUG("Message %02x ack received.", received);
#endif
              return true;
          }
      }
  }
  waiting_ack = false;
  messages.clear();
  return false;
}
}  // um6
