/**
 *
 *  \file
 *  \brief      Comms class definition. Does not manage the serial connection
 *              itself, but takes care of reading and writing to UM6.
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

#ifndef INCLUDE_COMMS_H_
#define INCLUDE_COMMS_H_

#include <stdint.h>
#include <string>
#include "um6/um6_fifo.h"

namespace serial {
  class Serial;
}

namespace um6 {

class SerialTimeout : public std::exception {};

class BadChecksum : public std::exception {};

class Registers;
class Accessor_;

class Comms {
  public:
    explicit Comms(serial::Serial& s) : serial_(s), first_spin_(true) {
        state = WAITING_HEADER;
        terminate = false;
        waiting_ack = false;
    }

    ~Comms() {
        terminate = true;
        receiver.join();
    }

    typedef boost::function<void (Registers&)> RegisterCallback;
    void startListeningThread(Registers* registers, int trigger, RegisterCallback callback);

    void send(const Accessor_& a) const;

    bool sendWaitAck(const Accessor_& a);

    static const uint8_t PACKET_HAS_DATA;
    static const uint8_t PACKET_IS_BATCH;
    static const uint8_t PACKET_BATCH_LENGTH_MASK;
    static const uint8_t PACKET_BATCH_LENGTH_OFFSET;

    static std::string checksum(const std::string& s);

    static std::string message(uint8_t address, std::string data);

  private:
    serial::Serial& serial_;
    bool first_spin_;

    void serialThread(Registers* registers, int trigger, RegisterCallback callback);

    CharFIFO fifo_;
    enum {WAITING_HEADER, WAITING_DATA_SIZE, WAITING_DATA} state;
    struct Message {
        uint8_t type;
        uint8_t address;
        bool has_data;
        bool is_batch;
        std::vector<uint8_t> data;
        void clear() {
            type = address = 0;
            has_data = is_batch = false;
            data.clear();
        }
    };
    bool waiting_ack;
    boost::thread receiver;
    boost::mutex msg_mutex;
    boost::condition_variable_any msg_cond;
    std::list<Message> messages;
    bool terminate;

};
}

#endif  // INCLUDE_COMMS_H_

