/*
 * Copyright (c) 2019 - 2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <vector>
#include <chrono>

// TODO: remove me
#include <clocale>
#include <iostream>
#include <iterator>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rev/CANMessage.h"
#include "rev/CANBridgeUtils.h"
#include "rev/CANStatus.h"
#include "rev/Drivers/DriverDeviceThread.h"

#include "utils/ThreadUtils.h"

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {


class SocketCANDeviceThread :public DriverDeviceThread { 
public:
    SocketCANDeviceThread() =delete;
    SocketCANDeviceThread(std::string port, long long threadIntervalMs = 1) : DriverDeviceThread(0xe45b5597, threadIntervalMs)
    { 
	std::cout << port << std::endl;
    }
    ~SocketCANDeviceThread()
    {
    }

    void Start() override {
        if (m_thread.get() != nullptr && m_thread->joinable()) {
            m_thread->join();
        }

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if(s < 0) {
		perror("socket");
		return;
	}

	strcpy(ifr.ifr_name, "can0");

	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
		return;
	}

        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return;
	}


        m_thread = std::make_unique<std::thread>(&SocketCANDeviceThread::CandleRun, this);

        // Set to high priority to prevent buffer overflow on the device on high client CPU load
        utils::SetThreadPriority(m_thread.get(), utils::ThreadPriority::High);
    }

    void OpenStream(uint32_t* handle, CANBridge_CANFilter filter, uint32_t maxSize, CANStatus *status) override {
        std::lock_guard<std::mutex> lock(m_streamMutex);

        // Create the handle
        *handle = m_counter++;

        // Add to the map
        m_readStream[*handle] = std::unique_ptr<CANStreamHandle>(new CANStreamHandle{filter.messageId, filter.messageMask, maxSize, utils::CircularBuffer<std::shared_ptr<CANMessage>>{maxSize}});

        *status = CANStatus::kOk;
    }



private:
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    int nbytes;

   void ReadMessages(bool &reading) {
       can_frame incomingFrame;

	nbytes = read(s, &incomingFrame, sizeof(struct can_frame));

	reading = (nbytes == sizeof(struct can_frame));

        // Received a new frame, store it
        if (reading) {

                auto msg = std::make_shared<CANMessage>(incomingFrame.can_id,
                                                    incomingFrame.data,
                                                    incomingFrame.can_dlc,
                                                    std::chrono::steady_clock::now().time_since_epoch().count() / 1000);

                // Read functions
                {
                    std::lock_guard<std::mutex> lock(m_readMutex);
                    m_readStore[incomingFrame.can_id] = msg;
                }

                // Streaming functions
                {
                    std::lock_guard<std::mutex> lock(m_streamMutex);
                    for (auto& stream : m_readStream) {
                        // Compare current size of the buffer to the max size of the buffer
                        if (!stream.second->messages.IsFull()
                            && rev::usb::CANBridge_ProcessMask({stream.second->messageId, stream.second->messageMask},
                            msg->GetMessageId())) {
                            stream.second->messages.Add(msg);
                        }
                    }
                }
            }

            m_threadStatus = CANStatus::kOk;
   }

   bool WriteMessages(detail::CANThreadSendQueueElement el, std::chrono::steady_clock::time_point now) {
        if (el.m_intervalMs == 0 || (now - el.m_prevTimestamp >= std::chrono::milliseconds(el.m_intervalMs)) ) {
            can_frame frame;
            frame.can_dlc = el.m_msg.GetSize();
            // set extended id flag
            frame.can_id = el.m_msg.GetMessageId() | 0x80000000;
            memcpy(frame.data, el.m_msg.GetData(), frame.can_dlc);

            // TODO: Feed back an error
            if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
                // std::cout << "Failed to send message: " << std::hex << (int)el.m_msg.GetMessageId() << std::dec << "  " << "reason why?" << std::endl;
                m_threadStatus = CANStatus::kDeviceWriteError;
                m_statusErrCount++;
                return false;
            } else {
                m_threadStatus = CANStatus::kOk;
                return true;
            }
        }
        return false;
   }

    void CandleRun() {
        while (m_threadComplete == false) {
            m_threadStatus = CANStatus::kOk; // Start each loop with the status being good. Really only a write issue.
            auto sleepTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(m_threadIntervalMs);

            // 1) Handle all received CAN traffic
            bool reading = false;
            ReadMessages(reading);

            // 2) Schedule CANMessage queue
            {
                std::lock_guard<std::mutex> lock(m_writeMutex);
                if (m_sendQueue.size() > 0) {
                    detail::CANThreadSendQueueElement el = m_sendQueue.front();
                    if (el.m_intervalMs == -1) {
                        m_sendQueue.pop();
                        continue;
                    }

                    auto now = std::chrono::steady_clock::now();

                    // Don't pop queue if send fails
                    if (WriteMessages(el, now)) {
                        m_sendQueue.pop();

                        // Return to end of queue if repeated
                        if (el.m_intervalMs > 0 ) {
                            el.m_prevTimestamp = now;
                            m_sendQueue.push(el);
                        }
                    }
                }
            }

            // 3) Stall thread
            if (!reading && m_sendQueue.empty()) {
                std::this_thread::sleep_until(sleepTime);
            }
        }

    }
};

} // namespace usb
} // namespace rev

