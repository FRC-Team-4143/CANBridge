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

#ifdef __linux__

#include "rev/Drivers/SocketCAN/SocketCANDevice.h"

#include <iostream> //TODO: Remove
#include <thread>

#include <hal/simulation/CanData.h>
#include <hal/CAN.h>

namespace rev {
namespace usb {

SocketCANDevice::SocketCANDevice(std::string port) :
    m_thread(port)
{
    m_descriptor = port;
    m_name = "SPARK MAX";     
    m_thread.Start();
}

SocketCANDevice::~SocketCANDevice()
{
    m_thread.Stop();
}

std::string SocketCANDevice::GetName() const
{
    return m_name;
}


std::string SocketCANDevice::GetDescriptor() const
{
    return m_descriptor;
}

int SocketCANDevice::GetId() const
{
    return 0;
}

int SocketCANDevice::GetNumberOfErrors() 
{
    return 0; //m_thread.GetNumberOfErrors();
}

CANStatus SocketCANDevice::SendCANMessage(const CANMessage& msg, int periodMs)
{
    //m_thread.EnqueueMessage(msg, periodMs);
    return CANStatus::kError; //m_thread.GetLastThreadError();
}

CANStatus SocketCANDevice::ReceiveCANMessage(std::shared_ptr<CANMessage>& msg, uint32_t messageID, uint32_t messageMask)
{
    CANStatus status = CANStatus::kTimeout;
   
    // parse through the keys, find the messges the match, and return it
    // The first in the message id, then the messages
    std::map<uint32_t, std::shared_ptr<CANMessage>> messages;
    //m_thread.ReceiveMessage(messages);
    std::shared_ptr<CANMessage> mostRecent;
    for (auto& m : messages) {
        if (CANBridge_ProcessMask({m.second->GetMessageId(), 0}, m.first) && CANBridge_ProcessMask({messageID, messageMask}, m.first)) {
            mostRecent = m.second;
            status = CANStatus::kOk;    
        }
    }

    if (status == CANStatus::kOk) {
        msg = mostRecent;
        status = CANStatus::kError; //m_thread.GetLastThreadError();
    } else {
        status = CANStatus::kError;
    }
    

    return status;
}

CANStatus SocketCANDevice::OpenStreamSession(uint32_t* sessionHandle, CANBridge_CANFilter filter, uint32_t maxSize)
{
    // Register the stream with the correct buffer size
    CANStatus stat = CANStatus::kOk;
    //m_thread.OpenStream(sessionHandle, filter, maxSize, &stat);
    return CANStatus::kError; //m_thread.GetLastThreadError();
}
CANStatus SocketCANDevice::CloseStreamSession(uint32_t sessionHandle)
{
    //m_thread.CloseStream(sessionHandle);
    return CANStatus::kError;// m_thread.GetLastThreadError();
}
CANStatus SocketCANDevice::ReadStreamSession(uint32_t sessionHandle, struct HAL_CANStreamMessage* msgs, uint32_t messagesToRead, uint32_t* messagesRead)
{
    //m_thread.ReadStream(sessionHandle, msgs, messagesToRead, messagesRead);
    return CANStatus::kError; //m_thread.GetLastThreadError();
}

CANStatus SocketCANDevice::GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr)
{
    rev::usb::CANStatusDetails details;
    //m_thread.GetCANStatus(&details);
    *busOff = details.busOffCount;
    *txFull = details.txFullCount;
    *receiveErr = details.receiveErrCount;
    *transmitErr = details.transmitErrCount;
    *percentBusUtilization = 0.0; // todo how to get this properly
    
    return CANStatus::kError; //m_thread.GetLastThreadError();
}

CANStatus SocketCANDevice::GetCANDetailStatus(float* percentBusUtilization, uint32_t* busOff, uint32_t* txFull, uint32_t* receiveErr, uint32_t* transmitErr, uint32_t* lastErrorTime)
{
    return GetCANDetailStatus(percentBusUtilization, busOff, txFull, receiveErr, transmitErr);
}

bool SocketCANDevice::IsConnected()
{
    return true;
}


} // namespace usb
} // namespace rev

#else
typedef int __ISOWarning__CLEAR_;
#endif // __linux__
