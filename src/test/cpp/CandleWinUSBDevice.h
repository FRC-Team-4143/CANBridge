/*
 * Copyright (c) 2019 REV Robotics
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

#include <map>
#include <string>

#include "candle.h"

#include "CANDevice.h"
#include "CANMessage.h"
#include "CANStatus.h"

namespace rev {
namespace usb {

class CandleWinUSBDevice : public CANDevice {
public:
    CandleWinUSBDevice(candle_handle hDev);
    ~CandleWinUSBDevice();

    virtual std::string GetName() const;
    virtual std::wstring GetDescriptor() const;

    virtual int GetId() const;

    virtual CANStatus SendMessage(CANMessage msg, int periodMs);
    virtual CANStatus RecieveMessage(CANMessage& msg, uint32_t messageMask, uint32_t& timestamp);
    virtual CANStatus OpenStreamSession();
    virtual CANStatus CloseStreamSession();
    virtual CANStatus ReadStreamSession();

    virtual CANStatus GetCANStatus();

    virtual bool IsConnected();
private:
    CandleWinUSBDevice() {}
    candle_handle m_handle;
};

} // namespace usb
} // namespace rev
