/*
Copyright(c) Microsoft Open Technologies, Inc. All rights reserved.

The MIT License(MIT)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#pragma once
#include "IStream.h"
#include "BleSerial.h"

namespace Microsoft {
namespace Maker {
namespace Serial {

public ref class DfRobotBleSerial sealed : public IStream
{
  public:
    virtual event IStreamConnectionCallback ^ConnectionEstablished;
    virtual event IStreamConnectionCallbackWithMessage ^ConnectionLost;
    virtual event IStreamConnectionCallbackWithMessage ^ConnectionFailed;

    ///<summary>
    ///A constructor which accepts a string corresponding to a device name or ID to connect to.
    ///</summary>
    DfRobotBleSerial (
        Platform::String ^device_name_
    ) :
        _bleSerial(ref new BleSerial(
            device_name_,
            Platform::Guid(0x0000dfb0, 0x0000, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb),
            Platform::Guid(0x0000dfb1, 0x0000, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb),
            Platform::Guid(0x0000dfb1, 0x0000, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb)
        ))
    { }

    ///<summary>
    ///A constructor which accepts a DeviceInformation object to explicitly specify which device to connect to.
    ///</summary>
    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    DfRobotBleSerial (
        Windows::Devices::Enumeration::DeviceInformation ^device_
    ) :
        _bleSerial(ref new BleSerial(
            device_,
            Platform::Guid(0x0000dfb0, 0x0000, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb),
            Platform::Guid(0x0000dfb1, 0x0000, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb),
            Platform::Guid(0x0000dfb1, 0x0000, 0x1000, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb)
        ))
    { }

    virtual inline
    ~DfRobotBleSerial (
        void
    ) {
        _bleSerial->~BleSerial();
    }

    virtual inline
    uint16_t
    available (
        void
    ) {
        return _bleSerial->available();
    }

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    inline
    void
    begin (
        void
    ) {
        //baud rate and serial configuration are unnecessary for BLE connections as we are not using asyncronous TTL
        begin(NULL, SerialConfig::NONE);
    }

    virtual inline
    void
    begin (
        uint32_t baud_,
        SerialConfig config_
    ) {
        _bleSerial->ConnectionEstablished += ref new IStreamConnectionCallback(this, &DfRobotBleSerial::OnConnectionEstablished);
        _bleSerial->ConnectionLost += ref new IStreamConnectionCallbackWithMessage(this, &DfRobotBleSerial::OnConnectionLost);
        _bleSerial->ConnectionFailed += ref new IStreamConnectionCallbackWithMessage(this, &DfRobotBleSerial::OnConnectionFailed);
        _bleSerial->begin(baud_, config_);
    }

    virtual inline
    bool
    connectionReady (
        void
    ) {
        return _bleSerial->connectionReady();
    }

    virtual inline
    void
    end (
        void
    ) {
        _bleSerial->end();
    }

    virtual inline
    void
    flush (
        void
    ) {
        _bleSerial->flush();
    }

    virtual inline
    void
    lock (
        void
    ) {
        _bleSerial->lock();
    }

    virtual inline
    uint16_t
    print (
        uint8_t c_
    ) {
        return _bleSerial->print(c_);
    }

    virtual inline
    uint16_t
    print (
        int32_t value_
    ) {
        return _bleSerial->print(value_);
    }

    virtual inline
    uint16_t
    print (
        int32_t value_,
        Radix base_
    ) {
        return _bleSerial->print(value_, base_);
    }

    virtual inline
    uint16_t
    print (
        uint32_t value_
    ) {
        return _bleSerial->print(value_);
    }

    virtual inline
    uint16_t
    print (
        uint32_t value_,
        Radix base_
    ) {
        return _bleSerial->print(value_, base_);
    }

    virtual inline
    uint16_t
    print (
        double value_
    ) {
        return _bleSerial->print(value_);
    }

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    virtual inline
    uint16_t
    print (
        double value_,
        int16_t decimal_place_
    ) {
        return _bleSerial->print(value_, decimal_place_);
    }

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    virtual inline
    uint16_t
    print (
        const Platform::Array<uint8_t> ^buffer_
    ) {
        return _bleSerial->print(buffer_);
    }

    virtual inline
    uint16_t
    read (
        void
    ) {
        return _bleSerial->read();
    }

    virtual inline
    void
    unlock (
        void
    ) {
        _bleSerial->unlock();
    }

    virtual inline
    uint16_t
    write (
        uint8_t c_
    ) {
        return _bleSerial->write(c_);
    }

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    virtual inline
    uint16_t
    write (
        const Platform::Array<uint8_t> ^buffer_
    ) {
        return _bleSerial->write(buffer_);
    }

    ///<summary>
    ///Begins an asyncronous request for all Bluetooth LE devices that are paired and may be used to attempt a device connection.
    ///</summary>
    static inline
    Windows::Foundation::IAsyncOperation<Windows::Devices::Enumeration::DeviceInformationCollection ^> ^
    listAvailableDevicesAsync (
        void
    ) {
        return BleSerial::listAvailableDevicesAsync();
    }

  private:
    BleSerial ^ const _bleSerial;

    inline
    void
    OnConnectionEstablished (
        void
    ) {
        ConnectionEstablished();
    }

    inline
    void
    OnConnectionLost (
        Platform::String ^message
    ) {
        ConnectionLost(message);
    }

    inline
    void
    OnConnectionFailed (
        Platform::String ^message
    ) {
        ConnectionFailed(message);
    }
};

} // namespace Serial
} // namespace Maker
} // namespace Microsoft
