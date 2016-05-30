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
#include <mutex>
#include <queue>

namespace Microsoft {
namespace Maker {
namespace Serial {

public ref class BleSerial sealed : public IStream
{
  public:
    virtual event IStreamConnectionCallback ^ConnectionEstablished;
    virtual event IStreamConnectionCallbackWithMessage ^ConnectionLost;
    virtual event IStreamConnectionCallbackWithMessage ^ConnectionFailed;

    ///<summary>
    ///A constructor which accepts a string corresponding to a device name or ID to connect to.
    ///</summary>
    BleSerial (
        Platform::String ^device_name_,
        Platform::Guid BLE_SERVICE_UUID,
        Platform::Guid BLE_SERIAL_RX_CHARACTERISTIC_UUID,
        Platform::Guid BLE_SERIAL_TX_CHARACTERISTIC_UUID
    );

    ///<summary>
    ///A constructor which accepts a DeviceInformation object to explicitly specify which device to connect to.
    ///</summary>
    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    BleSerial (
        Windows::Devices::Enumeration::DeviceInformation ^device_,
        Platform::Guid BLE_SERVICE_UUID,
        Platform::Guid BLE_SERIAL_RX_CHARACTERISTIC_UUID,
        Platform::Guid BLE_SERIAL_TX_CHARACTERISTIC_UUID
    );

    virtual
    ~BleSerial (
        void
    );

    virtual
    uint16_t
    available (
        void
    );

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    inline
    void
    begin (
        void
    ) {
        //baud rate and serial configuration are unnecessary for BLE connections as we are not using asyncronous TTL
        begin( NULL, SerialConfig::NONE );
    }

    virtual
    void
    begin (
        uint32_t baud_,
        SerialConfig config_
    );

    virtual
    bool
    connectionReady (
        void
    );

    virtual
    void
    end (
        void
    );

    virtual
    void
    flush (
        void
    );

    virtual
    void
    lock (
        void
    );

    virtual
    uint16_t
    print (
        uint8_t c_
    );

    virtual
    uint16_t
    print (
        int32_t value_
    );

    virtual
    uint16_t
    print (
        int32_t value_,
        Radix base_
    );

    virtual
    uint16_t
    print (
        uint32_t value_
    );

    virtual
    uint16_t
    print (
        uint32_t value_,
        Radix base_
    );

    virtual
    uint16_t
    print (
        double value_
    );

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    virtual
    uint16_t
    print (
        double value_,
        int16_t decimal_place_
    );

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    virtual
    uint16_t
    print (
        const Platform::Array<uint8_t> ^buffer_
    );

    virtual
    uint16_t
    read (
        void
    );

    virtual
    void
    unlock (
        void
    );

    virtual
    uint16_t
    write (
        uint8_t c_
    );

    [Windows::Foundation::Metadata::DefaultOverloadAttribute]
    virtual
    uint16_t
    write (
        const Platform::Array<uint8_t> ^buffer_
    );

    ///<summary>
    ///Begins an asyncronous request for all Bluetooth LE devices that are paired and may be used to attempt a device connection.
    ///</summary>
    static
    Windows::Foundation::IAsyncOperation<Windows::Devices::Enumeration::DeviceInformationCollection ^> ^
    listAvailableDevicesAsync (
        void
    );

  private:
    const Platform::Guid BLE_SERVICE_UUID;
    const Platform::Guid BLE_SERIAL_RX_CHARACTERISTIC_UUID;
    const Platform::Guid BLE_SERIAL_TX_CHARACTERISTIC_UUID;

    // Device specific members (set during instantation)
    Windows::Devices::Enumeration::DeviceInformation ^_device;
    Platform::String ^_device_name;

    //thread-safe mechanisms. std::unique_lock used to manage the lifecycle of std::mutex
    std::mutex _mutex;
    std::unique_lock<std::mutex> _ble_lock;

    std::atomic_bool _connection_ready;
    Windows::Devices::Enumeration::DeviceInformationCollection ^_device_collection;
    Windows::Devices::Bluetooth::GenericAttributeProfile::GattCharacteristic ^_gatt_rx_characteristic;
    Windows::Devices::Bluetooth::GenericAttributeProfile::GattCharacteristic ^_gatt_tx_characteristic;
    Windows::Devices::Bluetooth::BluetoothLEDevice ^_gatt_device;
    Windows::Devices::Bluetooth::GenericAttributeProfile::GattDeviceService ^_gatt_service;
    std::mutex _q_lock;
    std::queue<byte> _rx;
    Windows::Storage::Streams::DataWriter ^_tx;

    Concurrency::task<void>
    connectToDeviceAsync(
        Windows::Devices::Enumeration::DeviceInformation ^device_
    );

    Windows::Devices::Enumeration::DeviceInformation ^
    identifyDeviceFromCollection(
        Windows::Devices::Enumeration::DeviceInformationCollection ^devices_
    );

    void
    rxCallback(
        Windows::Devices::Bluetooth::GenericAttributeProfile::GattCharacteristic ^sender,
        Windows::Devices::Bluetooth::GenericAttributeProfile::GattValueChangedEventArgs ^args
    );
};

} // namespace Serial
} // namespace Maker
} // namespace Microsoft
