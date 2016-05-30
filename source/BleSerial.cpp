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

#include "pch.h"
#include "BleSerial.h"

#include <bitset>

using namespace Concurrency;
using namespace Windows::Devices::Bluetooth;
using namespace Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace Windows::Devices::Enumeration;
using namespace Windows::Storage::Streams;

using namespace Microsoft::Maker::Serial;

//******************************************************************************
//* Constructors
//******************************************************************************

BleSerial::BleSerial (
    Platform::String ^device_name_,
    Platform::Guid BLE_SERVICE_UUID,
    Platform::Guid BLE_SERIAL_RX_CHARACTERISTIC_UUID,
    Platform::Guid BLE_SERIAL_TX_CHARACTERISTIC_UUID
) :
    BLE_SERVICE_UUID( BLE_SERVICE_UUID),
    BLE_SERIAL_RX_CHARACTERISTIC_UUID(BLE_SERIAL_RX_CHARACTERISTIC_UUID),
    BLE_SERIAL_TX_CHARACTERISTIC_UUID(BLE_SERIAL_TX_CHARACTERISTIC_UUID),
    _connection_ready(ATOMIC_VAR_INIT(false)),
    _ble_lock(_mutex, std::defer_lock),
    _device(nullptr),
    _device_collection(nullptr),
    _device_name(device_name_),
    _gatt_rx_characteristic(nullptr),
    _gatt_tx_characteristic(nullptr),
    _gatt_device(nullptr),
    _gatt_service(nullptr),
    _tx(nullptr)
{ }

BleSerial::BleSerial (
    DeviceInformation ^device_,
    Platform::Guid BLE_SERVICE_UUID,
    Platform::Guid BLE_SERIAL_RX_CHARACTERISTIC_UUID,
    Platform::Guid BLE_SERIAL_TX_CHARACTERISTIC_UUID
) :
    BLE_SERVICE_UUID(BLE_SERVICE_UUID),
    BLE_SERIAL_RX_CHARACTERISTIC_UUID(BLE_SERIAL_RX_CHARACTERISTIC_UUID),
    BLE_SERIAL_TX_CHARACTERISTIC_UUID(BLE_SERIAL_TX_CHARACTERISTIC_UUID),
    _connection_ready(ATOMIC_VAR_INIT(false)),
    _ble_lock(_mutex, std::defer_lock),
    _device(device_),
    _device_name(nullptr),
    _device_collection(nullptr),
    _gatt_rx_characteristic(nullptr),
    _gatt_tx_characteristic(nullptr),
    _gatt_device(nullptr),
    _gatt_service(nullptr),
    _tx(nullptr)
{ }

//******************************************************************************
//* Destructors
//******************************************************************************

BleSerial::~BleSerial (
    void
) {
    //we will fire the ConnectionLost event in the case that this object is unexpectedly destructed while the connection is established.
    if( connectionReady() )
    {
        ConnectionLost( L"Your connection has been terminated. The Microsoft::Maker::Serial::BleSerial destructor was called unexpectedly." );
    }
    end();
}

//******************************************************************************
//* Public Methods
//******************************************************************************

uint16_t
BleSerial::available (
    void
) {
    // Check to see if connection is ready
    if (!connectionReady()) {
        return 0;
    } else {
        std::lock_guard<std::mutex> lock(_q_lock);
		if (_rx.size() > 0xFFFF) { return 0xFFFF; }
		return static_cast<uint16_t>(_rx.size());

    }
}

/// \details Immediately discards the incoming parameters, because they are used for standard serial connections and will have no bearing on a bluetooth connection.
/// \warning Must be called from the UI thread
void
BleSerial::begin (
    uint32_t baud_,
    SerialConfig config_
) {
    // Discard incoming parameters inherited from IStream interface.
    UNREFERENCED_PARAMETER(baud_);
    UNREFERENCED_PARAMETER(config_);

    // Ensure known good state
    end();

    // Although this path is not optimal, the behavior (the calls must be made from the UI thread) is mandated by the bluetooth API. The algorithm is a compromise to provide the succint, maintainable code.
    Concurrency::create_task(listAvailableDevicesAsync())
    .then([this](Windows::Devices::Enumeration::DeviceInformationCollection ^device_collection_) {
        // If a friendly name was specified, then identify the associated device
        if (_device_name) {
            // Store parameter as a member to ensure the duration of object allocation
            _device_collection = device_collection_;
            if (!_device_collection->Size)
            {
                throw ref new Platform::Exception(E_UNEXPECTED, L"No Bluetooth LE devices found or Bluetooth is disabled.");
            }

            _device = identifyDeviceFromCollection(_device_collection);
        }

        if (!_device) {
            throw ref new Platform::Exception(E_UNEXPECTED, L"ERROR! Hacking too much time!");
        }

        return connectToDeviceAsync(_device);
    }).then([this](Concurrency::task<void> t) {
        try
        {
            t.get();
        }
        catch (Platform::Exception ^e)
        {
            ConnectionFailed(L"BleSerial::connectToDeviceAsync failed with a Platform::Exception type. (message: " + e->Message + L")");
        }
        catch (...)
        {
            ConnectionFailed(L"BleSerial::connectToDeviceAsync failed with a non-Platform::Exception type. (name: " + _device_name + L")");
        }
    });
}

bool
BleSerial::connectionReady (
    void
) {
    return _connection_ready;
}

/// \ref https://social.msdn.microsoft.com/Forums/windowsapps/en-US/961c9d61-99ad-4a1b-82dc-22b6bd81aa2e/error-c2039-close-is-not-a-member-of-windowsstoragestreamsdatawriter?forum=winappswithnativecode
void
BleSerial::end (
    void
) {
    std::lock_guard<std::mutex> lock(_q_lock);
    _connection_ready =  false;
    if (_gatt_rx_characteristic) _gatt_rx_characteristic->WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue::None);
    if (_gatt_tx_characteristic) _gatt_tx_characteristic->WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue::None);

    // Reset with respect to dependencies
    _rx = std::queue<byte>();
    delete(_tx); //_tx->Close();
    _tx = nullptr;
    _gatt_rx_characteristic = nullptr;
    _gatt_tx_characteristic = nullptr;
    _gatt_service = nullptr;
    _gatt_device = nullptr;
    _device_collection = nullptr;
}

void
BleSerial::flush (
    void
) {
    if ( !connectionReady() ) { return; }

    create_task(_gatt_tx_characteristic->WriteValueAsync(_tx->DetachBuffer(), GattWriteOption::WriteWithoutResponse))
    .then([this](GattCommunicationStatus status_) {
        switch (status_) {
        case GattCommunicationStatus::Success:
            break;
        case GattCommunicationStatus::Unreachable:
            ConnectionLost( L"Your connection has been lost. The device is no longer available." );
            break;
        default:
            break;
        }
    });
}

/// \details An Advanced Query String is constructed based upon paired bluetooth GATT devices. Then a collection is returned of all devices matching the query.
/// \ref https://msdn.microsoft.com/en-us/library/aa965711(VS.85).aspx
/// \warning Must be called from UI thread
Windows::Foundation::IAsyncOperation<Windows::Devices::Enumeration::DeviceInformationCollection ^> ^
BleSerial::listAvailableDevicesAsync (
    void
) {
    // Construct AQS String for Bluetooth LE devices
    Platform::String ^device_aqs = Windows::Devices::Bluetooth::BluetoothLEDevice::GetDeviceSelector();

    // Identify all paired devices satisfying query
    return Windows::Devices::Enumeration::DeviceInformation::FindAllAsync(device_aqs);
}

void
BleSerial::lock (
    void
) {
    _ble_lock.lock();
}

uint16_t
BleSerial::print (
    uint8_t c_
) {
    return write(c_);
}

uint16_t
BleSerial::print (
    int32_t value_
) {
    return print(value_, Radix::DEC);
}

uint16_t
BleSerial::print (
    int32_t value_,
    Radix base_
) {
    constexpr int bit_size = (sizeof(int) * 8);
    std::bitset<bit_size> bits(value_);
    char text_value[bit_size + 1];

    switch (base_) {
      case Radix::BIN:
        sprintf_s(text_value, "%s", bits.to_string().c_str());
        break;
      case Radix::DEC:
        sprintf_s(text_value, "%i", value_);
        break;
      case Radix::HEX:
        sprintf_s(text_value, "%x", value_);
        break;
      case Radix::OCT:
        sprintf_s(text_value, "%o", value_);
        break;
      default:
        return static_cast<uint16_t>(-1);
    }

    return write(Platform::ArrayReference<uint8_t>(reinterpret_cast<uint8_t *>(const_cast<char *>(text_value)), strnlen(text_value, bit_size + 1)));
}

uint16_t
BleSerial::print (
    uint32_t value_
) {
    return print(value_, Radix::DEC);
}

uint16_t
BleSerial::print (
    uint32_t value_,
    Radix base_
) {
    constexpr int bit_size = (sizeof(unsigned int) * 8);
    std::bitset<bit_size> bits(value_);
    char text_value[bit_size + 1];

    switch (base_) {
      case Radix::BIN:
        sprintf_s(text_value, "%s", bits.to_string().c_str());
        break;
      case Radix::DEC:
        sprintf_s(text_value, "%u", value_);
        break;
      case Radix::HEX:
        sprintf_s(text_value, "%x", value_);
        break;
      case Radix::OCT:
        sprintf_s(text_value, "%o", value_);
        break;
      default:
        return static_cast<uint16_t>(-1);
    }

    return write(Platform::ArrayReference<uint8_t>(reinterpret_cast<uint8_t *>(const_cast<char *>(text_value)), strnlen(text_value, bit_size + 1)));
}

uint16_t
BleSerial::print (
    double value_
) {
    return print(value_, 2);
}

uint16_t
BleSerial::print (
    double value_,
    int16_t decimal_places_
) {
    constexpr int max_double_size = (sizeof(double) * 8);
    constexpr int max_int_size = (sizeof(int16_t) * 8);
    char format_string[max_int_size + 5];
    char text_value[max_double_size + 1];

    sprintf_s(format_string, "%%.%ilf", decimal_places_);
    sprintf_s(text_value, format_string, value_);

    return write(Platform::ArrayReference<uint8_t>(reinterpret_cast<uint8_t *>(const_cast<char *>(text_value)), strnlen(text_value, max_double_size + 1)));
}

uint16_t
BleSerial::print (
    const Platform::Array<uint8_t> ^buffer_
) {
    return write(buffer_);
}

uint16_t
BleSerial::read (
    void
) {
    uint16_t c = static_cast<uint16_t>(-1);

    if (available()) {
        std::lock_guard<std::mutex> lock(_q_lock);
        c = _rx.front();
        _rx.pop();
    }

    return c;
}

void
BleSerial::unlock (
    void
) {
    _ble_lock.unlock();
}

uint16_t
BleSerial::write (
    uint8_t c_
) {
    // Check to see if connection is ready
    if (!connectionReady()) { return 0; }

    _tx->WriteByte(c_);
    return 1;
}

uint16_t
BleSerial::write (
    const Platform::Array<uint8_t> ^buffer_
) {
    // Check to see if connection is ready
    if (!connectionReady()) { return 0; }

    _tx->WriteBytes(buffer_);
    return buffer_->Length;
}

//******************************************************************************
//* Private Methods
//******************************************************************************

Concurrency::task<void>
BleSerial::connectToDeviceAsync (
    Windows::Devices::Enumeration::DeviceInformation ^device_
) {
    _device_name = device_->Name;  // Update name in case device was specified directly
    return Concurrency::create_task(Windows::Devices::Bluetooth::BluetoothLEDevice::FromIdAsync(device_->Id))
    .then([this](Windows::Devices::Bluetooth::BluetoothLEDevice ^gatt_device_) {
        if( gatt_device_ == nullptr )
        {
            throw ref new Platform::Exception( E_UNEXPECTED, ref new Platform::String( L"Unable to initialize the device. BluetoothLEDevice::FromIdAsync returned null." ) );
        }

        // Store parameter as a member to ensure the duration of object allocation
        _gatt_device = gatt_device_;

        // Enable TX
        _tx = ref new DataWriter();
        _gatt_service = _gatt_device->GetGattService(BLE_SERVICE_UUID);
        _gatt_tx_characteristic = _gatt_service->GetCharacteristics(BLE_SERIAL_TX_CHARACTERISTIC_UUID)->GetAt(0);

        // Enable RX
        _gatt_rx_characteristic = _gatt_service->GetCharacteristics(BLE_SERIAL_RX_CHARACTERISTIC_UUID)->GetAt(0);
        _gatt_rx_characteristic->ValueChanged += ref new Windows::Foundation::TypedEventHandler<GattCharacteristic ^, GattValueChangedEventArgs ^>(this, &BleSerial::rxCallback);
        _gatt_rx_characteristic->WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue::Notify);

        // Set connection ready flag and fire connection established event
        _connection_ready = true;
        ConnectionEstablished();
    });
}

Windows::Devices::Enumeration::DeviceInformation ^
BleSerial::identifyDeviceFromCollection (
    Windows::Devices::Enumeration::DeviceInformationCollection ^devices_
) {
    for(auto &&device : devices_) {
        if (device->Id->Equals(_device_name) || device->Name->Equals(_device_name)) { return device; }
    }

    // If we searched and found nothing that matches the identifier, we've failed to connect and cannot recover.
    throw ref new Platform::Exception(E_INVALIDARG, L"No Bluetooth LE devices found matching the specified identifier.");
}

void
BleSerial::rxCallback (
    GattCharacteristic ^sender,
    GattValueChangedEventArgs ^args
) {
    // Extract data into workable form from parameters
    Platform::Array<byte> ^rx_data = ref new Platform::Array<byte>(args->CharacteristicValue->Length);
    DataReader::FromBuffer(args->CharacteristicValue)->ReadBytes(rx_data);

    {
        std::lock_guard<std::mutex> lock(_q_lock);
        std::for_each(rx_data->Data, rx_data->Data + rx_data->Length, [this](byte data_) {
            _rx.push(data_);
        });
    }
}
