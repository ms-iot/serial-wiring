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
#include "NetworkSerial.h"

#include <bitset>

using namespace Concurrency;
using namespace Windows::Devices::Bluetooth::Rfcomm;
using namespace Windows::Devices::Enumeration;
using namespace Windows::Networking::Sockets;
using namespace Windows::Storage::Streams;

using namespace Microsoft::Maker::Serial;

//******************************************************************************
//* Constructors
//******************************************************************************

NetworkSerial::NetworkSerial(
    Windows::Networking::HostName ^host_,
    uint16_t port_
    ) :
    _connection_ready( ATOMIC_VAR_INIT( false ) ),
    _network_lock( _nutex, std::defer_lock ),
    _current_load_operation( nullptr ),
    _host( host_ ),
    _port( port_ ),
    _stream_socket( nullptr ),
    _rx( nullptr ),
    _tx( nullptr )
{
}

//******************************************************************************
//* Destructors
//******************************************************************************

NetworkSerial::~NetworkSerial(
    void
    )
{
    //we will fire the ConnectionLost event in the case that this object is unexpectedly destructed while the connection is established.
    if( connectionReady() )
    {
        ConnectionLost( L"Your connection has been terminated. The Microsoft::Maker::Serial::NetworkSerial destructor was called unexpectedly." );
    }
    end();
}

//******************************************************************************
//* Public Methods
//******************************************************************************

uint16_t
NetworkSerial::available(
    void
    )
{
	// Check to see if connection is ready
	if ( !connectionReady() ) {
		return 0;
	}

	if (_rx->UnconsumedBufferLength) {
		if (_rx->UnconsumedBufferLength > 0xFFFF) { return 0xFFFF; }
		return _rx->UnconsumedBufferLength;
	}
	else if (_current_load_operation->Status != Windows::Foundation::AsyncStatus::Started) {
		// Attempt to detect disconnection
		if (_current_load_operation->Status == Windows::Foundation::AsyncStatus::Error)
		{
			_connection_ready = false;
			ConnectionLost(L"A fatal error has occurred in UsbSerial::read() and your connection has been lost.");
			return 0;
		}

		_current_load_operation = _rx->LoadAsync(MAX_READ_SIZE);
	}

	return 0;
}

/// \details Immediately discards the incoming parameters, because they are used for standard serial connections and will have no bearing on a network connection.
void
NetworkSerial::begin(
    uint32_t baud_,
    SerialConfig config_
    )
{
    // Discard incoming parameters inherited from IStream interface.
    UNREFERENCED_PARAMETER( baud_ );
    UNREFERENCED_PARAMETER( config_ );

    // Ensure known good state
    end();

    connectToHostAsync( _host, _port )
        .then( [ this ]( Concurrency::task<void> t )
    {
        try
        {
            t.get();
        }
        catch( Platform::Exception ^e )
        {
            ConnectionFailed( ref new Platform::String( L"NetworkSerial::connectAsync failed with a Platform::Exception type. Did you forget your network capabilities in the manifest? Message: " ) + e->Message );
        }
        catch( ... )
        {
            ConnectionFailed( ref new Platform::String( L"NetworkSerial::connectAsync failed with a non-Platform::Exception type." ) );
        }
    } );
}

bool
NetworkSerial::connectionReady(
    void
    )
{
    return _connection_ready;
}

/// \ref https://social.msdn.microsoft.com/Forums/windowsapps/en-US/961c9d61-99ad-4a1b-82dc-22b6bd81aa2e/error-c2039-close-is-not-a-member-of-windowsstoragestreamsdatawriter?forum=winappswithnativecode
void
NetworkSerial::end(
    void
    )
{
    _connection_ready = false;
    _current_load_operation = nullptr;

    // Reset with respect to dependencies
    delete( _rx );
    _rx = nullptr;
    delete( _tx );
    _tx = nullptr;
    delete( _stream_socket );
    _stream_socket = nullptr;
}

void
NetworkSerial::flush(
    void
    )
{
    if ( !connectionReady() )
    {
        return;
    }

    auto async_operation = _tx->StoreAsync();
    create_task( async_operation )
    .then( [ this, async_operation ]( unsigned int value_ )
    {
        UNREFERENCED_PARAMETER( value_ );

        //detect disconnection
        if ( async_operation->Status == Windows::Foundation::AsyncStatus::Error )
        {
            throw ref new Platform::Exception( E_UNEXPECTED );
        }

        return create_task( _tx->FlushAsync() );
    } )
    .then( [ this ]( task<bool> task_ )
    {
        try
        {
            task_.wait();
        }
        catch( Platform::Exception ^e )
        {
            _connection_ready = false;
            ConnectionLost( L"A fatal error has occurred in NetworkSerial::flush() and your connection has been lost. Error: " + e->Message );
        }
    } );
}

void
NetworkSerial::lock(
    void
    )
{
    _network_lock.lock();
}

uint16_t
NetworkSerial::print(
    uint8_t c_
    )
{
    return write(c_);
}

uint16_t
NetworkSerial::print(
    int32_t value_
    )
{
    return print(value_, Radix::DEC);
}

uint16_t
NetworkSerial::print(
    int32_t value_,
    Radix base_
    )
{
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
NetworkSerial::print(
    uint32_t value_
    )
{
    return print(value_, Radix::DEC);
}

uint16_t
NetworkSerial::print(
    uint32_t value_,
    Radix base_
    )
{
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
NetworkSerial::print(
    double value_
    )
{
    return print(value_, 2);
}

uint16_t
NetworkSerial::print(
    double value_,
    int16_t decimal_places_
    )
{
    constexpr int max_double_size = (sizeof(double) * 8);
    constexpr int max_int_size = (sizeof(int16_t) * 8);
    char format_string[max_int_size + 5];
    char text_value[max_double_size + 1];

    sprintf_s(format_string, "%%.%ilf", decimal_places_);
    sprintf_s(text_value, format_string, value_);

    return write(Platform::ArrayReference<uint8_t>(reinterpret_cast<uint8_t *>(const_cast<char *>(text_value)), strnlen(text_value, max_double_size + 1)));
}

uint16_t
NetworkSerial::print(
    const Platform::Array<uint8_t> ^buffer_
    )
{
    return write(buffer_);
}

uint16_t
NetworkSerial::read(
    void
    )
{
	uint16_t c = static_cast<uint16_t>( -1 );

	if ( available() ) {
		c = _rx->ReadByte();
	}

	return c;
}

void
NetworkSerial::unlock(
    void
    )
{
    _network_lock.unlock();
}

uint16_t
NetworkSerial::write(
    uint8_t c_
    )
{
    // Check to see if connection is ready
    if( !connectionReady() ) { return 0; }

    _tx->WriteByte( c_ );
    return 1;
}

uint16_t
NetworkSerial::write(
    const Platform::Array<uint8_t> ^buffer_
    )
{
    // Check to see if connection is ready
    if (!connectionReady()) { return 0; }

    _tx->WriteBytes(buffer_);
    return buffer_->Length;
}

//******************************************************************************
//* Private Methods
//******************************************************************************

Concurrency::task<void>
NetworkSerial::connectToHostAsync(
    Windows::Networking::HostName ^host_,
    uint16_t port_
    )
{
    _stream_socket = ref new StreamSocket();
    _stream_socket->Control->KeepAlive = true;
    return Concurrency::create_task( _stream_socket->ConnectAsync( host_, port_.ToString() ) )
        .then( [ this ]()
    {
        _rx = ref new Windows::Storage::Streams::DataReader( _stream_socket->InputStream );
        _rx->InputStreamOptions = Windows::Storage::Streams::InputStreamOptions::Partial;  // Partial mode will allow for better async reads
        _current_load_operation = _rx->LoadAsync( MAX_READ_SIZE );

        // Enable TX
        _tx = ref new Windows::Storage::Streams::DataWriter( _stream_socket->OutputStream );

        // Set connection ready flag
        _connection_ready = true;
        ConnectionEstablished();
    } );
}