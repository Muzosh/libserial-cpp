/*
 * Copyright (c) 2023 Petr Muzikant
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "serial-cpp/serial-cpp.hpp"
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>

#include <array>
#include <map>
#include <utility>
#include <string>
#include "serial-cpp/serial-cpp.hpp"

namespace serial_cpp
{

class SerialDeviceImpl
{
public:
    explicit SerialDeviceImpl(const QSerialPortInfo& portInfo)
    {
        serial = std::make_unique<QSerialPort>();
        serial->setPort(portInfo);
        if (!serial->open(QIODevice::ReadWrite)) {
            SERIAL_CPP_THROW(SDevicePortNotOpenedError,
                             "Serial" + portInfo.systemLocation().toStdString()
                                 + "could not be opened.");
        }
    }

    ~SerialDeviceImpl()
    {
        if (serial->isOpen()) {
            serial->close();
        }
    }

    // The rule of five (C++ Core guidelines C.21).
    SerialDeviceImpl(const SerialDeviceImpl& other) = delete;
    SerialDeviceImpl(SerialDeviceImpl&& other) noexcept = delete;
    SerialDeviceImpl& operator=(const SerialDeviceImpl& other) = delete;
    SerialDeviceImpl& operator=(SerialDeviceImpl&& other) noexcept = delete;

    // Command structure: INS MODE ALGO Lc1 Lc2 [DATA]* Le1 Le2
    // If no data si present, Lc1 and Lc2 must be 0
    SerialDeviceResponseApdu transmitBytes(const byte_vector& commandBytes) const
    {
        byte_vector result = sendAndReceive(*serial, commandBytes);
        return toResponse(result);
    }

    void setup(const SerialPortOptions options) { setSerialPortOptions(*serial, options); }

private:
    std::unique_ptr<QSerialPort> serial;

    SerialDeviceResponseApdu toResponse(byte_vector responseBytes) const
    {
        auto response = SerialDeviceResponseApdu::fromBytes(responseBytes);

        // Let expected errors through for handling in upper layers or in if blocks below.
        switch (response.sw1) {
        case SerialDeviceResponseApdu::OK:
        case SerialDeviceResponseApdu::MORE_DATA_AVAILABLE: // See the if block after next.
        case SerialDeviceResponseApdu::VERIFICATION_FAILED:
        case SerialDeviceResponseApdu::VERIFICATION_CANCELLED:
        case SerialDeviceResponseApdu::WRONG_LENGTH:
        case SerialDeviceResponseApdu::COMMAND_NOT_ALLOWED:
        case SerialDeviceResponseApdu::WRONG_PARAMETERS:
        case SerialDeviceResponseApdu::WRONG_LE_LENGTH: // See next if block.
            break;
        default:
            SERIAL_CPP_THROW(SDeviceError,
                             "SDeviceError response: '"
                                 + bytes2hexstr({response.sw1, response.sw2}));
        }

        if (response.sw1 == SerialDeviceResponseApdu::WRONG_LE_LENGTH) {
            SERIAL_CPP_THROW(SDeviceError, "Wrong LE length (SW1=0x6C) in response, please set LE");
        }

        return response;
    }
};

SerialDevice::SerialDevice(QSerialPortInfo _qPortInfo, bool _hasPinPad, byte_vector _serialID,
                           byte_vector _firmwareVersion, byte_vector _supportedSignatureAlgorithms,
                           byte_vector _initializedSignatureAlgorithms) :
    qPortInfo(std::move(_qPortInfo)),
    hasPinPad(_hasPinPad), serialID(std::move(_serialID)),
    firmwareVersion(std::move(_firmwareVersion)),
    supportedSignatureAlgorithms(std::move(_supportedSignatureAlgorithms)),
    initializedSignatureAlgorithms(std::move(_initializedSignatureAlgorithms)),
    serialDevice(std::make_unique<SerialDeviceImpl>(qPortInfo))
{
}

SerialDevice::SerialDevice() = default;
SerialDevice::~SerialDevice() = default;

void SerialDevice::setup(const SerialPortOptions options) const
{
    SERIAL_CPP_REQUIRE_NON_NULL(serialDevice)

    serialDevice->setup(options);
}

SerialDeviceResponseApdu SerialDevice::transmit(const SerialDeviceCommandApdu& command) const
{
    SERIAL_CPP_REQUIRE_NON_NULL(serialDevice)

    return serialDevice->transmitBytes(command.toBytes());
}

} // namespace serial_cpp
