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

#pragma once

#include <memory>
#include <vector>
#include <limits>
#include <string>

#include "qserialport.h"
#include "serial-cpp/serial-cpp-utils.hpp"

#include <QtSerialPort/qserialportinfo.h>

namespace serial_cpp
{
using byte_vector = std::vector<unsigned char>;

#ifdef _WIN32
using string_t = std::wstring;
#else
using string_t = std::string;
#endif

/** Returns the value of the response status bytes SW1 and SW2 as a single status word SW. */
inline constexpr uint16_t toSW(byte_vector::value_type sw1, byte_vector::value_type sw2)
{
    return uint16_t(sw1 << 8) | sw2;
}

struct SerialPortOptions
{
    int baudRate;
    QSerialPort::DataBits dataBits;
    QSerialPort::Parity parity;
    QSerialPort::StopBits stopBits;
    QSerialPort::FlowControl flowControl;
};

inline void setSerialPortOptions(QSerialPort& serial, SerialPortOptions options)
{
    serial.setBaudRate(options.baudRate);
    serial.setDataBits(options.dataBits);
    serial.setParity(options.parity);
    serial.setStopBits(options.stopBits);
    serial.setFlowControl(options.flowControl);
}

// Default serial device options for obtaining status.
static SerialPortOptions defaultSerialOptions {
    .baudRate = 115200,
    .dataBits = QSerialPort::DataBits::Data8,
    .parity = QSerialPort::Parity::NoParity,
    .stopBits = QSerialPort::StopBits::OneStop,
    .flowControl = QSerialPort::FlowControl::NoFlowControl,
};

/** Struct that wraps response APDUs. */
struct SerialDeviceResponseApdu
{
    enum Status {
        OK = 0x90,
        MORE_DATA_AVAILABLE = 0x61,
        VERIFICATION_FAILED = 0x63,
        VERIFICATION_CANCELLED = 0x64,
        WRONG_LENGTH = 0x67,
        COMMAND_NOT_ALLOWED = 0x69,
        WRONG_PARAMETERS = 0x6a,
        WRONG_LE_LENGTH = 0x6c,
        PIN_BLOCKED = 0x97
    };

    byte_vector::value_type sw1 {};
    byte_vector::value_type sw2 {};

    byte_vector data;

    static const size_t MAX_DATA_SIZE = std::numeric_limits<unsigned short>::max();
    static const size_t MAX_SIZE = MAX_DATA_SIZE + 2; // + sw1 and sw2

    SerialDeviceResponseApdu(byte_vector::value_type s1, byte_vector::value_type s2,
                             byte_vector d = {}) :
        sw1(s1),
        sw2(s2), data(std::move(d))
    {
    }

    SerialDeviceResponseApdu() = default;

    static SerialDeviceResponseApdu fromBytes(const byte_vector& data)
    {
        if (data.size() < 2) {
            throw std::invalid_argument(
                "Need at least 2 bytes for creating SerialDeviceResponseApdu");
        }

        if (data.size() == 2) {
            return SerialDeviceResponseApdu {data[0], data[1]};
        }

        // SW1 and SW2 are at the start
        return SerialDeviceResponseApdu {data[0], data[1],
                                         byte_vector {data.cbegin() + 2, data.cend()}};
    }

    byte_vector toBytes() const
    {
        // makes a copy, valid both if data is empty or full
        byte_vector bytes = data;

        // SW1 and SW2 are at the start
        bytes.insert(bytes.begin(), sw2);
        bytes.insert(bytes.begin(), sw1);

        return bytes;
    }

    uint16_t toSW() const { return serial_cpp::toSW(sw1, sw2); }

    bool isOK() const { return sw1 == OK && sw2 == 0x00; }

    // TODO: friend function toString() in utilities.hpp
};

/** Struct that wraps command APDUs. */
struct SerialDeviceCommandApdu
{
    unsigned char ins;
    unsigned char mode;
    unsigned char algo;
    unsigned short le;
    // Lc is data.size() or 0x0000
    byte_vector data;

    static const size_t MAX_DATA_SIZE = std::numeric_limits<unsigned short>::max();

    SerialDeviceCommandApdu(unsigned char ins, unsigned char mode, unsigned char algo,
                            byte_vector d = {}, unsigned short le = 0) :
        ins(ins),
        mode(mode), algo(algo), le(le), data(std::move(d))
    {
    }

    SerialDeviceCommandApdu(const SerialDeviceCommandApdu& other, byte_vector data) :
        ins(other.ins), mode(other.mode), algo(other.algo), le(other.le), data(std::move(data))
    {
    }

    SerialDeviceCommandApdu(const SerialDeviceCommandApdu& other, size_t new_le) :
        ins(other.ins), mode(other.mode), algo(other.algo), data(std::move(other.data))
    {
        if (new_le > MAX_DATA_SIZE) {
            throw std::invalid_argument("Le must be less than 65536");
        }
        le = static_cast<unsigned short>(new_le);
    }

    bool isLeSet() const { return le != (unsigned short)0; }

    static SerialDeviceCommandApdu fromBytes(const byte_vector& bytes)
    {
        if (bytes.size() < 7) {
            throw std::invalid_argument("Command APDU must have 7 or more bytes");
        }

        if (bytes.size() == 7) {
            if (bytes[3] != 0x00 || bytes[4] != 0x00) {
                throw std::invalid_argument("Le must be 0x0000 if not data is present");
            }
            return SerialDeviceCommandApdu {
                bytes[0], bytes[1], bytes[2], byte_vector(),
                static_cast<unsigned short>(uint16_t(bytes[5]) << 8 | bytes[6])};
        }

        // bytes contain data
        auto dataStart = bytes.cbegin() + 5;

        return SerialDeviceCommandApdu {
            bytes[0], bytes[1], bytes[2], byte_vector(dataStart, bytes.cend() - 2),
            static_cast<unsigned short>(uint16_t(*(bytes.cend() - 2)) << 8 | *(bytes.cend() - 1))};
    }

    byte_vector toBytes() const
    {
        if (data.size() > MAX_DATA_SIZE) {
            throw std::invalid_argument("Data size larger than maximum");
        }

        auto bytes = byte_vector {ins, mode, algo};

        if (!data.empty()) {
            bytes.push_back(static_cast<unsigned char>(data.size() >> 8));
            bytes.push_back(static_cast<unsigned char>(data.size() & 0xff));
            bytes.insert(bytes.end(), data.cbegin(), data.cend());
        } else {
            bytes.push_back(0x00);
            bytes.push_back(0x00);
        }

        bytes.push_back(static_cast<unsigned char>(le >> 8));
        bytes.push_back(static_cast<unsigned char>(le & 0xff));

        return bytes;
    }
};

class SerialDeviceImpl;
using SerialDeviceImplPtr = std::unique_ptr<SerialDeviceImpl>;

/** PIN pad PIN entry timer timeout */
constexpr uint8_t PIN_PAD_PIN_ENTRY_TIMEOUT = 90; // 1 minute, 30 seconds

class SerialDevice
{
public:
    using ptr = std::unique_ptr<SerialDevice>;

    SerialDevice(QSerialPortInfo _qPortInfo, bool _hasPinPad, byte_vector _serialID,
                 byte_vector _firmwareVersion, byte_vector _supportedSignatureAlgorithms,
                 byte_vector _initializedSignatureAlgorithms);
    SerialDevice(); // Null object constructor.
    ~SerialDevice();

    // The rule of five.
    SerialDevice(const SerialDevice&) = delete;
    SerialDevice& operator=(const SerialDevice&) = delete;
    SerialDevice(SerialDevice&&) = delete;
    SerialDevice& operator=(SerialDevice&&) = delete;

    SerialDeviceResponseApdu transmit(const SerialDeviceCommandApdu& command) const;
    void setup(const SerialPortOptions options) const;

    QSerialPortInfo qPortInfo;
    bool hasPinPad;
    byte_vector serialID;
    byte_vector firmwareVersion;
    byte_vector supportedSignatureAlgorithms;
    byte_vector initializedSignatureAlgorithms;

private:
    SerialDeviceImplPtr serialDevice;
};

class SerialPortHandler
{
public:
    explicit SerialPortHandler(const QSerialPortInfo& _qPortInfo);

    SerialDevice::ptr connectToSerialDevice() const
    {
        return std::make_unique<SerialDevice>(std::move(qPortInfo), hasPinPad, serialID,
                                              firmwareVersion, supportedSignatureAlgorithms,
                                              initializedSignatureAlgorithms);
    }

    QSerialPortInfo qPortInfo;
    bool hasPinPad;
    byte_vector serialID;
    byte_vector firmwareVersion;
    byte_vector supportedSignatureAlgorithms;
    byte_vector initializedSignatureAlgorithms;
};

std::vector<const std::string> listPorts();

// Utility functions.

extern const byte_vector APDU_RESPONSE_OK;

/** Convert bytes to hex string. */
std::string bytes2hexstr(const byte_vector& bytes);

byte_vector sendAndReceive(QSerialPort& serial, const byte_vector& commandBytes,
                           int timeoutMs = 3000);

/** Transmit APDU command and verify that expected response is received. */
void transmitApduWithExpectedResponse(const SerialDevice& serialDevice,
                                      const SerialDeviceCommandApdu& command,
                                      const byte_vector& expectedResponseBytes = APDU_RESPONSE_OK);
void transmitApduWithExpectedResponse(const SerialDevice& serialDevice,
                                      const byte_vector& commandBytes,
                                      const byte_vector& expectedResponseBytes = APDU_RESPONSE_OK);

size_t readCertificateLengthFromAsn1(const SerialDevice& serialDevice,
                                     const SerialDeviceCommandApdu& command);

// Errors.
/** Base class for all serial-cpp errors. */
class SerialCppError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/** Programming or system errors. */
class SDeviceSystemError : public SerialCppError
{
public:
    using SerialCppError::SerialCppError;
};

/** Base class for all SerialDevice API errors. */
class SDeviceError : public SerialCppError
{
public:
    using SerialCppError::SerialCppError;
};

/** Thrown when the USB serial could not be or was not opened. */
class SDevicePortNotOpenedError : public SDeviceError
{
public:
    using SDeviceError::SDeviceError;
};

/** Thrown when no serial devices are connected to the system. */
class SDeviceNoDevicesError : public SDeviceError
{
public:
    using SDeviceError::SDeviceError;
};

/** Thrown when there was a timeout when reading data from the device. */
class SDeviceTimeoutError : public SDeviceError
{
public:
    using SDeviceError::SDeviceError;
};

/** Thrown when the device returned data in unexpected format */
class SDeviceDataFormatError : public SDeviceError
{
public:
    using SDeviceError::SDeviceError;
};

/** Thrown when there was some error during communication with the device. */
class SDeviceCommunicationError : public SDeviceError
{
public:
    using SDeviceError::SDeviceError;
};

} // namespace serial_cpp
