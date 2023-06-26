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
#include "serial-cpp/serial-cpp-utils.hpp"

#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>

#include <sstream>
#include <iomanip>

#include <QDebug>

using namespace serial_cpp;
using namespace std::string_literals;

#ifdef HIBYTE
#undef HIBYTE
#endif
#ifdef LOBYTE
#undef LOBYTE
#endif

constexpr unsigned char HIBYTE(size_t w)
{
    return static_cast<unsigned char>((w >> 8) & 0xff);
}
constexpr unsigned char LOBYTE(size_t w)
{
    return static_cast<unsigned char>(w & 0xff);
}

namespace
{

const unsigned char DER_SEQUENCE_TYPE_TAG = 0x30;
const unsigned char DER_TWO_BYTE_LENGTH = 0x82;

class UnexpectedResponseError : public SDeviceError
{
public:
    explicit UnexpectedResponseError(const SerialDeviceCommandApdu& command,
                                     const byte_vector& expectedResponseBytes,
                                     const SerialDeviceResponseApdu& response, const char* file,
                                     const int line, const char* callerFunctionName) :
        SDeviceError("transmitApduWithExpectedResponse(): Unexpected response to command '"s
                     + bytes2hexstr(command.toBytes()) + "' - expected '"s
                     + bytes2hexstr(expectedResponseBytes) + "', got '"s
                     + bytes2hexstr(response.toBytes()) + " in " + removeAbsolutePathPrefix(file)
                     + ':' + std::to_string(line) + ':' + callerFunctionName)
    {
    }
};

} // namespace

namespace serial_cpp
{

const byte_vector APDU_RESPONSE_OK {SerialDeviceResponseApdu::OK, 0x00};

std::string bytes2hexstr(const byte_vector& bytes)
{
    std::ostringstream hexStringBuilder;

    hexStringBuilder << std::setfill('0') << std::hex;

    for (const auto byte : bytes)
        hexStringBuilder << std::setw(2) << short(byte);

    return hexStringBuilder.str();
}

// Command structure: INS MODE ALGO Lc1 Lc2 [DATA]* Le1 Le2
// If no data si present, Lc1 and Lc2 must be 0
byte_vector sendAndReceive(QSerialPort& serial, const byte_vector& commandBytes, int timeoutMs)
{
    // Clear the buffer
    serial.clear();

    // Send command
    size_t bytesWritten =
        serial.write(reinterpret_cast<const char*>(commandBytes.data()), commandBytes.size());

    if (serial.waitForBytesWritten(timeoutMs) && bytesWritten != commandBytes.size()) {
        serial.close();
        SERIAL_CPP_THROW(SDeviceCommunicationError,
                         "sendAndReceive(): Failed to write command bytes");
    }

    // Get SW
    std::vector<uint8_t> result(2);
    qint64 bytesAvailable = serial.bytesAvailable();
    bool waitForReadyRead = serial.waitForReadyRead(timeoutMs);
    if (!bytesAvailable && !waitForReadyRead) {
        serial.close();
        SERIAL_CPP_THROW(SDeviceTimeoutError, "sendAndReceive(): Timeout when getting SW");
    };
    qint64 bytesRead = serial.read(reinterpret_cast<char*>(result.data()), 2);

    if (bytesRead == -1) {
        serial.close();
        SERIAL_CPP_THROW(SDeviceSystemError,
                         "Failed to read data from " + serial.portName().toStdString());
    }

    if (bytesRead != 2) {
        serial.close();
        SERIAL_CPP_THROW(SDeviceCommunicationError, "sendAndReceive(): Failed to read SW");
    }

    // last two bytes of command are le
    uint16_t le =
        commandBytes[commandBytes.size() - 2] << 8 | commandBytes[commandBytes.size() - 1];

    // If data is expected and SW was OK, read data
    if (le != 0 && result[0] == 0x90 && result[1] == 0x00) {
        std::vector<uint8_t> data;
        size_t readTotal = 0;

        while (true) {
            // If bytes are available, no need to call waitForReadyRead
            // If bytes available is 0, waitForReadyRead fills the buffer in almost an instant.
            if (!serial.bytesAvailable() && !serial.waitForReadyRead(timeoutMs)) {
                SERIAL_CPP_THROW(SDeviceSystemError, "No bytes were available");
            }

            QByteArray readBytes = serial.readAll();
            data.insert(data.end(), readBytes.begin(), readBytes.end());
            readTotal += readBytes.size();

            if (readTotal >= le) {
                break;
            }
        }

        if (readTotal > le) {
            serial.close();
            SERIAL_CPP_THROW(SDeviceCommunicationError,
                             "sendAndReceive(): Length of all data is longer than expected");
        }

        result.insert(result.end(), data.begin(), data.end());
    }

    return result;
}

void transmitApduWithExpectedResponse(const SerialDevice& serialDevice,
                                      const byte_vector& commandBytes,
                                      const byte_vector& expectedResponseBytes)
{
    const auto command = SerialDeviceCommandApdu::fromBytes(commandBytes);
    transmitApduWithExpectedResponse(serialDevice, command, expectedResponseBytes);
}

void transmitApduWithExpectedResponse(const SerialDevice& serialDevice,
                                      const SerialDeviceCommandApdu& command,
                                      const byte_vector& expectedResponseBytes)
{
    const auto response = serialDevice.transmit(command);
    if (response.toBytes() != expectedResponseBytes) {
        throw UnexpectedResponseError(command, expectedResponseBytes, response, __FILE__, __LINE__,
                                      __func__);
    }
}

size_t readCertificateLengthFromAsn1(const SerialDevice& serialDevice,
                                     const SerialDeviceCommandApdu& command)
{
    auto response = serialDevice.transmit(command);

    // Verify expected DER header, first byte must be SEQUENCE.
    if (response.data[0] != DER_SEQUENCE_TYPE_TAG) {
        SERIAL_CPP_THROW(SDeviceError,
                         "readDataLengthFromAsn1(): First byte must be SEQUENCE (0x30), but is 0x"s
                             + bytes2hexstr({response.data[0]}));
    }

    // TODO: support other lenghts besides 2.
    // Assume 2-byte length, so second byte must be 0x82.
    if (response.data[1] != DER_TWO_BYTE_LENGTH) {
        SERIAL_CPP_THROW(SDeviceError,
                         "readDataLengthFromAsn1(): Second byte must be two-byte length indicator "s
                         "(0x82), but is 0x"s
                             + bytes2hexstr({response.data[1]}));
    }

    // Read 2-byte length field at offset 2 and 3 and add the 4 DER length bytes.
    const auto length = size_t((response.data[2] << 8) + response.data[3] + 4);

    return length;
}

} // namespace serial_cpp
