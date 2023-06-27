/*
 * Copyright (c) 2023 Petr Muzikant, Cybernetica AS, petr.muzikant@cyber.ee
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
#include <numeric>
#include <QDebug>

namespace serial_cpp
{

// Initializes SerialPortHandler by sending one command to obtain STATUS and parsing it to class
// members
SerialPortHandler::SerialPortHandler(const QSerialPortInfo& _qPortInfo)
{
    qPortInfo = _qPortInfo;

    // GET STATUS
    QSerialPort serial;
    serial.setPort(qPortInfo);
    // qInfo() << "-----------------------------------------------------";
    // qInfo() << "qPortInfo.portName():" << qPortInfo.portName();
    // qInfo() << "qPortInfo.systemLocation():" << qPortInfo.systemLocation();
    // qInfo() << "qPortInfo.description():" << qPortInfo.description();
    // qInfo() << "qPortInfo.manufacturer():" << qPortInfo.manufacturer();
    // qInfo() << "qPortInfo.productIdentifier():" << qPortInfo.productIdentifier();
    // qInfo() << "qPortInfo.serialNumber():" << qPortInfo.serialNumber();
    // qInfo() << "qPortInfo.vendorIdentifier():" << qPortInfo.vendorIdentifier();
    // qInfo() << "qPortInfo.isNull():" << qPortInfo.isNull();
    // qInfo() << "qPortInfo.hasProductIdentifier():" << qPortInfo.hasProductIdentifier();
    // qInfo() << "qPortInfo.hasVendorIdentifier():" << qPortInfo.hasVendorIdentifier();

    setSerialPortOptions(serial, defaultSerialOptions);

    if (!serial.open(QIODevice::ReadWrite)) {
        SERIAL_CPP_THROW(SDevicePortNotOpenedError,
                         std::string("Failed to open serial port, error: ")
                             + serial.errorString().toStdString());
    }

    // Send GET_STATUS command
    // Timeout 50 is here because most of the ports are not supported devices,
    // so we need to filter through them quickly
    byte_vector result = sendAndReceive(serial, {0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF}, 50);

    serial.close();

    // Check SW
    if (result[0] != 0x90 || result[1] != 0x00) {
        SERIAL_CPP_THROW(SDeviceDataFormatError,
                         "Wrong SW in response to GET STATUS command"
                             + bytes2hexstr(std::vector<byte_vector::value_type>(
                                 result.begin(), result.begin() + 2)));
    }

    // Extract data
    byte_vector data(result.begin() + 2, result.end());

    // FORMAT OF RETURNED DATA (256 bytes):
    //      SERIALID[10]
    //      HAS_PINPAD{0x00 or 0x01}
    //      VERSION[3]
    //      SEPARATOR{0xFF, 0x01}
    //      SSA_SIZE{SIZE}
    //      SUPPORTED_SIGNATURE_ALGORITHMS[SSA_SIZE]
    //      SEPARATOR{0xFF, 0x02}
    //      ISA_SIZE{SIZE}
    //      {INITIALIZED_SIGNATURE_ALGORITHM, MODE(AUTH | SIGN)}[ISA_SIZE]
    //      the rest is 0xFF until the end of the buffer

    // SERIALID[10]
    serialID = byte_vector(data.begin(), data.begin() + 10);

    // HAS_PINPAD{0x00 or 0x01}
    hasPinPad = data[10] == 0x01;

    // VERSION[3]
    firmwareVersion = byte_vector(data.begin() + 11, data.begin() + 14);

    // SEPARATOR{0xFF, 0x01}
    if (data[14] != 0xFF || data[15] != 0x01) {
        SERIAL_CPP_THROW(SDeviceDataFormatError, "Wrong format in response to GET STATUS command");
    }

    // SIZE{SIZE}
    size_t ssaSize = data[16];

    // SUPPORTED_SIGNATURE_ALGORITHMS[SSA_SIZE]
    supportedSignatureAlgorithms = byte_vector(data.begin() + 17, data.begin() + 17 + ssaSize);

    // SEPARATOR{0xFF, 0x02}
    if (data[17 + ssaSize] != 0xFF || data[18 + ssaSize] != 0x02) {
        SERIAL_CPP_THROW(SDeviceDataFormatError, "Wrong format in response to GET STATUS command");
    }

    // ISA_SIZE{SIZE}
    size_t isaSize = data[19 + ssaSize];

    // {INITIALIZED_SIGNATURE_ALGORITHM, MODE(AUTH | SIGN)}[ISA_SIZE]
    initializedSignatureAlgorithms =
        byte_vector(data.begin() + 20 + ssaSize, data.begin() + 20 + ssaSize + isaSize);

    // Check if the rest is 0xFF
    for (size_t i = 20 + ssaSize + isaSize; i < data.size(); i++) {
        if (data[i] != 0xFF) {
            SERIAL_CPP_THROW(SDeviceDataFormatError,
                             "Wrong format in response to GET STATUS command");
        }
    }
}

} // namespace serial_cpp
