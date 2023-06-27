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

#include <dirent.h>

// #include <cstring>
// #include <memory>
// #include <algorithm>
// #include <map>

namespace
{

using namespace serial_cpp;

std::vector<const std::string> getPortNames()
{
#ifdef __APPLE__
    std::vector<const std::string> ports;
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir("/dev")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string name = ent->d_name;
            if (name.find("cu.wch") == 0) {
                std::string port = "/dev/" + name;
                ports.push_back(port);
            }
        }
        closedir(dir);
    } else {
        throw SDeviceError("Cannot open /dev directory");
    }
    return ports;
#endif
    throw SDeviceError("Not implemented");
}

} // anonymous namespace

namespace serial_cpp
{

std::vector<const std::string> listPorts()
{
    return getPortNames();
}

} // namespace serial_cpp
