/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// This defines an stream interface for communication via USB, Ethernet, etc.---这定义了通过USB、以太网等进行通信的流接口。

#ifndef DRIVERS_GNSS_STREAM_H
#define DRIVERS_GNSS_STREAM_H

#include <stdint.h>
#include <ros/ros.h>

#include "util/macros.h"

namespace apollo {
namespace drivers {
namespace gnss {

// An abstract class of Stream.
// One should use the create_xxx() functions to create a Stream object.
class Stream {
public:
    // Return a pointer to a Stream object. The caller should take ownership.
    static Stream* create_tcp(const char* address, uint16_t port, uint32_t timeout_usec = 1000000);

    static Stream* create_udp(const char* address, uint16_t port, uint32_t timeout_usec = 1000000);

    // Currently the following baud rates are supported:---目前支持以下波特率：
    //  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600.
    static Stream* create_serial(const char* device_name, uint32_t baud_rate,
                                 uint32_t timeout_usec = 0);

    static Stream* create_ntrip(const std::string& address, uint16_t port,
                                const std::string& mountpoint, const std::string& user,
                                const std::string& passwd, uint32_t timeout_s = 30);

    virtual ~Stream() {}

    // Stream status.
    enum class Status {
        DISCONNECTED,
        CONNECTED,
        ERROR,
    };

    static constexpr size_t NUM_STATUS = static_cast<int>(Stream::Status::ERROR) + 1;
    Status get_status() const { return _status; }

    // Returns whether it was successful to connect.
    virtual bool connect() = 0;

    // Returns whether it was successful to disconnect.
    virtual bool disconnect() = 0;

    void register_login_data(const std::vector<std::string> login_data) {
        _login_data.assign(login_data.begin(), login_data.end());
    }

    void login() {
      for (size_t i = 0; i < _login_data.size(); ++i) {
            write(_login_data[i]);
            ROS_INFO_STREAM("Login: " << _login_data[i]);
            // sleep a little to avoid overun of the slow serial interface.---稍微睡一会儿，以避免慢速串行接口的过度运行。
            ros::Duration(0.2).sleep();
        }
    }

    // Reads up to max_length bytes. Returns actually number of bytes read.
    virtual size_t read(uint8_t* buffer, size_t max_length) = 0;

    // Returns how many bytes it was successful to write.
    virtual size_t write(const uint8_t* buffer, size_t length) = 0;

    size_t write(const std::string& buffer) {
        return write(reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size());
    }

protected:
    Stream() {}

    Status _status = Status::DISCONNECTED;

private:
    std::vector<std::string > _login_data;
    DISABLE_COPY_AND_ASSIGN(Stream);
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo

#endif  // DRIVERS_GNSS_STREAM_H
