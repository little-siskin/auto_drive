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

#ifndef DRIVERS_GNSS_RAW_STREAM_H
#define DRIVERS_GNSS_RAW_STREAM_H

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/stream.h"
#include "proto/config.pb.h"
#include "proto/gnss_status.pb.h"

#include <signal.h>

namespace {
    // void sig_handler(int sig) {
    //     apollo::drivers::gnss::login_setting();
    // }

    //apollo::drivers::gnss::bool login_setting();
}


namespace apollo {
namespace drivers{
namespace gnss {

class RawStream {
public:
    RawStream(ros::NodeHandle& nh, const std::string &name,
              const std::string &raw_topic, const std::string &rtcm_topic,
              const std::string &stream_status_topic);
    ~RawStream();
    bool init(const std::string &cfg_file);

    struct Status {
        bool filter[Stream::NUM_STATUS] = {false};
        Stream::Status status;
    };
    //static bool login_setting();
private:
    void data_spin();
    void ntrip_spin();
    bool connect();
    bool disconnect();
    bool login();
    //new add
    //static void sig_handler(int sig);
    void init_signal();
    //static bool login_setting();
    //*******
    bool logout();
    void stream_status_check();
    std::string status_string(Stream::Status status);


    static constexpr size_t BUFFER_SIZE = 2048;
    uint8_t _buffer[BUFFER_SIZE];
    uint8_t _buffer_ntrip[BUFFER_SIZE];

    std::shared_ptr<Stream> _data_stream;
    std::shared_ptr<Stream> _command_stream;
    std::shared_ptr<Stream> _in_rtk_stream;
    std::shared_ptr<Stream> _out_rtk_stream;


    std::shared_ptr<Status> _data_stream_status;
    std::shared_ptr<Status> _command_stream_status;
    std::shared_ptr<Status> _in_rtk_stream_status;
    std::shared_ptr<Status> _out_rtk_stream_status;

    bool _rtk_software_solution = false;
    bool _is_healthy = true;
    config::Config _config;

    const std::string _raw_data_topic;
    const std::string _rtcm_data_topic;
    const ros::Publisher _raw_data_publisher;
    const ros::Publisher _rtcm_data_publisher;
    const ros::Publisher _stream_status_publisher;

    boost::shared_ptr<apollo::common::gnss_status::StreamStatus> _stream_status;
    std::unique_ptr<std::thread> _data_thread_ptr;
    std::unique_ptr<std::thread> _ntrip_thread_ptr;
};

}
}
}
#endif
