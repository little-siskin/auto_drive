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

#include <memory>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/stream.h"
#include "gnss/utils.h"
#include "raw_stream.h"
#include "proto/config.pb.h"
#include <iostream>
#include <sys/prctl.h>
//#include <sys/io.h>
#include <sys/unistd.h>


namespace {
void switch_stream_status(const apollo::drivers::gnss::Stream::Status &status,
                          apollo::common::gnss_status::StreamStatus_Type &report_status_type) {
    switch (status) {
    case apollo::drivers::gnss::Stream::Status::CONNECTED:
        report_status_type = apollo::common::gnss_status::StreamStatus::CONNECTED;
        break;

    case apollo::drivers::gnss::Stream::Status::DISCONNECTED:
        report_status_type = apollo::common::gnss_status::StreamStatus::DISCONNECTED;
        break;

    case apollo::drivers::gnss::Stream::Status::ERROR:
    default:
        report_status_type = apollo::common::gnss_status::StreamStatus::DISCONNECTED;
        break;
    }
}


}

namespace apollo{
namespace drivers{
namespace gnss{


}
}
}

namespace apollo {
namespace drivers{
namespace gnss {

Stream* create_stream(const config::Stream& sd);

bool login_setting(){

    ROS_INFO("---loging_setting----");
    char *pathname;
    pathname = getenv("ROS_ROOT");
    std::string back_string = std::string("/../../../..");
    std::string path_string = std::string(pathname) + back_string + "/modules/drivers/gnss/conf/gnss_conf_setting.txt";
    std::string cfg_file = path_string;
    std::shared_ptr<Stream> _data_stream;
    std::shared_ptr<Stream> _command_stream;
    config::Config _config;
    Stream* s = nullptr;
    if (!parse_config_text(cfg_file, &_config)) {
        ROS_INFO("Parse config context failed.");
        return false;
    }

    s = create_stream(_config.data());
    if (s == nullptr) {
        ROS_ERROR("Failed to create data stream.");
        return false;
    }
    _data_stream.reset(s);

    if (_config.has_command()) {
        s = create_stream(_config.command());
        if (s == nullptr) {
            ROS_ERROR("Failed to create command stream.");
            return false;
        }
        _command_stream.reset(s);
    } else {
        _command_stream = _data_stream;
    }

    std::vector<std::string> login_setting_data;
    for (int i = 0; i < _config.login_setting_commands_size(); ++i) {
        _command_stream->write(_config.login_setting_commands(i));
        login_setting_data.emplace_back(_config.login_setting_commands(i));
        ROS_INFO_STREAM("Login command: " << _config.login_setting_commands(i));
        // sleep a little to avoid overun of the slow serial interface.
        ros::Duration(0.2).sleep();
    }
    _command_stream->register_login_data(login_setting_data);

    return true;
}

void sig_handler(int sig) {
    ROS_INFO("-----sig_handler----");
    apollo::drivers::gnss::login_setting();
}


Stream* create_stream(const config::Stream& sd) {
    switch (sd.type_case()) {
    case config::Stream::kSerial:
        if (!sd.serial().has_device()) {
            ROS_ERROR("Serial def has no device field.");
            return nullptr;
        }
        if (!sd.serial().has_baud_rate()) {
            ROS_ERROR_STREAM("Serial def has no baud_rate field. Use default baud rate "
                         << sd.serial().baud_rate());
            return nullptr;
        }
        return Stream::create_serial(sd.serial().device().c_str(), sd.serial().baud_rate());

    case config::Stream::kTcp:
        if (!sd.tcp().has_address()) {
            ROS_ERROR("tcp def has no address field.");
            return nullptr;
        }
        if (!sd.tcp().has_port()) {
            ROS_ERROR("tcp def has no port field.");
            return nullptr;
        }
        return Stream::create_tcp(sd.tcp().address().c_str(), sd.tcp().port());

    case config::Stream::kUdp:
        if (!sd.udp().has_address()) {
            ROS_ERROR("tcp def has no address field.");
            return nullptr;
        }
        if (!sd.udp().has_port()) {
            ROS_ERROR("tcp def has no port field.");
            return nullptr;
        }
        return Stream::create_udp(sd.udp().address().c_str(), sd.udp().port());

    case config::Stream::kNtrip:
        if (!sd.ntrip().has_address()) {
            ROS_ERROR("ntrip def has no address field.");
            return nullptr;
        }
        if (!sd.ntrip().has_port()) {
            ROS_ERROR("ntrip def has no port field.");
            return nullptr;
        }
        if (!sd.ntrip().has_mount_point()) {
            ROS_ERROR("ntrip def has no mount point field.");
            return nullptr;
        }
        if (!sd.ntrip().has_user()) {
            ROS_ERROR("ntrip def has no user field.");
            return nullptr;
        }
        if (!sd.ntrip().has_password()) {
            ROS_ERROR("ntrip def has no passwd field.");
            return nullptr;
        }
        return Stream::create_ntrip(sd.ntrip().address(), sd.ntrip().port(),
                                    sd.ntrip().mount_point(), sd.ntrip().user(),
                                    sd.ntrip().password(), sd.ntrip().timeout_s());
    default:
        return nullptr;
    }
}

RawStream::RawStream(ros::NodeHandle &nh,
                     const std::string &name,
                     const std::string &raw_data_topic,
                     const std::string &rtcm_data_topic,
                     const std::string &stream_status_topic) :
    _raw_data_topic(raw_data_topic),
    _rtcm_data_topic(rtcm_data_topic),
    _raw_data_publisher(nh.advertise<std_msgs::String>(_raw_data_topic, 256)),
    _rtcm_data_publisher(nh.advertise<std_msgs::String>(_rtcm_data_topic, 256)),
    _stream_status_publisher(nh.advertise<apollo::common::gnss_status::StreamStatus>(stream_status_topic, 256, true)) {
    _stream_status.reset(new apollo::common::gnss_status::StreamStatus());
}

RawStream::~RawStream() {
    this->logout();
    this->disconnect();
}

bool RawStream::init(const std::string& cfg_file) {

    init_signal();

    if (!_stream_status) {
        ROS_ERROR_STREAM("New stream status failed.");
        return false;
    }
    _stream_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    _stream_status->set_ins_stream_type(apollo::common::gnss_status::StreamStatus::DISCONNECTED);
    _stream_status->set_rtk_stream_in_type(apollo::common::gnss_status::StreamStatus::DISCONNECTED);
    _stream_status->set_rtk_stream_out_type(apollo::common::gnss_status::StreamStatus::DISCONNECTED);
    _stream_status_publisher.publish(_stream_status);
    std::cout << "cfg_file = " << cfg_file << std::endl;    //cfg_file ??????????????????
    //std::cout << "_config = " << &_config << std::endl;   _config ??????????????????
    if (!parse_config_text(cfg_file, &_config)) {
        ROS_INFO("Parse config context failed.");
        return false;
    }
    ROS_INFO_STREAM("Loaded config:\n" << _config.DebugString());
    std::cout << "config_file endding----" << std::endl;    //

    // Creates streams.
    Stream* s = nullptr;
    if (!_config.has_data()) {
        ROS_INFO("Error: Config file must provide the data stream.");
        return false;
    }
    s = create_stream(_config.data());
    if (s == nullptr) {
        ROS_ERROR("Failed to create data stream.");
        return false;
    }
    _data_stream.reset(s);
    //std::cout << "--------------data_serial_stream_ok----------"<<std::endl;
    Status *status = new Status();
    if (!status) {
        ROS_ERROR("Failed to create data stream status.");
        return false;
    }
    _data_stream_status.reset(status);

    if (_config.has_command()) {
        s = create_stream(_config.command());
        if (s == nullptr) {
            ROS_ERROR("Failed to create command stream.");
            return false;
        }
        _command_stream.reset(s);

        status = new Status();
        if (!status) {
            ROS_ERROR("Failed to create command stream status.");
            return false;
        }
        _command_stream_status.reset(status);
    } else {
        _command_stream = _data_stream;
        _command_stream_status = _data_stream_status;
    }
    //--------------------------------------------
    //std::cout <<*s<<std::endl;

    //std::cout << "----------rtk-------------"<<std::endl;
    if (_config.has_rtk_from()) {
        s = create_stream(_config.rtk_from());
        if (s == nullptr) {
            ROS_ERROR("Failed to create rtk_from stream.");
            return false;
        }
        _in_rtk_stream.reset(s);

        status = new Status();
        if (!status) {
            ROS_ERROR("Failed to create rtk_from stream status.");
            return false;
        }
        _in_rtk_stream_status.reset(status);

        if (_config.has_rtk_to()) {
            s = create_stream(_config.rtk_to());
            if (s == nullptr) {
                ROS_ERROR("Failed to create rtk_to stream.");
                return false;
            }
            _out_rtk_stream.reset(s);

            status = new Status();
            if (!status) {
                ROS_ERROR("Failed to create rtk_to stream status.");
                return false;
            }
            _out_rtk_stream_status.reset(status);
        } else {
            _out_rtk_stream = _data_stream;
            _out_rtk_stream_status = _data_stream_status;
        }

        if (_config.has_rtk_solution_type()) {
            if (_config.rtk_solution_type() == config::Config::RTK_SOFTWARE_SOLUTION) {
                _rtk_software_solution = true;
            }
        }
    }
    //std::cout << "----------login-------------"<<std::endl;
    if (_config.login_commands_size() == 0) {
        ROS_WARN("No login_commands in config file.");
    }

    if (_config.logout_commands_size() == 0) {
        ROS_WARN("No logout_commands in config file.");
    }

    // connect and login
    if (!connect()) {
        ROS_ERROR("gnss driver connect failed.");
        return false;
    }

    if (!login()) {
        ROS_ERROR("gnss driver login failed.");
        return false;
    }

    uint32_t pid = fork();
    if(pid == 0){
        prctl(PR_SET_NAME,"gnss_setter");
        while(1){
            pause();
        }
    }
    std::cout << "--------------------pid="<<pid << std::endl;
    _data_thread_ptr.reset(new std::thread(&RawStream::data_spin, this));
    _ntrip_thread_ptr.reset(new std::thread(&RawStream::ntrip_spin, this));

    return true;
}

bool RawStream::connect() {
    if (_data_stream) {
        if (_data_stream->get_status() != Stream::Status::CONNECTED) {
            if (!_data_stream->connect()) {
                ROS_ERROR("data stream connect failed.");
                return false;
            }
            _data_stream_status->status = Stream::Status::CONNECTED;
            _stream_status->set_ins_stream_type(apollo::common::gnss_status::StreamStatus::CONNECTED);
        }
    }

    if (_command_stream) {
        if (_command_stream->get_status() != Stream::Status::CONNECTED) {
            if (!_data_stream->connect()) {
                ROS_ERROR("command stream connect failed.");
                return false;
            }
            _command_stream_status->status = Stream::Status::CONNECTED;
        }
    }

    if (_in_rtk_stream) {
        if (_in_rtk_stream->get_status() != Stream::Status::CONNECTED) {
            if (!_in_rtk_stream->connect()) {
                ROS_ERROR("in rtk stream connect failed.");
            } else {
                _in_rtk_stream_status->status = Stream::Status::CONNECTED;
                _stream_status->set_rtk_stream_in_type(apollo::common::gnss_status::StreamStatus::CONNECTED);
            }
        }
    } else {
        _stream_status->set_rtk_stream_in_type(apollo::common::gnss_status::StreamStatus::CONNECTED);
    }

    if (_out_rtk_stream) {
        if (_out_rtk_stream->get_status() != Stream::Status::CONNECTED) {
            if (!_out_rtk_stream->connect()) {
                ROS_ERROR("out rtk stream connect failed.");
            } else {
                _out_rtk_stream_status->status = Stream::Status::CONNECTED;
                _stream_status->set_rtk_stream_out_type(apollo::common::gnss_status::StreamStatus::CONNECTED);
            }
        }
     } else {
        _stream_status->set_rtk_stream_out_type(apollo::common::gnss_status::StreamStatus::CONNECTED);
    }
    return true;
}

bool RawStream::disconnect() {
    if (_data_stream) {
        if (_data_stream->get_status() == Stream::Status::CONNECTED) {
            if (!_data_stream->disconnect()) {
                ROS_ERROR("data stream disconnect failed.");
                return false;
            }
        }
    }

    if (_command_stream) {
        if (_command_stream->get_status() == Stream::Status::CONNECTED) {
            if (!_data_stream->disconnect()) {
                ROS_ERROR("command stream disconnect failed.");
                return false;
            }
        }
    }
    if (_in_rtk_stream) {
        if (_in_rtk_stream->get_status() == Stream::Status::CONNECTED) {
            if (!_in_rtk_stream->disconnect()) {
                ROS_ERROR("in rtk stream disconnect failed.");
                return false;
            }
        }
    }
    if (_out_rtk_stream) {
        if (_out_rtk_stream->get_status() == Stream::Status::CONNECTED) {
            if (!_out_rtk_stream->disconnect()) {
                ROS_ERROR("out rtk stream disconnect failed.");
                return false;
            }
        }
    }

    return true;
}

bool RawStream::login() {
    std::vector<std::string> login_data;
    for (int i = 0; i < _config.login_commands_size(); ++i) {
        _command_stream->write(_config.login_commands(i));
        login_data.emplace_back(_config.login_commands(i));
        ROS_INFO_STREAM("Login command: " << _config.login_commands(i));
        // sleep a little to avoid overun of the slow serial interface.
        ros::Duration(0.2).sleep();
    }
    _command_stream->register_login_data(login_data);
    return true;
}
/*
void RawStream::sig_handler(int sig) {
    login_setting();
}
*/
void RawStream::init_signal() {
    ROS_INFO("----init_signal-----");
    signal(SIGUSR1, sig_handler);
}
/*
bool RawStream::login_setting() {
    std::vector<std::string> login_setting_data;
    for (int i = 0; i < _config.login_setting_commands_size(); ++i) {
        _command_stream->write(_config.login_setting_commands(i));
        login_setting_data.emplace_back(_config.login_setting_commands(i));
        ROS_INFO_STREAM("Login command: " << _config.login_setting_commands(i));
        // sleep a little to avoid overun of the slow serial interface.
        ros::Duration(0.2).sleep();
    }
    _command_stream->register_login_data(login_setting_data);
    return true;
}
*/
bool RawStream::logout() {
    for (int i = 0; i < _config.logout_commands_size(); ++i) {
        _command_stream->write(_config.logout_commands(i));
        ROS_INFO_STREAM("Logout command: " << _config.logout_commands(i));
    }
    return true;
}

std::string RawStream::status_string(Stream::Status status) {
    switch (status) {
    case Stream::Status::DISCONNECTED:
        return std::string("disconnected");

    case Stream::Status::ERROR:
        return std::string("error");

    default:
        return std::string("ok.");
    }
}

void RawStream::stream_status_check() {
    bool status_report = false;
    apollo::common::gnss_status::StreamStatus_Type report_stream_status;

    if (_data_stream && (_data_stream->get_status() != _data_stream_status->status)) {
        _data_stream_status->status = _data_stream->get_status();
        status_report = true;
        switch_stream_status(_data_stream_status->status, report_stream_status);
        _stream_status->set_ins_stream_type(report_stream_status);
    }

    if (_in_rtk_stream && (_in_rtk_stream->get_status() != _in_rtk_stream_status->status)) {
        _in_rtk_stream_status->status = _in_rtk_stream->get_status();
        status_report = true;
        switch_stream_status(_in_rtk_stream_status->status, report_stream_status);
        _stream_status->set_rtk_stream_in_type(report_stream_status);
    }

    if (_out_rtk_stream && (_out_rtk_stream->get_status() != _out_rtk_stream_status->status)) {
        _out_rtk_stream_status->status = _out_rtk_stream->get_status();
        status_report = true;
        switch_stream_status(_out_rtk_stream_status->status, report_stream_status);
        _stream_status->set_rtk_stream_out_type(report_stream_status);
    }

    if (status_report) {
        _stream_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
        _stream_status_publisher.publish(_stream_status);
    }
}

void RawStream::data_spin() {
    _stream_status->mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    _stream_status_publisher.publish(_stream_status);
    while (ros::ok()) {
        
        size_t length = _data_stream->read(_buffer, BUFFER_SIZE);
        
        if (length > 0) {
            std_msgs::StringPtr msg_pub(new std_msgs::String);
            if (!msg_pub) {
                ROS_ERROR("New data sting msg failed.");
                continue;
            }
            msg_pub->data.assign(reinterpret_cast<const char*>(_buffer), length);
            _raw_data_publisher.publish(msg_pub);
        }//if
        stream_status_check();
    }
}

void RawStream::ntrip_spin() {
    if (_in_rtk_stream == nullptr) {
        return;
    }
    while (ros::ok()) {
        size_t ret = 0;
        size_t length = _in_rtk_stream->read(_buffer_ntrip, BUFFER_SIZE);
        if (length > 0) {
            if (_rtk_software_solution) {
                std_msgs::StringPtr rtkmsg_pub(new std_msgs::String);
                if (!rtkmsg_pub) {
                    ROS_ERROR("New rtkmsg failed.");
                    continue;
                }
                rtkmsg_pub->data.assign(reinterpret_cast<const char*>(_buffer_ntrip), length);
                _rtcm_data_publisher.publish(rtkmsg_pub);
            } else {
                if (_out_rtk_stream == nullptr) {
                    continue;
                }
                ret = _out_rtk_stream->write(_buffer_ntrip, length);
                if (ret != length) {
                    ROS_ERROR_STREAM("Expect write out rtk stream bytes " << length
                                     << " but got " << ret);
                }
            }
        }
    }
}


}
}
}
