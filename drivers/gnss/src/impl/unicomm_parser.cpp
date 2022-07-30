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

// An parser for decoding binary messages from a Unicomm receiver. The following messages must be
// logged in order for this parser to work properly.
//
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include "gnss/parser.h"
#include "gnss/unicomm_messages.h"
#include "proto/gnss.pb.h"
#include "proto/imu.pb.h"
#include "proto/ins.pb.h"

namespace apollo {
namespace drivers {
namespace gnss {

// Anonymous namespace that contains helper constants and functions.
namespace {

constexpr size_t BUFFER_SIZE = 256;

constexpr int SECONDS_PER_WEEK = 60 * 60 * 24 * 7;

constexpr double DEG_TO_RAD = M_PI / 180.0;

constexpr float FLOAT_NAN = std::numeric_limits<float>::quiet_NaN();

// The Unicomm's orientation covariance matrix is pitch, roll, and yaw. We use the index array below
// to convert it to the orientation covariance matrix with order roll, pitch, and yaw.
constexpr int INDEX[] = {4, 3, 5,
                         1, 0, 2,
                         7, 6, 8};
static_assert(sizeof(INDEX) == 9 * sizeof(int), "Incorrect size of INDEX");

template <typename T>
constexpr bool is_zero(T value) {
    return value == static_cast<T>(0);
}

// CRC algorithm from the Unicomm document.
inline uint32_t crc32_word(uint32_t word) {
    for (int j = 0; j < 8; ++j) {
        if (word & 1) {
            word = (word >> 1) ^ 0xEDB88320;
        } else {
            word >>= 1;
        }
    }
    return word;
}

inline uint32_t crc32_block(const uint8_t* buffer, size_t length) {
    uint32_t word = 0;
    while (length--) {
        uint32_t t1 = (word >> 8) & 0xFFFFFF;
        uint32_t t2 = crc32_word((word ^ *buffer++) & 0xFF);
        word = t1 ^ t2;
    }
    return word;
}

// Converts Unicomm's azimuth (north = 0, east = 90) to FLU yaw (east = 0, north = pi/2).
constexpr double azimuth_deg_to_yaw_rad(double azimuth) {
    return (90.0 - azimuth) * DEG_TO_RAD;
}

// A helper that fills an Point3D object (which uses the FLU frame) using RFU measurements.
inline void rfu_to_flu(double r, double f, double u, ::apollo::common::Point3D* flu) {
    flu->set_x(f);
    flu->set_y(-r);
    flu->set_z(u);
}

}  // namespace

class UnicommParser : public Parser {
public:
    UnicommParser();

    virtual MessageType get_message(MessagePtr& message_ptr);

private:
    bool check_crc();

    Parser::MessageType prepare_message(MessagePtr& message_ptr);

    // The handle_xxx functions return whether a message is ready.
    bool handle_best_pos(const unicomm::BestPos* pos, uint16_t gps_week, uint32_t gps_millisecs);

    bool handle_best_vel(const unicomm::BestVel* vel, uint16_t gps_week, uint32_t gps_millisecs);

    bool handle_corr_imu_data(const unicomm::CorrImuData* imu);

    bool handle_ins_cov(const unicomm::InsCov* cov);

    //bool handle_ins_pva(const unicomm::InsPva* pva);
    //new add
    bool handle_ins_pva(const unicomm::DrPva* pva, uint16_t gps_week, uint32_t gps_millisecs);

    bool handle_raw_imu_x(const unicomm::RawImuX* imu);
    bool handle_raw_imu_x_sacrifice(const unicomm::RawImuX* imu, uint16_t gps_week, uint32_t gps_millisecs);
    bool handle_range_sacrifice(uint16_t gps_week, uint32_t gps_millisecs);

    double _gyro_scale = 0.0;

    double _accel_scale = 0.0;

    float _imu_measurement_span = 1.0 / 200.0;
    float _imu_measurement_hz = 200.0;

    // TODO: Get mapping from configuration file.
    int _imu_frame_mapping = 5;

    double _imu_measurement_time_previous = -1.0;

    std::vector<uint8_t> _buffer;

    size_t _header_length = 0;

    size_t _total_length = 0;

    // -1 is an unused value.
    unicomm::SolutionStatus _solution_status = static_cast<unicomm::SolutionStatus>(-1);
    unicomm::SolutionType _position_type = static_cast<unicomm::SolutionType>(-1);
    unicomm::SolutionType _velocity_type = static_cast<unicomm::SolutionType>(-1);
    //unicomm::InsStatus _ins_status = static_cast<unicomm::InsStatus>(-1);
    //new add
    unicomm::SolutionStatus _ins_status = static_cast<unicomm::SolutionStatus>(-1);  

    ::apollo::drivers::gnss::Gnss _gnss;
    ::apollo::drivers::gnss::Imu _imu;
    ::apollo::drivers::gnss::Ins _ins;
};

Parser* Parser::create_unicomm() {
    return new UnicommParser();
}

UnicommParser::UnicommParser() {
    _buffer.reserve(BUFFER_SIZE);
    _ins.mutable_position_covariance()->Resize(9, FLOAT_NAN);
    _ins.mutable_euler_angles_covariance()->Resize(9, FLOAT_NAN);
    _ins.mutable_linear_velocity_covariance()->Resize(9, FLOAT_NAN);
}

Parser::MessageType UnicommParser::get_message(MessagePtr& message_ptr) {
    if (_data == nullptr) {
        return MessageType::NONE;
    }
    //ROS_INFO("get_message");
    while (_data < _data_end) {
        if (_buffer.size() == 0) {  // Looking for SYNC0
            if (*_data == unicomm::SYNC_0) {
                _buffer.push_back(*_data);
            }
            ++_data;
        } else if (_buffer.size() == 1) {  // Looking for SYNC1
            if (*_data == unicomm::SYNC_1) {
                _buffer.push_back(*_data++);
            } else {
                _buffer.clear();
            }
        } else if (_buffer.size() == 2) {  // Looking for SYNC2
            switch (*_data) {
            case unicomm::SYNC_2_LONG_HEADER:
                
                _buffer.push_back(*_data++);
                _header_length = sizeof(unicomm::LongHeader);
                break;
            case unicomm::SYNC_2_SHORT_HEADER:
               
                _buffer.push_back(*_data++);
                _header_length = sizeof(unicomm::ShortHeader);
                break;
            default:
               
                _buffer.clear();
            }
        } else if (_header_length > 0) {  // Working on header.
            if (_buffer.size() < _header_length) {
              
                _buffer.push_back(*_data++);
            } else {
                if (_header_length == sizeof(unicomm::LongHeader)) {
                   
                    _total_length = _header_length + unicomm::CRC_LENGTH +
                            reinterpret_cast<unicomm::LongHeader*>(_buffer.data())->message_length;
                } else if (_header_length == sizeof(unicomm::ShortHeader)) {
                   
                    _total_length = _header_length + unicomm::CRC_LENGTH +
                            reinterpret_cast<unicomm::ShortHeader*>(_buffer.data())->message_length;
                } else {
                    ROS_ERROR("Incorrect _header_length. Should never reach here.");
                    _buffer.clear();
                }
                _header_length = 0;
               
            }
        } else if (_total_length > 0) {
            if (_buffer.size() < _total_length) {  // Working on body.
               
                _buffer.push_back(*_data++);
                continue;
            }
           
            MessageType type = prepare_message(message_ptr);
            _buffer.clear();
            _total_length = 0;
            if (type != MessageType::NONE) {
               
                return type;
            }
        }
    }
    return MessageType::NONE;
}

bool UnicommParser::check_crc() {
    size_t l = _buffer.size() - unicomm::CRC_LENGTH;
    return crc32_block(_buffer.data(), l) == *reinterpret_cast<uint32_t*>(_buffer.data() + l);
}

Parser::MessageType UnicommParser::prepare_message(MessagePtr& message_ptr) {
    if (!check_crc()) {
        ROS_ERROR("CRC check failed.");
        return MessageType::NONE;
    }

    uint8_t* message = nullptr;
    unicomm::MessageId message_id;
    uint16_t message_length;
    uint16_t gps_week;
    uint32_t gps_millisecs;
    if (_buffer[2] == unicomm::SYNC_2_LONG_HEADER) {
        auto header = reinterpret_cast<const unicomm::LongHeader*>(_buffer.data());
        message = _buffer.data() + sizeof(unicomm::LongHeader);
        gps_week = header->gps_week;
        gps_millisecs = header->gps_millisecs;
        message_id = header->message_id;
        message_length = header->message_length;
    } else {
        auto header = reinterpret_cast<const unicomm::ShortHeader*>(_buffer.data());
        message = _buffer.data() + sizeof(unicomm::ShortHeader);
        gps_week = header->gps_week;
        gps_millisecs = header->gps_millisecs;
        message_id = header->message_id;
        message_length = header->message_length;
    }
    switch (message_id) {
    case unicomm::RANGE:
        //ROS_INFO("handle range_sacrifice");
        handle_range_sacrifice(gps_week, gps_millisecs);
        break;
    case unicomm::BESTGNSSPOS:
    case unicomm::BESTPOS:
    case unicomm::PSRPOS:
        //if(message_id == unicomm::BESTPOS)  ROS_INFO("handle BESTPOS");
        //else if(message_id == unicomm::PSRPOS)  ROS_INFO("handle PSRPOS");
        //ROS_ERROR_COND(message_length != sizeof(unicomm::BestPos), "Incorrect message_length");
        if (message_length != sizeof(unicomm::BestPos)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }
        if (handle_best_pos(reinterpret_cast<unicomm::BestPos*>(message), gps_week,
                            gps_millisecs)) {

            //if(message_id == unicomm::BESTPOS)  ROS_INFO("finish BESTPOS");
            //else if(message_id == unicomm::PSRPOS)  ROS_INFO("finish PSRPOS");

            message_ptr = &_gnss;
            return MessageType::GNSS;
        }
        break;

    case unicomm::BESTGNSSVEL:
    case unicomm::BESTVEL:
    case unicomm::PSRVEL:
        ROS_INFO("handle BESTVEL");
        //ROS_ERROR_COND(message_length != sizeof(unicomm::BestVel), "Incorrect message_length");
        if (message_length != sizeof(unicomm::BestVel)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }
        if (handle_best_vel(reinterpret_cast<unicomm::BestVel*>(message), gps_week,
                            gps_millisecs)) {
            message_ptr = &_gnss;
            return MessageType::GNSS;
        }
        break;

    case unicomm::CORRIMUDATA:
    case unicomm::CORRIMUDATAS:
        //ROS_ERROR_COND(message_length != sizeof(unicomm::CorrImuData), "Incorrect message_length");
        if (message_length != sizeof(unicomm::CorrImuData)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_corr_imu_data(reinterpret_cast<unicomm::CorrImuData*>(message))) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    case unicomm::INSCOV:
    case unicomm::INSCOVS:
        //ROS_ERROR_COND(message_length != sizeof(unicomm::InsCov), "Incorrect message_length");
        if (message_length != sizeof(unicomm::InsCov)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_ins_cov(reinterpret_cast<unicomm::InsCov*>(message))) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    //case unicomm::INSPVA:
    //new add
    case unicomm::DRPVA:
    //case unicomm::INSPVAS:
        ROS_INFO("handle DRPVA");
        //ROS_ERROR_COND(message_length != sizeof(unicomm::InsPva), "Incorrect message_length");
        if (message_length != sizeof(unicomm::DrPva)/*+32*/) {
            std::cout << "drpva_message_length = " << (int)message_length<<"sizeof(unicomm::DrPva)"<<sizeof(unicomm::DrPva)<<std::endl;
            ROS_ERROR("Incorrect message_length");
            break;
        }

        if (handle_ins_pva(reinterpret_cast<unicomm::DrPva*>(message), gps_week, gps_millisecs)) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    case unicomm::RAWIMUX:
    case unicomm::RAWIMUSX:
        //ROS_INFO("handle raw_imu_x_sacrifice");
        //ROS_ERROR_COND(message_length != sizeof(unicomm::RawImuX), "Incorrect message_length");
        if (message_length != sizeof(unicomm::RawImuX)) {
            ROS_ERROR("Incorrect message_length");
            break;
        }
        /*
        if (handle_raw_imu_x(reinterpret_cast<unicomm::RawImuX*>(message))) {
            message_ptr = &_imu;
            return MessageType::IMU;

        break;*/
        //new add
        if (handle_raw_imu_x(reinterpret_cast<unicomm::RawImuX*>(message))) {
            message_ptr = &_ins;
            return MessageType::INS;
        }
        break;

    default:
        break;
    }
    return MessageType::NONE;
}


bool UnicommParser::handle_range_sacrifice(uint16_t gps_week, uint32_t gps_millisecs) {
    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    //std::cout << "_gnss.measurement_time()="<<(long long)_gnss.measurement_time()<<" seconds="<<(long long)seconds<<std::endl;
    if (_gnss.measurement_time() != seconds) {
        _gnss.set_measurement_time(seconds);
        return false;
    }
    //ROS_INFO("finish range_sacrifice");
    return true;
}


bool UnicommParser::handle_raw_imu_x_sacrifice(const unicomm::RawImuX* imu, uint16_t gps_week, uint32_t gps_millisecs) {
    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    //std::cout << "_gnss.measurement_time()="<<(long long)_gnss.measurement_time()<<" seconds="<<(long long)seconds<<std::endl;
    if (_gnss.measurement_time() != seconds) {
        _gnss.set_measurement_time(seconds);
        return false;
    }
    //ROS_INFO("finish raw_imu_x_sacrifice");
    return true;
}

bool UnicommParser::handle_best_pos(const unicomm::BestPos* pos, uint16_t gps_week, uint32_t gps_millisecs) {
    _gnss.mutable_position()->set_lon(pos->longitude);
    //std::cout << "--------------pos->lon = " << pos->longitude<<std::endl;
    _gnss.mutable_position()->set_lat(pos->latitude);
    _gnss.mutable_position()->set_height(pos->height_msl + pos->undulation);
    _gnss.mutable_position_std_dev()->set_x(pos->longitude_std_dev);
    _gnss.mutable_position_std_dev()->set_y(pos->latitude_std_dev);
    _gnss.mutable_position_std_dev()->set_z(pos->height_std_dev);
    _gnss.set_num_sats(pos->num_sats_in_solution);
    if (_solution_status != pos->solution_status) {
        _solution_status = pos->solution_status;
        ROS_INFO_STREAM("Solution status: " << static_cast<int>(_solution_status));
    }
    if (_position_type != pos->position_type) {
        _position_type = pos->position_type;
        ROS_INFO_STREAM("Position type: " << static_cast<int>(_position_type));
    }
    _gnss.set_solution_status(static_cast<uint32_t>(pos->solution_status));
    if (pos->solution_status == unicomm::SolutionStatus::SOL_COMPUTED) {
        _gnss.set_position_type(static_cast<uint32_t>(pos->position_type));
        switch (pos->position_type) {
        case unicomm::SolutionType::SINGLE:
        case unicomm::SolutionType::INS_PSRSP:
            _gnss.set_type(apollo::drivers::gnss::Gnss::SINGLE);
            break;
        case unicomm::SolutionType::PSRDIFF:
        case unicomm::SolutionType::WAAS:
        case unicomm::SolutionType::INS_SBAS:
            _gnss.set_type(apollo::drivers::gnss::Gnss::PSRDIFF);
            break;
        case unicomm::SolutionType::FLOATCONV:
        case unicomm::SolutionType::L1_FLOAT:
        case unicomm::SolutionType::IONOFREE_FLOAT:
        case unicomm::SolutionType::NARROW_FLOAT:
        case unicomm::SolutionType::RTK_DIRECT_INS:
        case unicomm::SolutionType::INS_RTKFLOAT:
            _gnss.set_type(apollo::drivers::gnss::Gnss::RTK_FLOAT);
            break;
        case unicomm::SolutionType::WIDELANE:
        case unicomm::SolutionType::NARROWLANE:
        case unicomm::SolutionType::L1_INT:
        case unicomm::SolutionType::WIDE_INT:
        case unicomm::SolutionType::NARROW_INT:
        case unicomm::SolutionType::INS_RTKFIXED:
            _gnss.set_type(apollo::drivers::gnss::Gnss::RTK_INTEGER);
            break;
        case unicomm::SolutionType::OMNISTAR:
        case unicomm::SolutionType::INS_OMNISTAR:
        case unicomm::SolutionType::INS_OMNISTAR_HP:
        case unicomm::SolutionType::INS_OMNISTAR_XP:
        case unicomm::SolutionType::OMNISTAR_HP:
        case unicomm::SolutionType::OMNISTAR_XP:
        case unicomm::SolutionType::PPP_CONVERGING:
        case unicomm::SolutionType::PPP:
        case unicomm::SolutionType::INS_PPP_CONVERGING:
        case unicomm::SolutionType::INS_PPP:
            _gnss.set_type(apollo::drivers::gnss::Gnss::PPP);
            break;
        case unicomm::SolutionType::PROPOGATED:
            _gnss.set_type(apollo::drivers::gnss::Gnss::PROPAGATED);
            break;
        default:
            _gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
        }
    } else {
        _gnss.set_type(apollo::drivers::gnss::Gnss::INVALID);
        _gnss.set_position_type(0);
    }
    if (pos->datum_id != unicomm::DatumId::WGS84) {
        ROS_ERROR_STREAM_THROTTLE(5, "Unexpected Datum Id: " << static_cast<int>(pos->datum_id));
    }

    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    //std::cout <<"gps_week="<<(int)gps_week<<" "<<"gps_millisecs="<<(int)gps_millisecs<<std::endl;
    if (_gnss.measurement_time() != seconds) {
        //ROS_INFO("_gnss.measurement_time() != seconds");
        //std::cout << "_gnss.measurement_time()="<<_gnss.measurement_time()<<"seconds="<<seconds<<std::endl;
        //std::cout << "_gnss.measurement_time()="<<(long long)_gnss.measurement_time()<<" seconds="<<(long long)seconds<<std::endl;
        _gnss.set_measurement_time(seconds);
        //ROS_INFO("_gnss.measurement_time() != seconds");
        //std::cout << "_gnss.measurement_time()="<<_gnss.measurement_time()<<"seconds="<<seconds<<std::endl;
        return false;
    }
    return true;
}

bool UnicommParser::handle_best_vel(const unicomm::BestVel* vel, uint16_t gps_week, uint32_t gps_millisecs) {
    if (_velocity_type != vel->velocity_type) {
        _velocity_type = vel->velocity_type;
        ROS_INFO_STREAM("Velocity type: " << static_cast<int>(_velocity_type));
    }
    if (!_gnss.has_velocity_latency() || _gnss.velocity_latency() != vel->latency) {
        ROS_INFO_STREAM("Velocity latency: " << static_cast<int>(vel->latency));
        _gnss.set_velocity_latency(vel->latency);
    }
    double yaw = azimuth_deg_to_yaw_rad(vel->track_over_ground);
    _gnss.mutable_linear_velocity()->set_x(vel->horizontal_speed * cos(yaw));
    _gnss.mutable_linear_velocity()->set_y(vel->horizontal_speed * sin(yaw));
    _gnss.mutable_linear_velocity()->set_z(vel->vertical_speed);

    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    //std::cout << "_gnss.measurement_time()="<<(long long)_gnss.measurement_time()<<" seconds="<<(long long)seconds<<std::endl;
    if (_gnss.measurement_time() != seconds) {
        _gnss.set_measurement_time(seconds);
        return false;
    }
    //ROS_INFO("finish BESTVEL");
    return true;
}

bool UnicommParser::handle_corr_imu_data(const unicomm::CorrImuData* imu) {
    rfu_to_flu(imu->x_velocity_change * _imu_measurement_hz,
               imu->y_velocity_change * _imu_measurement_hz,
               imu->z_velocity_change * _imu_measurement_hz,
               _ins.mutable_linear_acceleration());
    rfu_to_flu(imu->x_angle_change * _imu_measurement_hz,
               imu->y_angle_change * _imu_measurement_hz,
               imu->z_angle_change * _imu_measurement_hz,
               _ins.mutable_angular_velocity());

    double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }
    _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    return true;
}

bool UnicommParser::handle_ins_cov(const unicomm::InsCov* cov) {
    for (int i = 0; i < 9; ++i) {
        _ins.set_position_covariance(i, cov->position_covariance[i]);
        _ins.set_euler_angles_covariance(INDEX[i],
                                         (DEG_TO_RAD * DEG_TO_RAD) * cov->attitude_covariance[i]);
        _ins.set_linear_velocity_covariance(i, cov->velocity_covariance[i]);
    }
    return false;
}
/*
bool UnicommParser::handle_ins_pva(const unicomm::InsPva* pva) {
    if (_ins_status != pva->status) {
        _ins_status = pva->status;
        ROS_INFO_STREAM("INS status: " << static_cast<int>(_ins_status));
    }
    _ins.mutable_position()->set_lon(pva->longitude);
    _ins.mutable_position()->set_lat(pva->latitude);
    _ins.mutable_position()->set_height(pva->height);
    _ins.mutable_euler_angles()->set_x(pva->roll * DEG_TO_RAD);
    _ins.mutable_euler_angles()->set_y(-pva->pitch * DEG_TO_RAD);
    _ins.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
    _ins.mutable_linear_velocity()->set_x(pva->east_velocity);
    _ins.mutable_linear_velocity()->set_y(pva->north_velocity);
    _ins.mutable_linear_velocity()->set_z(pva->up_velocity);

    switch (pva->status) {
    case unicomm::InsStatus::ALIGNMENT_COMPLETE:
    case unicomm::InsStatus::SOLUTION_GOOD:
        _ins.set_type(apollo::drivers::gnss::Ins::GOOD);
        break;
    case unicomm::InsStatus::ALIGNING:
    case unicomm::InsStatus::HIGH_VARIANCE:
    case unicomm::InsStatus::SOLUTION_FREE:
        _ins.set_type(apollo::drivers::gnss::Ins::CONVERGING);
        break;
    default:
        _ins.set_type(apollo::drivers::gnss::Ins::INVALID);
    }

    double seconds = pva->gps_week * SECONDS_PER_WEEK + pva->gps_seconds;
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }

    _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    return true;
}*/

//new add
bool UnicommParser::handle_ins_pva(const unicomm::DrPva* pva, uint16_t gps_week, uint32_t gps_millisecs) {
    if (_ins_status != pva->solution_status) {
        _ins_status = pva->solution_status;
        ROS_INFO_STREAM("INS status: " << static_cast<int>(_ins_status));
    }
    _ins.mutable_position()->set_lon(pva->longitude);
    _ins.mutable_position()->set_lat(pva->latitude);
    _ins.mutable_position()->set_height(pva->height);
    _ins.mutable_euler_angles()->set_x(pva->roll * DEG_TO_RAD);
    _ins.mutable_euler_angles()->set_y(-pva->pitch * DEG_TO_RAD);
    _ins.mutable_euler_angles()->set_z(azimuth_deg_to_yaw_rad(pva->azimuth));
    _ins.mutable_linear_velocity()->set_x(pva->east_velocity);
    _ins.mutable_linear_velocity()->set_y(pva->north_velocity);
    _ins.mutable_linear_velocity()->set_z(pva->up_velocity);

    switch (pva->solution_status){
        case unicomm::SolutionStatus::SOL_COMPUTED:
            _ins.set_type(apollo::drivers::gnss::Ins::GOOD);
            break;
        default:
            _ins.set_type(apollo::drivers::gnss::Ins::INVALID);
    }

    /*
    double seconds = (double)pva->dr_age;//replaced by dr_age
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }*/
    //double seconds = gps_week * SECONDS_PER_WEEK + (long long)(gps_millisecs * 1e-3);
    double seconds = gps_week * SECONDS_PER_WEEK + gps_millisecs * 1e-3;
    //std::cout << "_ins.measurement_time()="<<(long long)_ins.measurement_time()<<" seconds="<<(long long)seconds<<std::endl;
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }

    _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    //ROS_INFO("finish DRPVA");
    return true;
}

bool UnicommParser::handle_raw_imu_x(const unicomm::RawImuX* imu) {
    if (imu->imu_error != 0) {
        ROS_WARN_STREAM("IMU error. Status: " << std::hex << std::showbase << imu->imuStatus);
    }
    if (is_zero(_gyro_scale)) {
        unicomm::ImuParameter param = unicomm::get_imu_parameter(imu->imu_type);
        ROS_INFO_STREAM("IMU type: " << static_cast<unsigned>(imu->imu_type) << "; "
                  << "Gyro scale: " << param.gyro_scale << "; "
                  << "Accel scale: " << param.accel_scale << "; "
                  << "Sampling rate: " << param.sampling_rate_hz << ".");
      
        if (is_zero(param.sampling_rate_hz)) {
            ROS_ERROR_STREAM_THROTTLE(5, "Unsupported IMU type: " << static_cast<int>(imu->imu_type));
            return false;
        }
        _gyro_scale = param.gyro_scale * param.sampling_rate_hz;
        _accel_scale = param.accel_scale * param.sampling_rate_hz;
        _imu_measurement_hz = param.sampling_rate_hz;
        _imu_measurement_span = 1.0 / param.sampling_rate_hz;
        _imu.set_measurement_span(_imu_measurement_span);
    }

    double time = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    if (_imu_measurement_time_previous > 0.0 &&
        fabs(time - _imu_measurement_time_previous - _imu_measurement_span) > 1e-4) {
        ROS_WARN_STREAM("Unexpected delay between two IMU measurements at: "
                     << time - _imu_measurement_time_previous);
    }
    _imu.set_measurement_time(time);
    switch (_imu_frame_mapping) {
    case 5:  // Default mapping.
        rfu_to_flu(imu->x_velocity_change * _accel_scale,
                   -imu->y_velocity_change_neg * _accel_scale,
                   imu->z_velocity_change * _accel_scale,
                   _ins.mutable_linear_acceleration());
        rfu_to_flu(imu->x_angle_change * _gyro_scale,
                   -imu->y_angle_change_neg * _gyro_scale,
                   imu->z_angle_change * _gyro_scale,
                   _ins.mutable_angular_velocity());
        break;
    case 6:
        rfu_to_flu(-imu->y_velocity_change_neg * _accel_scale,
                   imu->x_velocity_change * _accel_scale,
                   -imu->z_velocity_change * _accel_scale,
                   _ins.mutable_linear_acceleration());
        rfu_to_flu(-imu->y_angle_change_neg * _gyro_scale,
                   imu->x_angle_change * _gyro_scale,
                   -imu->z_angle_change * _gyro_scale,
                   _ins.mutable_angular_velocity());
        break;
        /**------------
        case 5:  // Default mapping.
        rfu_to_flu(imu->x_velocity_change * _accel_scale,
                   -imu->y_velocity_change_neg * _accel_scale,
                   imu->z_velocity_change * _accel_scale,
                   _imu.mutable_linear_acceleration());
        rfu_to_flu(imu->x_angle_change * _gyro_scale,
                   -imu->y_angle_change_neg * _gyro_scale,
                   imu->z_angle_change * _gyro_scale,
                   _imu.mutable_angular_velocity());
        break;
    case 6:
        rfu_to_flu(-imu->y_velocity_change_neg * _accel_scale,
                   imu->x_velocity_change * _accel_scale,
                   -imu->z_velocity_change * _accel_scale,
                   _imu.mutable_linear_acceleration());
        rfu_to_flu(-imu->y_angle_change_neg * _gyro_scale,
                   imu->x_angle_change * _gyro_scale,
                   -imu->z_angle_change * _gyro_scale,
                   _imu.mutable_angular_velocity());
        break;
        ----------------*/
    default:
        ROS_ERROR_STREAM_THROTTLE(5, "Unsupported IMU frame mapping: " << _imu_frame_mapping);
        ROS_INFO("RAWIMU error");
    }

    //*----------------
    //new add
    //double seconds = imu->gps_week * SECONDS_PER_WEEK + (long long)((imu->gps_seconds));
    double seconds = imu->gps_week * SECONDS_PER_WEEK + imu->gps_seconds;
    //std::cout << "_ins.measurement_time()="<<(long long)_ins.measurement_time()<<" seconds="<<(long long)seconds<<std::endl;
    if (_ins.measurement_time() != seconds) {
        _ins.set_measurement_time(seconds);
        return false;
    }
    _ins.mutable_header()->set_timestamp_sec(ros::Time::now().toSec());
    //return true;
    //*-------------------

    _imu_measurement_time_previous = time;
    //ROS_INFO("finish RAWIMU");
    return true;
}


}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
