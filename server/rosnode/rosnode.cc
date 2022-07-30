#include "modules/server/rosnode/rosnode.h"
#include <semaphore.h>


Rosnode::Rosnode(){
    set_gps_status(false);
}

Rosnode::~Rosnode(){}

void Rosnode::localization_callback(const apollo::localization::LocalizationEstimate &localization){
    g_localization.CopyFrom(localization);
    g_localization_detected = true;
}


void Rosnode::ins_status_callback(const apollo::common::gnss_status::InsStatus &ins_status){
    g_ins_status.CopyFrom(ins_status);
    g_ins_detected = true;
}

void Rosnode::gnss_status_callback(const apollo::common::gnss_status::GnssStatus &gnss_status){
    g_gnss_status.CopyFrom(gnss_status);
    g_gnss_detected = true;
}

void Rosnode::chassis_callback(const apollo::canbus::ChassisDetail &chassis){
    g_chassis.CopyFrom(chassis);
    g_chassis_detected = true;
    std::cout << "Detect chassis." << std::endl;
}


bool Rosnode::gps_test(){
    bool timeout = false;
    struct timeval start_time;
    struct timeval now;
    uint64_t timeout_sec = 5;
    g_ins_detected = false;
    g_gnss_detected = false;

    gettimeofday(&start_time, NULL);

    //static ros::Rate loop_rate(10);

    //while (!(timeout) && ((!g_ins_detected) || (!g_gnss_detected)||(!g_localization_detected))) {
    while (!(timeout) && ((!g_ins_detected) || (!g_gnss_detected))) {
        ros::spinOnce();
        gettimeofday(&now, NULL);
        if (static_cast<uint64_t>(now.tv_sec * 1000000 + now.tv_usec
                                - start_time.tv_sec * 1000000
                                - start_time.tv_usec)
            > static_cast<uint64_t>(timeout_sec * 1000000)) {
            std::cout << "Detect timeout." << std::endl;
            timeout = true;
        }
    }
    /* if(g_ins_status.type() == apollo::common::gnss_status::InsStatus::GOOD){
        set_gps_status(true);
        return true;
    }
    else{ // timeout or InsStatus wrong
        //std::cout << "g_ins_status.type()" << g_ins_status.type() << std::endl;
        //std::cout << "g_gnss_status.type()" << g_gnss_status.position_type() << std::endl;
        set_gps_status(false);
        return false;
    } */
    if(timeout){
        return false;
    }
    else{
        return true;
    }
}




void server_sigint_handler(int signal_num){
    std::cout << "Received signal: " << signal_num << std::endl;
    if (signal_num != SIGINT) {
        return;
    }
    ros::shutdown();
    std::cout << "Server rosnode has already been closed."<< std::endl;
}

bool Rosnode::ros_init(int argc, char *argv[]){

    ros::init(argc, argv, std::string("server"));
    this->node_handle_.reset(new ros::NodeHandle());
    signal(SIGINT,server_sigint_handler);
    localization_sub = this->node_handle_->subscribe("/apollo/localization/pose",1, &Rosnode::localization_callback,this);
    ins_status_sub = this->node_handle_->subscribe("/apollo/sensor/gnss/ins_status",1, &Rosnode::ins_status_callback,this);
    gnss_status_sub = this->node_handle_->subscribe("/apollo/sensor/gnss/gnss_status",1, &Rosnode::gnss_status_callback,this);
    chassis_sub = this->node_handle_->subscribe("/apollo/canbus/chassis_detail",1,&Rosnode::chassis_callback,this);
    return true;
}

void Rosnode::set_gps_status(bool status){
    is_gps_started = status;
}

bool Rosnode::get_gps_status(){
    return is_gps_started;
}


apollo::common::gnss_status::GnssStatus Rosnode::get_gnss_status(){
    return g_gnss_status;
}

apollo::common::gnss_status::InsStatus Rosnode::get_ins_status(){
    return g_ins_status;
}

apollo::localization::LocalizationEstimate Rosnode::get_localization(){
    return g_localization;
}


