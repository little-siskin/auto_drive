#include <iostream>
#include <assert.h>
#include "modules/localization/proto/localization.pb.h"
#include "modules/common/proto/gnss_status.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "third_party/ros/include/ros/ros.h"
#include <sys/time.h>
#include <csignal>
#include "thread"




class Rosnode {


private:
    bool ros_start_running;
    bool ros_run_running;
    volatile bool g_ins_detected = false;
    volatile bool g_gnss_detected = false;
    volatile bool g_localization_detected = false;
    volatile bool g_chassis_detected = false;
    
    apollo::common::gnss_status::InsStatus g_ins_status;
    apollo::common::gnss_status::GnssStatus g_gnss_status;
    apollo::localization::LocalizationEstimate g_localization;
    apollo::canbus::ChassisDetail g_chassis;
    std::unique_ptr<ros::NodeHandle> node_handle_;
    ros::Subscriber ins_status_sub;
    ros::Subscriber gnss_status_sub;
    ros::Subscriber localization_sub;
    ros::Subscriber chassis_sub;

    std::unique_ptr<std::thread> _ros_spin_thread_ptr;

    bool is_gps_started;
    void set_gps_status(bool status);

    

public:

    Rosnode();
    ~Rosnode();
    void localization_callback(const apollo::localization::LocalizationEstimate &localization);
    void ins_status_callback(const apollo::common::gnss_status::InsStatus &ins_status);
    void gnss_status_callback(const apollo::common::gnss_status::GnssStatus &gnss_status);
    void chassis_callback(const apollo::canbus::ChassisDetail &chassis);
    apollo::common::gnss_status::GnssStatus get_gnss_status();
    apollo::common::gnss_status::InsStatus get_ins_status();
    apollo::localization::LocalizationEstimate get_localization();
    bool gps_test();
    bool canbus_test();
    bool ros_init(int argc, char *argv[]);
    bool get_gps_status();
};

void server_sigint_handler(int signal_num);

