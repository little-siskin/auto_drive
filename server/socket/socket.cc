#include "modules/server/socket/socket.h"
#include <semaphore.h>
using namespace std;



extern sem_t sem_1;
extern sem_t sem_2;

Socket::Socket(int port){
    m_port = port;
    _rosnode_ptr.reset(new Rosnode());
    set_connected(false);
}


Socket::~Socket(){
    set_connected(false);
    close(sock_fd);
}


bool Socket::init_socket(int argc, char *argv[]){
    _rosnode_ptr->ros_init(argc,argv); //initiate rosnode

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    cout << "socket_fd = "<< sock_fd << endl;
    if(sock_fd < 0){
        cout<< "Socket initial error." << endl;
        return false;
    }
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));  
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(m_port);
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
    len = sizeof(addr_serv);
    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0){
        cout << "Socket bind false." << endl;
        return false;
    }
    return true;
}

void Socket::run_socket(){
    _recv_thread_ptr.reset(new thread(&Socket::recv_command,this));
    _recv_thread_ptr->join();
}


std::string Socket::compose_localization_gnss_status(apollo::common::gnss_status::GnssStatus &gnss_status,apollo::localization::LocalizationEstimate &localization){
    /*
        assign gnss_status
    */
    std::string result_string = "";

    std::string gnss_timestamp_sec_string = to_string(gnss_status.mutable_header()->timestamp_sec());
    std::string gnss_solution_completed_string = to_string(gnss_status.solution_completed());
    std::string gnss_solution_status_string = to_string(gnss_status.solution_status());
    std::string gnss_position_type_string = to_string(gnss_status.position_type());
    std::string gnss_num_sats_string = to_string(gnss_status.num_sats());

    /*
        assign localization
    */

    std::string localization_timestamp_sec_string = to_string(localization.mutable_header()->timestamp_sec());
    std::string localization_module_name_string = localization.mutable_header()->module_name();
    std::string localization_sequence_num_string = to_string(localization.mutable_header()->sequence_num());
    std::string localization_position_x_string = to_string(localization.mutable_pose()->mutable_position()->x());
    std::string localization_position_y_string = to_string(localization.mutable_pose()->mutable_position()->y());
    std::string localization_position_z_string = to_string(localization.mutable_pose()->mutable_position()->z());
    std::string localization_orientation_qx_string = to_string(localization.mutable_pose()->mutable_orientation()->qx());
    std::string localization_orientation_qy_string = to_string(localization.mutable_pose()->mutable_orientation()->qy());
    std::string localization_orientation_qz_string = to_string(localization.mutable_pose()->mutable_orientation()->qz());
    std::string localization_orientation_qw_string = to_string(localization.mutable_pose()->mutable_orientation()->qw());
    std::string localization_linear_velocity_x_string = 
                            to_string(localization.mutable_pose()->mutable_linear_velocity()->x());
    std::string localization_linear_velocity_y_string = 
                            to_string(localization.mutable_pose()->mutable_linear_velocity()->y());
    std::string localization_linear_velocity_z_string = 
                            to_string(localization.mutable_pose()->mutable_linear_velocity()->z());
    std::string localization_linear_acceleration_x_string = 
                            to_string(localization.mutable_pose()->mutable_linear_acceleration()->x());
    std::string localization_linear_acceleration_y_string = 
                            to_string(localization.mutable_pose()->mutable_linear_acceleration()->y());
    std::string localization_linear_acceleration_z_string = 
                            to_string(localization.mutable_pose()->mutable_linear_acceleration()->z());
    std::string localization_angular_velocity_x_string = 
                            to_string(localization.mutable_pose()->mutable_angular_velocity()->x());
    std::string localization_angular_velocity_y_string = 
                            to_string(localization.mutable_pose()->mutable_angular_velocity()->y());
    std::string localization_angular_velocity_z_string = 
                            to_string(localization.mutable_pose()->mutable_angular_velocity()->z());
    // new add
    std::string localization_heading_string;
    std::string localization_raw_heading_string;
    std::string localization_raw_pitch_string;
    std::string localization_raw_roll_string;
    localization_heading_string = 
                            to_string(localization.mutable_pose()->heading());
    localization_raw_heading_string = 
                            to_string(localization.mutable_pose()->raw_heading());                        
    localization_raw_pitch_string = 
                            to_string(localization.mutable_pose()->raw_pitch());
    localization_raw_roll_string = 
                            to_string(localization.mutable_pose()->raw_roll());
    result_string = "#" + gnss_timestamp_sec_string + ","
                        + gnss_solution_completed_string + ","
                        + gnss_solution_status_string + ","
                        + gnss_position_type_string + ","
                        + gnss_num_sats_string + ","
                        + localization_timestamp_sec_string + ","
                        + localization_module_name_string + ","
                        + localization_sequence_num_string + ","
                        + localization_position_x_string + ","
                        + localization_position_y_string + ","
                        + localization_position_z_string + ","
                        + localization_orientation_qx_string + ","
                        + localization_orientation_qy_string + ","
                        + localization_orientation_qz_string + ","
                        + localization_orientation_qw_string + ","
                        + localization_linear_velocity_x_string + ","
                        + localization_linear_velocity_y_string + ","
                        + localization_linear_velocity_z_string + ","
                        + localization_linear_acceleration_x_string + ","
                        + localization_linear_acceleration_y_string + ","
                        + localization_linear_acceleration_z_string + ","
                        + localization_angular_velocity_x_string + ","
                        + localization_angular_velocity_y_string + ","
                        + localization_angular_velocity_z_string + ","
                        + localization_heading_string + ","
                        + localization_raw_heading_string + ","
                        + localization_raw_pitch_string + ","
                        + localization_raw_roll_string;
    
    return result_string;
}

void Socket::test_status(){

    std::string result_string = "";
    std::string gnss_timestamp_sec_string;
    std::string gnss_solution_completed_string;
    std::string gnss_solution_status_string;
    std::string gnss_position_type_string;
    std::string gnss_num_sats_string;
    std::string localization_timestamp_sec_string;
    std::string localization_module_name_string;
    std::string localization_sequence_num_string;
    std::string localization_position_x_string;
    std::string localization_position_y_string;
    std::string localization_position_z_string;
    std::string localization_orientation_qx_string;
    std::string localization_orientation_qy_string;
    std::string localization_orientation_qz_string;
    std::string localization_orientation_qw_string;
    std::string localization_linear_velocity_x_string;
    std::string localization_linear_velocity_y_string;
    std::string localization_linear_velocity_z_string;
    std::string localization_linear_acceleration_x_string;
    std::string localization_linear_acceleration_y_string;
    std::string localization_linear_acceleration_z_string;
    std::string localization_angular_velocity_x_string;
    std::string localization_angular_velocity_y_string;
    std::string localization_angular_velocity_z_string;
    std::string localization_heading_string;
    // new add
    std::string localization_raw_heading_string;
    std::string localization_raw_pitch_string;
    std::string localization_raw_roll_string;

    
    //apollo::server::LocalizationGnssStatus* ptr_localization_gnss_status = new apollo::server::LocalizationGnssStatus;
    //int index = 1;
    while(get_connected()){
        sem_wait(&sem_1); 
        if(get_connected()){
                result_string = "";

                /*
                    assign gnss_status 
                */

                gnss_timestamp_sec_string = to_string(_rosnode_ptr->get_gnss_status().mutable_header()->timestamp_sec());
                gnss_solution_completed_string = to_string(_rosnode_ptr->get_gnss_status().solution_completed());
                gnss_solution_status_string = to_string(_rosnode_ptr->get_gnss_status().solution_status());
                gnss_position_type_string = to_string(_rosnode_ptr->get_gnss_status().position_type());
                gnss_num_sats_string = to_string(_rosnode_ptr->get_gnss_status().num_sats());

                /*
                    assign localization
                */

                localization_timestamp_sec_string = to_string(_rosnode_ptr->get_localization().mutable_header()->timestamp_sec());
                localization_module_name_string = _rosnode_ptr->get_localization().mutable_header()->module_name();
                localization_sequence_num_string = to_string(_rosnode_ptr->get_localization().mutable_header()->sequence_num());
                localization_position_x_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_position()->x());
                localization_position_y_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_position()->y());
                localization_position_z_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_position()->z());
                localization_orientation_qx_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_orientation()->qx());
                localization_orientation_qy_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_orientation()->qy());
                localization_orientation_qz_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_orientation()->qz());
                localization_orientation_qw_string = to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_orientation()->qw());
                localization_linear_velocity_x_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_linear_velocity()->x());
                localization_linear_velocity_y_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_linear_velocity()->y());
                localization_linear_velocity_z_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_linear_velocity()->z());
                localization_linear_acceleration_x_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_linear_acceleration()->x());
                localization_linear_acceleration_y_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_linear_acceleration()->y());
                localization_linear_acceleration_z_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_linear_acceleration()->z());
                localization_angular_velocity_x_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_angular_velocity()->x());
                localization_angular_velocity_y_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_angular_velocity()->y());
                localization_angular_velocity_z_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->mutable_angular_velocity()->z());
                localization_heading_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->heading());
                localization_raw_heading_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->raw_heading());                        
                localization_raw_pitch_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->raw_pitch());
                localization_raw_roll_string = 
                                        to_string(_rosnode_ptr->get_localization().mutable_pose()->raw_roll());
                result_string = "#" + gnss_timestamp_sec_string + ","
                                    + gnss_solution_completed_string + ","
                                    + gnss_solution_status_string + ","
                                    + gnss_position_type_string + ","
                                    + gnss_num_sats_string + ","
                                    + localization_timestamp_sec_string + ","
                                    + localization_module_name_string + ","
                                    + localization_sequence_num_string + ","
                                    + localization_position_x_string + ","
                                    + localization_position_y_string + ","
                                    + localization_position_z_string + ","
                                    + localization_orientation_qx_string + ","
                                    + localization_orientation_qy_string + ","
                                    + localization_orientation_qz_string + ","
                                    + localization_orientation_qw_string + ","
                                    + localization_linear_velocity_x_string + ","
                                    + localization_linear_velocity_y_string + ","
                                    + localization_linear_velocity_z_string + ","
                                    + localization_linear_acceleration_x_string + ","
                                    + localization_linear_acceleration_y_string + ","
                                    + localization_linear_acceleration_z_string + ","
                                    + localization_angular_velocity_x_string + ","
                                    + localization_angular_velocity_y_string + ","
                                    + localization_angular_velocity_z_string + ","
                                    + localization_heading_string + ","
                                    + localization_raw_heading_string + ","
                                    + localization_raw_pitch_string + ","
                                    + localization_raw_roll_string;
                
                send_status(result_string);
                //index++;
            }
    }
}


bool Socket::send_status(std::string status){
    strcpy(status_buf,status.c_str());
    send_num = sendto(sock_fd, status_buf, status.length(), 0, (struct sockaddr *)&addr_client, (socklen_t)sizeof(addr_client));  
    if(send_num < 0){
        cout << "Socket sendto error." << endl;
        return false;   
    }
    return true;
}

void Socket::set_connected(bool status){
    connected = status;
}

bool Socket::get_connected(){
    return connected;
}


void Socket::recv_command(){

    while(1){
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&(addr_client), (socklen_t *)&(len)); 
        recv_buf[recv_num] = '\0';
        printf("recv_buf=%s\n",recv_buf);
        if(recv_num < 0) {
            cout << "Socket recvfrom error. " << recv_num <<" "<<sock_fd<< endl;
            set_connected(false);
        }
        else{
            if(strcmp(recv_buf,connect_command) == 0){
                set_connected(true);
                send_status(std::string("@connected."));
                cout << "connected"<<endl;
            }
            else if(strcmp(recv_buf,disconnect_command) == 0){
                set_connected(false);
                send_status(std::string("@disconnected."));
                cout << "disconnected"<<endl;
            }
            else if(strcmp(recv_buf,test_gps_command) == 0){
                if(sem_init(&sem_1, 0, 0) < 0){
                    std::cout << "initiate semaphore 1 fail." << std::endl;
                }
                if(sem_init(&sem_2, 0, 0) < 0){
                    std::cout << "initiate semaphore 2 fail." << std::endl;
                }
                //set_connected(true);
                //printf("recv_buf=%s\n",recv_buf);
                //_send_thread_ptr.reset(new std::thread(&Socket::test_status,this));
                if(!_rosnode_ptr->gps_test()){
                    send_status(string("@test gps fail."));
                }
                else{
                    send_status(string("@test gps successful."));
                }
                //set_connected(false);
                
                //_send_thread_ptr->join();  
                sem_destroy(&sem_1);
                sem_destroy(&sem_2);
            }
            else if(strcmp(recv_buf,run_gps_command) == 0){
                if(sem_init(&sem_1, 0, 0) < 0){
                    std::cout << "initiate semaphore 1 fail." << std::endl;
                }
                if(sem_init(&sem_2, 0, 0) < 0){
                    std::cout << "initiate semaphore 2 fail." << std::endl;
                }
                set_connected(true);
                //printf("recv_buf=%s\n",recv_buf);
                _send_thread_ptr.reset(new std::thread(&Socket::test_status,this));
                ros::spin();
                set_connected(false);
                sem_post(&sem_1);
                _send_thread_ptr->join();
                //if(_rosnode_ptr->get_gps_status()){
                //    send_status(string("@run gps fail."));
                //}
                //else{
                    //_send_thread_ptr.reset(new std::thread(&Socket::test_status,this));
                    //_rosnode_ptr->ros_run();
                  //  send_status(string("@run gps successfully."));
                    //ros::spin();
                   // _send_thread_ptr->join(); 
                //}
                //set_connected(false);
                //sem_post(&sem_2);
                //_send_thread_ptr->join();  
                sem_destroy(&sem_1);
                sem_destroy(&sem_2);
                break;
            }
            else if(strcmp(recv_buf,shut_down_command) == 0){
                std::cout << "shut down." << std::endl;
                send_status(std::string("@shut down."));
                break;
            }
        }
    }
}

bool Socket::is_connected(){
    if(this->connected == true){
        return true;
    }
    else{
        return false;
    }
}



