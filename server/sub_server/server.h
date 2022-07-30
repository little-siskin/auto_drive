#include <stdio.h>   
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <unistd.h>   
#include <errno.h>   
#include <string.h>   
#include <stdlib.h>   
#include "thread"
#include <utility>

#include <string>
#include <sstream>

#include "modules/server/rosnode/rosnode.h"
#include "modules/server/proto/localization_gnss_status.pb.h"




class Server {

private:
    int sock_fd;
    struct sockaddr_in addr_serv;
    struct sockaddr_in addr_client;
    int len;
    int recv_num;
    int send_num;
    int m_port;
    char send_buf[4096];
    char recv_buf[4096];
    char status_buf[4096];
    bool connected;
    const char *connect_command = "connect";
    const char *disconnect_command = "disconnect";
    const char *test_gps_command = "test_gps";
    const char *test_canbus_command = "test_canbus";
    const char *run_server_once_command = "run_server_once";
    const char *shut_down_command = "shut_down";
    const char *set_gnss_command = "set_gnss"; 

    
private:
    void set_connected(bool status);
    bool get_connected();
    std::string compose_localization_gnss_status();
    void ros_spin();
    bool send_protocol(std::string status);
    void recv_command();
    bool is_connected();
public:
    std::unique_ptr<Rosnode> _rosnode_ptr;
    std::unique_ptr<std::thread> _recv_thread_ptr;
    std::unique_ptr<std::thread> _send_thread_ptr;
    std::unique_ptr<std::thread> _ros_spin_thread_ptr;
    
public:
    Server(int port);
    ~Server();

    bool init_server(int argc, char *argv[]);
    void run_server();
};
