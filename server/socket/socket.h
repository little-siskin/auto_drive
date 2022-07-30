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




class Socket {

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

private:
    const char* connect_command = "connect";
    const char* disconnect_command = "disconnect";
    const char* test_gps_command = "test_gps";
    const char* run_gps_command = "run_gps";
    const char* shut_down_command = "shut_down";
    void set_connected(bool status);
    bool get_connected();
    std::string compose_localization_gnss_status(apollo::common::gnss_status::GnssStatus &gnss_status
                                            ,apollo::localization::LocalizationEstimate &localization);

public:
    std::unique_ptr<Rosnode> _rosnode_ptr;
    std::unique_ptr<std::thread> _recv_thread_ptr;
    std::unique_ptr<std::thread> _send_thread_ptr;

    
public:
    Socket(int port);
    ~Socket();

    void test_status();
    bool send_status(std::string status);
    void recv_command();
    bool init_socket(int argc, char *argv[]);
    bool is_connected();
    void run_socket();
};
