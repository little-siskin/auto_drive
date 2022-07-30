#include "modules/server/sub_server/server.h"
#include "modules/common/log.h"
#include "gflags/gflags.h"
#include <memory>

int main(int argc, char *argv[]){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::unique_ptr<Server> _server_ptr;
    _server_ptr.reset(new Server(4000));

    if(!_server_ptr->init_server(argc,argv)){
        std::cout<<"initiate server error."<<std::endl;
        return 1;
    }
    _server_ptr->run_server();
    std::cout<<"Server has been closed."<<std::endl;
    return 0;
}