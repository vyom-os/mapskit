#include <pybind11/pybind11.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include "mapskit/mapskit.hpp"

using namespace mapskit;


MapsKit::MapsKit() {
    // Constructor can be empty for now
}

MapsKit::MapsKit(std::string custom_conf_pth) {
    config_path_ = custom_conf_pth;
}

MapsKit::~MapsKit() {
    // Destructor can be empty for now
}

void MapsKit::test_method(void){
    std::cout << "hi from test_method" << std::endl;
}

int MapsKit::run_server(void){
    // custom config
    if(config_path_ != std::string(1, '\0')){
        pid_t pid = fork();
        if (pid == -1) {
            return -1;
        } else if (pid == 0) {
            // Child process
            std::string command = "source /opt/ros/humble/setup.bash && ros2 launch " + launch_path_ + " param_file:=" + config_path_;
            execl("/bin/bash", "bash", "-c", command.c_str(), (char *)NULL);
            _exit(127);
        }
        server_pid_ = pid;
        return pid;
    }
    // default config
    else{
        pid_t pid = fork();
        if (pid == -1) {
            return -1;
        } else if (pid == 0) {
            // Child process
            std::string command = "source /opt/ros/humble/setup.bash && ros2 launch " + launch_path_;
            execl("/bin/bash", "bash", "-c", command.c_str(), (char *)NULL);
            _exit(127);
        }
        server_pid_ = pid;
        return pid;
    }
    return -1;
}

PYBIND11_MODULE(mapskit_core_py, m) {
    m.doc() = "pybind11 mapskit plugin"; // optional module docstring

    py::class_<MapsKit>(m, "MapsKit")
        .def(py::init<>())
        .def(py::init<std::string>())
        .def("test_method", &MapsKit::test_method, "A test method")
        .def("run_server", &MapsKit::run_server, "Run the server");
}
