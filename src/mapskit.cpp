#include <pybind11/pybind11.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fstream>

#include "mapskit/mapskit.hpp"
#include <fkYAML/node.hpp>
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

void MapsKit::set_launch_path(std::string path){
    launch_path_ = path;
}

int MapsKit::run_server(void){
    // custom config
    if(config_path_ != std::string(1, '\0')){
        std::cout << "launch using custom config at path=" << config_path_ << "\n";
        
        std::ifstream ifs(config_path_);
        fkyaml::node root = fkyaml::node::deserialize(ifs);
        std::string cloud_in_topic = root["mapskit_voxel_server"]["ros__parameters"]["cloud_in_topic"].get_value<std::string>();
        std::string voxelmap_full_topic = root["mapskit_voxel_server"]["ros__parameters"]["voxelmap_full_topic"].get_value<std::string>();
        std::string voxelmap_binary_topic = root["mapskit_voxel_server"]["ros__parameters"]["voxelmap_binary_topic"].get_value<std::string>();
        std::string voxelmap_centers_topic = root["mapskit_voxel_server"]["ros__parameters"]["voxelmap_centers_topic"].get_value<std::string>();

        pid_t pid = fork();
        if (pid == -1) {
            return -1;
        } else if (pid == 0) {
            // Child process
            std::string command = "source /opt/ros/humble/setup.bash && ros2 launch " + launch_path_ + " params_file:=" + config_path_;
            command += " cloud_in_topic:=" + cloud_in_topic;
            command += " voxelmap_full_topic:=" + voxelmap_full_topic;
            command += " voxelmap_binary_topic:=" + voxelmap_binary_topic;
            command += " voxelmap_centers_topic:=" + voxelmap_centers_topic;

            execl("/bin/bash", "bash", "-c", command.c_str(), (char *)NULL);
            _exit(127);
        }
        server_pid_ = pid;
        return pid;
    }
    // default config
    else{
        std::cout << "launch using default config" << "\n";
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
        .def("set_launch_path", &MapsKit::set_launch_path, "Set the launch file path")
        .def("run_server", &MapsKit::run_server, "Run the server");
}
