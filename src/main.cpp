#include <iostream>
#include <sys/wait.h>

#include "mapskit/mapskit.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{

    std::cout << "Cofigured Prefix Path: " << MAPSKIT_CONFIG_PATH << std::endl;
    std::cout << "Cofigured Launch Path: " << MAPSKIT_LAUNCH_PATH << std::endl;

    mapskit::MapsKit mk;
    int pid = mk.run_server();
    std::cout << "pid is" << pid << " \n";
    waitpid(pid, nullptr, 0);
    // rclcpp::init(argc, argv);
    // auto node = std::make_shared<rclcpp::Node>("mapskit_core_node");
    // rclcpp::spin(node);
    // rclcpp::shutdown();

    return 0;   
}
