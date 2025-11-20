#ifndef MAPSKIT_HPP
#define MAPSKIT_HPP

/* pybind11/pybind11.h needs to be the 1st include file */
#include <pybind11/pybind11.h>

#include <stdio.h>
#include <iostream>
#include <fstream>

// #include <fkYAML/node.hpp>

namespace py = pybind11;

namespace mapskit {

class MapsKit {
public:
    // Constructor
    MapsKit();
    MapsKit(std::string conf_yml_pth);
    
    // Destructor
    ~MapsKit();
    
    

    // Copy constructor
    MapsKit(const MapsKit& other);
    
    // Copy assignment operator
    MapsKit& operator=(const MapsKit& other);
    
    // Move constructor
    MapsKit(MapsKit&& other) noexcept;
    
    // Move assignment operator
    MapsKit& operator=(MapsKit&& other) noexcept;
    
    // Public member functions
    void initialize();
    void test_method(void);
    int run_server(void);
    void shutdown_server(void);    

private:
    // Private member variables
    // Private member functions
    std::string config_path_{'\0'};
    const std::string launch_path_ = MAPSKIT_LAUNCH_PATH;
    volatile int server_pid_{0};
};

} // namespace mapskit

#endif // MAPSKIT_HPP