#ifndef MAPSKIT_HPP
#define MAPSKIT_HPP

#include <pybind11/pybind11.h>
#include <stdio.h>
#include <iostream>

namespace py = pybind11;

namespace mapskit {

class MapsKit {
public:
    // Constructor
    MapsKit();
    
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
    
private:
    // Private member variables
    // Private member functions
};

} // namespace mapskit

#endif // MAPSKIT_HPP