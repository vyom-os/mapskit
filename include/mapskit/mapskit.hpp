#ifndef MAPSKIT_HPP
#define MAPSKIT_HPP


#include <stdio.h>
#include <iostream>
#include <pybind11/pybind11.h>

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
    
private:
    // Private member variables
    void test_method(void);
    // Private member functions
};

} // namespace mapskit

#endif // MAPSKIT_HPP