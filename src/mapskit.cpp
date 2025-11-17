#include <pybind11/pybind11.h>

#include "mapskit/mapskit.hpp"

using namespace mapskit;


MapsKit::MapsKit() {
    // Constructor can be empty for now
}

MapsKit::~MapsKit() {
    // Destructor can be empty for now
}

void MapsKit::test_method(void){
    std::cout << "hi from test_method" << std::endl;
}


PYBIND11_MODULE(mapskit, m) {
    m.doc() = "pybind11 mapskit plugin"; // optional module docstring

    py::class_<MapsKit>(m, "MapsKit")
        .def(py::init<>())
        .def("test_method", &MapsKit::test_method, "A test method");
}
