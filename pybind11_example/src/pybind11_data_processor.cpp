#include <pybind11/detail/common.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pybind11_example/data_processor.hpp"

namespace py = pybind11;
PYBIND11_MODULE(pybind_cpp_data_processor, m)
{
  py::class_<DataProcessor>(m, "DataProcessor")
      .def(py::init())
      .def("setData", &DataProcessor::setData)
      .def("process", &DataProcessor::process);
}
//colcon build --event-handlers console_direct+ --packages-select pybind11_example