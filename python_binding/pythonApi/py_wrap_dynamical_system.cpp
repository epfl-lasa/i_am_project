
#include "../pybind11/include/pybind11/pybind11.h"
#include "../pybind11/include/pybind11/eigen.h"


#include "../include/dynamical_system.h"
#include <string>
#include <iostream>

namespace py = pybind11;

PYBIND11_MODULE(py_wrap_dynamical_system, m)
{
    py::class_<hitting_DS>(m, "hitting_DS")
    .def(py::init<Eigen::Vector3f &, Eigen::Vector3f &>()) // constructor
    .def("flux_DS", &hitting_DS::flux_DS)
    .def("linear_DS", &hitting_DS::linear_DS)
    .def("vel_max_DS", &hitting_DS::vel_max_DS)
    .def("get_des_direction", &hitting_DS::get_des_direction)
    .def("get_current_position", &hitting_DS::get_current_position)
    .def("get_DS_attractor", &hitting_DS::get_DS_attractor)
    .def("get_gain", &hitting_DS::get_gain)
    .def("set_des_direction", &hitting_DS::set_des_direction)
    .def("set_current_position", &hitting_DS::set_current_position)
    .def("set_DS_attractor", &hitting_DS::set_DS_attractor)
    .def("set_gain", &hitting_DS::set_gain);
}
