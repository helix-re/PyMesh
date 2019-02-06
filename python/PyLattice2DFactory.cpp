#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Lattice2D.h>
#include <Lattice2DFactory.h>

namespace py = pybind11;
using namespace PyMesh;

void init_Lattice2DFactory(py::module &m) {
    py::class_<Lattice2DFactory>(m, "Lattice2DFactory")
    .def(py::init<>())
    .def("create", py::overload_cast<const double&>(&Lattice2DFactory::create), py::return_value_policy::move)
    .def("create", py::overload_cast<const MatrixFr&, const bool&, const double& >(&Lattice2DFactory::create), py::return_value_policy::move)
    .def("create", py::overload_cast<const MatrixIr&, const MatrixFr&, const double& >(&Lattice2DFactory::create), py::return_value_policy::move);
}