#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Lattice2D.h>

namespace py = pybind11;
using namespace PyMesh;

void init_Lattice2D(py::module &m) {
    py::class_<Lattice2D, std::shared_ptr<Lattice2D> >(m, "Lattice2D")
        .def("AddEdge", &Lattice2D::AddEdge)
        .def("BuildConnections", &Lattice2D::BuildConnections)
        .def("GetLattice", &Lattice2D::GetLattice)
        .def("GetVertexIndex", &Lattice2D::GetVertexIndex)
        .def("GetVertex", &Lattice2D::GetVertex)
        .def("GetEdge", &Lattice2D::GetEdge)
        .def("GetVertexConnections", &Lattice2D::GetVertexConnections)
        .def("GetEdgeConnections", &Lattice2D::GetEdgeConnections);
}