#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/iostream.h>

#include <Lattice2D.h>

#include <LatticeAlgorithms/TopologicalOrder.h>
#include <LatticeAlgorithms/SegmentDetection.h>

namespace py = pybind11;
using namespace PyMesh;

void init_LatticeAlgorithms(py::module& m) {
    m.def("lattice_sort_topology",
            &LatticeAlgorithms::TopologicalOrder::compute);
    m.def("lattice_detect_segments",[](const Lattice2D::Ptr& lattice) -> std::vector<std::pair<std::vector<unsigned int>,bool>> {
                py::scoped_ostream_redirect redirout(std::cout, py::module::import("sys").attr("stdout"));
                py::scoped_ostream_redirect redirerr(std::cerr, py::module::import("sys").attr("stderr"));
                auto ret = LatticeAlgorithms::SegmentDetection::compute(lattice);
                std::cout << std::flush;
                std::cerr << std::flush;
                return ret;
            });
}