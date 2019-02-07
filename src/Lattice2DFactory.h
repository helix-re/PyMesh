#pragma once
#include <string>
#include <Lattice2D.h>

namespace PyMesh {

class Lattice2DFactory {

public:
    Lattice2DFactory();
    Lattice2D::Ptr create(const double& precision = Lattice2D::DOUBLE_PRECISION);
    Lattice2D::Ptr create(const MatrixFr& mat, const bool& contour, const double& precision = Lattice2D::DOUBLE_PRECISION);
    Lattice2D::Ptr create(const MatrixIr& edges,const MatrixFr& vertices,const double& precision = Lattice2D::DOUBLE_PRECISION);

private:
    Lattice2DFactory(const Lattice2DFactory& other) = delete;
    Lattice2DFactory& operator=(const Lattice2DFactory& other) = delete;
};
}