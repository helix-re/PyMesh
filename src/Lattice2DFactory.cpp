#include "Lattice2DFactory.h"

using namespace PyMesh;

Lattice2DFactory::Lattice2DFactory()
{
}

Lattice2D::Ptr Lattice2DFactory::create(const double& precision)
{
    return Lattice2D::Ptr(new Lattice2D(precision));
}

Lattice2D::Ptr Lattice2DFactory::create(const MatrixFr& mat, 
                                        const bool& contour, 
                                        const double& precision)
{
    return Lattice2D::Ptr(new Lattice2D(mat,contour,precision));
}

Lattice2D::Ptr Lattice2DFactory::create(const MatrixIr& edges,
                                        const MatrixFr& vertices,
                                        const double& precision)
{
    return Lattice2D::Ptr(new Lattice2D(edges,vertices,precision));
}