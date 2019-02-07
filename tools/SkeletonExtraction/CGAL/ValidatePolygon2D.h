#pragma once

#include <Core/EigenTypedef.h>
#include <Core/Exception.h>
#include <Lattice2D.h>

namespace PyMesh {

class ValidatePolygon2D{

public:

enum class result : unsigned int
{
    SUCCESS = 0,
    EMPTY_LATTICE = 1,
    NOT_SIMPLE = 2,
    COLLINEAR = 3,
    COUNTERCLOCKWISE = 4,
    NON_MANIFOLD = 5
};

void convert_to_lattice( const MatrixFr& points,
                             const std::vector<MatrixFr>& holes,
                             Lattice2D::Ptr& contour_lattice,
                             std::vector<Lattice2D::Ptr>& hole_lattices);

static result compute( const Lattice2D::Ptr& lattice );

};

}