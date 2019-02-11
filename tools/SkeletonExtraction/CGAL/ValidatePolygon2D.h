#pragma once

#include <Core/EigenTypedef.h>
#include <Core/Exception.h>
#include <Lattice2D.h>

namespace PyMesh {

class ValidatePolygon2D{

public:

enum class result : unsigned int
{
    SUCCESS             = 0,
    EMPTY_LATTICE       = 1,
    NOT_SIMPLE          = 2,
    COLLINEAR           = 3,
    NON_MANIFOLD        = 4,
    CLOCKWISE           = 5
    
};

static result compute( const Lattice2D::Ptr& lattice );

static bool Hole_Inside( const Lattice2D::Ptr& lattice, const Lattice2D::Ptr& hole );

};

}