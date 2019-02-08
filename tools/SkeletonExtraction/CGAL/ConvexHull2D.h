#pragma once


#include <Lattice2D.h>

namespace PyMesh {

class ConvexHull2D{

public:

static Lattice2D::Ptr compute(const Lattice2D::Ptr& input);

};

}
