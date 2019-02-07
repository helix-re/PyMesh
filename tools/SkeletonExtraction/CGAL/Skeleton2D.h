#pragma once

#include <vector>
#include <Lattice2D.h>

namespace PyMesh {

/*
* Computes skeleton
* Assumptions :
* 1. all polygons are simple polygons.
* 2. All polygons are oriented correctly.
* 3. No self interections.
* all above issues should be fixed before hand
*/
class Skeleton2D{
public: 
static Lattice2D::Ptr compute(const Lattice2D::Ptr& contour_lattice);

static Lattice2D::Ptr compute(const Lattice2D::Ptr& contour_lattice, 
                              const std::vector<Lattice2D::Ptr>& hole_lattices);

};

}
