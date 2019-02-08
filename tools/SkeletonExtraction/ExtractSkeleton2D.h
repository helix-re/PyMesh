#pragma once

#include <Core/EigenTypedef.h>

#include "SkeletonEngine.h"

#include "CGAL/ValidatePolygon2D.h"

#include <vector>

namespace PyMesh {

class ExtractSkeleton2D : public SkeletonEngine
{

public:
    virtual void run(const MatrixFr& points, const std::vector<MatrixFr>& holes);
protected:

    /*
    * validates contour and fixes it possilbe
    * 
    */
    ValidatePolygon2D::result validate_and_fix( Lattice2D::Ptr& lattice );
};

} // end of namespace PyMesh