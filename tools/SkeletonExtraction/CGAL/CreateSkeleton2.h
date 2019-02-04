#pragma once

#include <Core/EigenTypedef.h>

#include <SkeletonExtraction/SkeletonEngine.h>

#include <vector>

namespace PyMesh {

class CreateSkeleton2 : public SkeletonEngine
{
    virtual void run(const MatrixFr& points, const std::vector<MatrixFr>& holes);
};

} // end of namespace PyMesh