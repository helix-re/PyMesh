#pragma once

#include <vector>
#include <set>
#include <Core/Exception.h>
#include <Lattice2D.h>

/*
* detects all line segments..
*/
namespace PyMesh
{
namespace LatticeAlgorithms
{
    namespace SegmentDetection
    {
        /*
        * returns vector of line segments
        * set contains edge indices
        * bool indicates if it is a closed loop or not
        */ 
        std::vector<std::pair<std::vector<unsigned int>,bool>> 
                compute(const Lattice2D::Ptr& lattice);

    }
}
}