#pragma once

#include <vector>
#include <set>
#include <Core/Exception.h>
#include <Lattice2D.h>

/*
* Class that computes topological order of vertices in a lattice
* The lattice/graph must be a directed acyclic graph (DAG).
* else it throws an exception
*/
namespace PyMesh
{
namespace LatticeAlgorithms
{
    namespace TopologicalOrder
    { 
        std::vector<unsigned int> compute_full(const Lattice2D::Ptr& lattice);

        std::vector<unsigned int> compute(const Lattice2D::Ptr& lattice, const std::set<unsigned int>& edge_indices);
    }
}
}