#include "TopologicalOrder.h"

#include <boost/graph/topological_sort.hpp>
#include <boost/config.hpp>
#include <iostream>
#include <list>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <iterator>
#include <utility>

using namespace boost;
using namespace PyMesh;

using namespace LatticeAlgorithms;
 
/* Topological sort will need to color the graph.  Here we use an
*  internal decorator, so we "property" the color to the graph.
*/
typedef boost::adjacency_list<vecS, vecS, directedS, property<vertex_color_t, default_color_type> > Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

std::vector<unsigned int>  TopologicalOrder::compute_full(const Lattice2D::Ptr& lattice)
{
    std::vector<unsigned int> result;

    auto num_edges = lattice->GetNumEdges();
    std::vector<std::pair<unsigned int ,unsigned int>> edges(num_edges);

    for( std::size_t index = 0; index < num_edges; ++index )
    {
        edges[index] = lattice->GetEdge(index);
    }

    Graph G(edges.begin(), edges.end(), num_edges);

    std::vector< Vertex > vertices;

    topological_sort(G, std::back_inserter(vertices));

    for (auto it=vertices.rbegin(); it != vertices.rend(); ++it)
    result.push_back(*it);

    return result;
}

std::vector<unsigned int> TopologicalOrder::compute(
    const Lattice2D::Ptr& lattice, 
    const std::set<unsigned int>& edge_indices)
{
    std::vector<unsigned int> result;

    auto num_edges = edge_indices.size();
    std::vector<std::pair<unsigned int ,unsigned int>> edges;

    for( const auto& edge_index : edge_indices )
    {
        edges.push_back(lattice->GetEdge(edge_index));
    }

    Graph G(edges.begin(), edges.end(), num_edges);

    std::vector< Vertex > vertices;

    topological_sort(G, std::back_inserter(vertices));

    for (auto it=vertices.rbegin(); it != vertices.rend(); ++it)
    result.push_back(*it);

    return result;
}
