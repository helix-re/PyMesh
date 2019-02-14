#include "SegmentDetection.h"

#include <iostream>

using namespace PyMesh;

using namespace LatticeAlgorithms;

namespace PyMesh
{
    /*
    * returns un processed edge index which has edge connections that are not equal to 2
    * -1 of all of the edges are processed 
    */
    std::set<unsigned int> unprocessed_branch_indices( const Lattice2D::Ptr&         lattice,
                                  const std::set<unsigned int>& processed_edge_indices)
    {
        std::set<unsigned int> unprocessed_indices;

        unsigned int num_vertices =  lattice->GetNumVertices();
        for( unsigned int index = 0; index < num_vertices; ++index )
        {
            auto vertex_edge_connections = lattice->GetVertexToEdgeConnections(index);
            if(vertex_edge_connections.size() != 2 )
            {
                for(const auto& edge : vertex_edge_connections )
                {
                    auto it = processed_edge_indices.find( edge );
                    if( it == processed_edge_indices.end() )
                    {
                        unprocessed_indices.insert(edge);
                    }
                }

                if(!unprocessed_indices.empty()) break;

            }
        }

        return unprocessed_indices;
    }

    /*
    * Finds common vertex index
    * Returns common vertex index
    * Returns -1 if there is no common vertex
    */
   bool edges_connected( const Lattice2D::Ptr& lattice,
                      const unsigned int& edge_index1,
                      const unsigned int& edge_index2 )
    {
        auto edge1 = lattice->GetEdge(edge_index1);
        auto edge2 = lattice->GetEdge(edge_index2);
        if( edge1.first == edge2.first )
        {
            return true;
        }
        else if( edge1.second == edge2.first )
        {
            return true;
        }
        else if( edge1.first == edge2.second )
        {
            return true;
        }
        else if( edge1.second == edge2.second )
        {
            return true;
        }
        return false;
    }

    void process_branch( const Lattice2D::Ptr& lattice,
                         std::vector<std::pair<std::vector<unsigned int>,bool>>& branches,  //contains all processed branches
                         std::set<unsigned int>& processed_edge_indices,                    //contains all processed edge indices
                         unsigned int current_edge)                                         //current edge
    {
        // create a new branch
        std::vector<unsigned int> my_branch;

        while(true)
        {
            my_branch.push_back(current_edge);
            processed_edge_indices.insert(current_edge);
            auto connections = lattice->GetEdgeToEdgeConnections(current_edge);

            std::set<unsigned int> unprocessed_connections;

            for( const auto& connection : connections )
            {
                auto it = processed_edge_indices.find(connection);
                if(it == processed_edge_indices.end())
                {
                    unprocessed_connections.insert(connection);
                }
            }

            if( unprocessed_connections.size() == 1 )
            {
                auto new_edge = *(unprocessed_connections.begin());
                
                auto it = processed_edge_indices.find(new_edge);
                if( it == processed_edge_indices.end() )
                {
                    current_edge = new_edge;
                    continue;
                }
            }
            // this branch finished

            // these are new branches
            processed_edge_indices.insert(unprocessed_connections.begin(),unprocessed_connections.end());
            for( auto unprocessed_connection : unprocessed_connections )
            {
                process_branch(lattice,
                               branches,
                               processed_edge_indices,
                               unprocessed_connection);
            }
            if( my_branch.size() > 1 )
            {
                //check if the edges are connected
                auto conncted  = edges_connected( lattice, 
                                                  *(my_branch.begin()),
                                                  *(my_branch.rbegin()) );
                branches.push_back( {my_branch,conncted} );
            }
            else if( !my_branch.empty() )
            {
                branches.push_back( {my_branch,false} );
            }
            break;
        }
    }
}

std::vector<std::pair<std::vector<unsigned int>,bool>> 
SegmentDetection::compute(const Lattice2D::Ptr& lattice)
{
    std::vector<std::pair<std::vector<unsigned int>,bool>>  branches;

    std::set<unsigned int> processed_edge_indices;

    while(true)
    {
        auto indices = unprocessed_branch_indices(lattice,processed_edge_indices);

        if(indices.empty())
        {
            // no more new branches
            break;
        }

        processed_edge_indices.insert(indices.begin(), indices.end());

        for( auto index : indices )
        {
            process_branch(lattice,
                       branches,
                       processed_edge_indices,
                       index
                       );
        }
    }

    return branches;
}
