#include "CreateSkeleton2.h"

#include <list>
#include <vector>

#include<boost/shared_ptr.hpp>

#ifdef WITH_CGAL

#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_straight_skeleton_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;
typedef K::Point_2                   Point ;
typedef CGAL::Polygon_2<K>           Polygon_2;
typedef CGAL::Straight_skeleton_2<K> Ss ;
typedef boost::shared_ptr<Ss> SsPtr ;

using namespace PyMesh;

void CreateSkeleton2::run(const MatrixFr& points)
{
    Polygon_2 poly;
    std::size_t num_vertices = points.rows();

    for (size_t i=0; i<num_vertices; i++) {
        const VectorF& v = points.row(i);
        poly.push_back( Point(v[0],v[1]) );
    }

    std::vector<Point>                          skeleton_points;   //skeliton points
    std::set<std::set<std::size_t>>             skeleton_edges;    //skeleton edges represented as point indices 

    //compute skeleton
    SsPtr ss = CGAL::create_interior_straight_skeleton_2(poly.vertices_begin(), poly.vertices_end());

    std::map<size_t,Point> skeleton_map_points;//tmp object
    typedef typename Ss::Halfedge_const_iterator Halfedge_const_iterator;
    for ( Halfedge_const_iterator i = ss->halfedges_begin(); i != ss->halfedges_end(); ++i )
    {
        if( i->is_bisector() )
        {
            std::set<std::size_t> edge;
            edge.insert(i->opposite()->vertex()->id());
            edge.insert(i->vertex()->id());
            skeleton_edges.insert(edge);
            skeleton_map_points[i->vertex()->id()]             = i->vertex()->point();
            skeleton_map_points[i->opposite()->vertex()->id()] = i->opposite()->vertex()->point();
        }
    }

    // perform re indiexing and populate vertices
    std::map<size_t,size_t> re_index;
    for( const auto& skeleton_map_point : skeleton_map_points )
    {
        re_index[skeleton_map_point.first] = re_index.size()-1;
        skeleton_points.push_back(skeleton_map_point.second);
    }

    num_vertices = skeleton_points.size();
    Matrix2Fr computed_vertices(num_vertices,2);
    for(size_t index = 0; index < num_vertices; ++index)
    {
        computed_vertices(index,0) = skeleton_points[index].x();
        computed_vertices(index,1) = skeleton_points[index].y();
    }

    size_t num_skeleton_edges = skeleton_edges.size();
    Matrix2Ir computed_edges(num_skeleton_edges,2); //this is the format that has to be returned
    size_t index = 0;
    for( const auto& edge : skeleton_edges ){
        auto it = edge.begin();
        computed_edges(index,0) = re_index[*it]; it++;
        computed_edges(index,1) = re_index[*it];
        index++;
    }

    m_vertices = std::move(computed_vertices);
    m_edges = std::move(computed_edges);
}

#endif