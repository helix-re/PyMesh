#include "Skeleton2D.h"
#include "Lattice2DFactory.h"

#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_straight_skeleton_2.h>
#include<CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point;
typedef CGAL::Polygon_2<K>                                  Polygon_2;
typedef CGAL::Polygon_with_holes_2<K>                       Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K>                        StraightSkeleton;
typedef boost::shared_ptr<StraightSkeleton>                 StraightSkeletonPtr;

using namespace PyMesh;

Lattice2D::Ptr Skeleton2D::compute(const Lattice2D::Ptr& contour_lattice)
{
    Polygon_2 outer;

    std::vector<unsigned int> outer_indicies = contour_lattice->GetContourIndices();

    for(auto& index : outer_indicies)
    {
        auto v =  contour_lattice->GetVertex(index);
        Point point(v[0],v[1]);
        outer.push_back(std::move(point));
    }

    Lattice2D::Ptr skeleton_lattice = Lattice2DFactory().create();

    auto skeleton_creator = CGAL::create_interior_straight_skeleton_2(outer);
    for ( auto it  = skeleton_creator->halfedges_begin(); 
               it != skeleton_creator->halfedges_end(); 
             ++it )
    {
        if( !it->is_bisector() ) continue;
        
        auto& pt1  = it->vertex()->point();
        auto& pt2  = it->opposite()->vertex()->point();

        Vector2F lat_pt1; lat_pt1[0] = pt1.x(); lat_pt1[1] = pt1.y();
        Vector2F lat_pt2; lat_pt2[0] = pt2.x(); lat_pt2[1] = pt2.y();

        skeleton_lattice->AddEdge(lat_pt1, lat_pt2);
    }

    skeleton_lattice->BuildConnections();

    return skeleton_lattice;
}

Lattice2D::Ptr Skeleton2D::compute(const Lattice2D::Ptr& contour_lattice, 
                                   const std::vector<Lattice2D::Ptr>& hole_lattices)
{
    Polygon_2 outer;

    std::vector<unsigned int> outer_indicies = contour_lattice->GetContourIndices();

    for(auto& index : outer_indicies)
    {
        auto v =  contour_lattice->GetVertex(index);
        Point point(v[0],v[1]);
        outer.push_back(std::move(point));
    }

    Polygon_with_holes poly( outer );

    for( const auto& hole_lattice : hole_lattices )
    {
        Polygon_2 hole;

        std::vector<unsigned int> indicies = hole_lattice->GetContourIndices();

        for(auto& index : indicies)
        {
            auto v =  hole_lattice->GetVertex(index);
            Point point(v[0],v[1]);
            hole.push_back(std::move(point));
        }
        poly.add_hole(hole);
    }

    Lattice2D::Ptr skeleton_lattice = Lattice2DFactory().create();

    auto skeleton_creator = CGAL::create_interior_straight_skeleton_2(poly);
    for ( auto it  = skeleton_creator->halfedges_begin(); 
               it != skeleton_creator->halfedges_end(); 
             ++it )
    {
        if( !it->is_bisector() ) continue;
        
        auto& pt1  = it->vertex()->point();
        auto& pt2  = it->opposite()->vertex()->point();

        Vector2F lat_pt1; lat_pt1[0] = pt1.x(); lat_pt1[1] = pt1.y();
        Vector2F lat_pt2; lat_pt2[0] = pt2.x(); lat_pt2[1] = pt2.y();

        skeleton_lattice->AddEdge(lat_pt1, lat_pt2);
    }

    skeleton_lattice->BuildConnections();

    return skeleton_lattice;
}