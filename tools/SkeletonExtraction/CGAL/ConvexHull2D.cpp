#include "ConvexHull2D.h"
#include "Lattice2DFactory.h"
#include <Core/Exception.h>
#include <list>
#include <vector>
#include <sstream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>

using namespace PyMesh;
using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;

Lattice2D::Ptr ConvexHull2D::compute(const Lattice2D::Ptr& input)
{
    auto num_vertices = input->GetNumVertices();

    //First find the normal of the polygon to fix the order
    if(num_vertices < 3){
        std::stringstream err_msg;
        err_msg << "Few vertices to process num_vertices=" << num_vertices;
        throw RuntimeError(err_msg.str());
    }

    // compute convex hull
    std::vector<Point_2> vz_points;
    for (size_t index=0; index<num_vertices; ++index) {
            VectorF v = input->GetVertex(index);
            Point_2 point(v[0],v[1]);
            vz_points.push_back( std::move(point));
    }

    std::vector<Point_2> hull;
    CGAL::convex_hull_2(vz_points.begin(), vz_points.end(), std::back_inserter(hull));

    // conver the result to lattice object
    auto num_hull_vertices = hull.size();
    if( num_hull_vertices < 3 ){
        std::stringstream err_msg;
        err_msg << "Failed to compute hull num of hull vertices=" << num_hull_vertices;
        throw RuntimeError(err_msg.str());
    }

    Lattice2D::Ptr hull_lattice = Lattice2DFactory().create();

    for( size_t index = 0; index < num_hull_vertices-1; ++index ){
         Vector2F v1; v1[0] = hull[index].x();   v1[1] = hull[index].y();
         Vector2F v2; v2[0] = hull[index+1].x(); v2[1] = hull[index+1].y();

        hull_lattice->AddEdge( v1, v2 );
    }
    hull_lattice->BuildConnections();

    return hull_lattice;
}