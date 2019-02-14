#include "ValidatePolygon2D.h"

#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point;
typedef CGAL::Polygon_2<K>                                  Polygon_2;

using namespace PyMesh;

ValidatePolygon2D::result ValidatePolygon2D::compute(const Lattice2D::Ptr& lattice)
{
    Polygon_2 poly;

    std::vector<unsigned int> indicies = lattice->GetContourIndices();

    if( indicies.empty() ) return ValidatePolygon2D::result::EMPTY_LATTICE;

    for(auto& index : indicies)
    {
        auto v =  lattice->GetVertex(index);
        Point point(v[0],v[1]);
        poly.push_back(std::move(point));
    }

    if( poly.is_empty() ) return ValidatePolygon2D::result::EMPTY_LATTICE;

    if( !poly.is_simple() ) return ValidatePolygon2D::result::NOT_SIMPLE;

    if( poly.is_collinear_oriented() ) return ValidatePolygon2D::result::COLLINEAR;

    //check non manifold scenario for the contour
    for(auto& index : indicies)
    {
        if( lattice->GetVertexToVertexConnections(index).size() != 2 )
        {
            return ValidatePolygon2D::result::NON_MANIFOLD;
        }
        if( lattice->GetVertexToEdgeConnections(index).size()   != 2 )
        {
            return ValidatePolygon2D::result::NON_MANIFOLD;
        }
    }

    if( poly.is_clockwise_oriented() ) return ValidatePolygon2D::result::CLOCKWISE;

    return ValidatePolygon2D::result::SUCCESS;
}

bool ValidatePolygon2D::Hole_Inside( const Lattice2D::Ptr& lattice, const Lattice2D::Ptr& hole)
{
    Polygon_2 poly;

    std::vector<unsigned int> indicies = lattice->GetContourIndices();

    if( indicies.empty() ) return false;

    for(auto& index : indicies)
    {
        auto v =  lattice->GetVertex(index);
        Point point(v[0],v[1]);
        poly.push_back(std::move(point));
    }

    std::vector<unsigned int> hole_indicies = hole->GetContourIndices();

    for(auto& index : hole_indicies)
    {
        auto v =  hole->GetVertex(index);
        Point point(v[0],v[1]);
        
        if( !poly.has_on_bounded_side(point) )
        {
            return false;
        }
    }

    return true;
}