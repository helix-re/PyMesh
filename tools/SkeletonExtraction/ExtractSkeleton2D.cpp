#include "ExtractSkeleton2D.h"
#include "Lattice2DFactory.h"
#include "CGAL/Skeleton2D.h"


#include <list>
#include <vector>
#include <sstream>

using namespace PyMesh;


ValidatePolygon2D::result ExtractSkeleton2D::validate_and_fix( Lattice2D::Ptr& lattice )
{
    auto result = ValidatePolygon2D::compute( lattice );

    if( result == ValidatePolygon2D::result::CLOCKWISE )
    {
        lattice->ReverseContour();
        return ValidatePolygon2D::result::SUCCESS;
    }

    return result;
}

void ExtractSkeleton2D::run(const MatrixFr& points, const std::vector<MatrixFr>& holes)
{

    Lattice2D::Ptr outer_lattice = Lattice2DFactory().create(points, true);

    auto result = validate_and_fix(outer_lattice);

    if( result != ValidatePolygon2D::result::SUCCESS )
    {
        std::stringstream err_msg;
        err_msg << "Contour is not valid error code : " << static_cast<unsigned int>(result);
        throw NotImplementedError(err_msg.str());
    }

    std::vector<Lattice2D::Ptr> hole_lattices; 

    for( const auto& hole : holes )
    {
        Lattice2D::Ptr hole_lattice = Lattice2DFactory().create(hole, true);

        if( validate_and_fix(hole_lattice) != ValidatePolygon2D::result::SUCCESS ) continue;


        if( !ValidatePolygon2D::Hole_Inside(outer_lattice,hole_lattice)) continue;

        //holes to should be in reverse direction
        hole_lattice->ReverseContour();

        // all error checks passed
        // add to holes list
        hole_lattices.push_back(hole_lattice);
    }

    if( hole_lattices.empty() )
    {
        m_lattice = Skeleton2D::compute(outer_lattice);
    }
    else
    {
        m_lattice = Skeleton2D::compute(outer_lattice, hole_lattices);
    }
    
}

//#endif