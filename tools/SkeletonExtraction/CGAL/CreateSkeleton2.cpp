#include "CreateSkeleton2.h"

#include <list>
#include <vector>
#include <sstream>
#include<boost/shared_ptr.hpp>

#ifdef WITH_CGAL

#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/iterator.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Filtered_predicate.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Cartesian_converter.h>
#include <CGAL/convex_hull_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                   Point;
typedef CGAL::Polygon_2<K>           Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K> Ss;
typedef boost::shared_ptr<Ss> SsPtr;

using namespace PyMesh;

namespace convex_hull
{
    std::vector<Point> compute_hull( const MatrixFr& points){

        std::size_t num_vertices = points.rows();
        //First find the normal of the polygon to fix the order
        if(num_vertices < 3){
            std::stringstream err_msg;
            err_msg << "Few vertices to process num_vertices=" << num_vertices;
            throw RuntimeError(err_msg.str());
        }

        // compute convex hull
        std::vector<Point> vz_points;
        for (size_t i=0; i<num_vertices; i++) {
                const VectorF& v = points.row(i);
                Point point(v[0],v[1]);
                vz_points.push_back( std::move(point) );
        }

        std::vector<Point> result;
        CGAL::convex_hull_2(vz_points.begin(), vz_points.end(), std::back_inserter(result));
        return result;
    }
}

namespace find
{
    typedef CGAL::Simple_cartesian<double> K;
    typedef CGAL::Simple_cartesian<CGAL::Interval_nt_advanced> FK;
    typedef CGAL::Simple_cartesian<CGAL::MP_Float> EK;
    typedef CGAL::Cartesian_converter<K, EK> C2E;
    typedef CGAL::Cartesian_converter<K, FK> C2F;
    // Define my predicate, parameterized by a kernel.
    template < typename K >
    struct My_orientation_2
    {
    typedef typename K::RT            RT;
    typedef typename K::Point_2       Point_2;
    typedef typename K::Orientation   result_type;
    result_type
    operator()(const Point_2 &p, const Point_2 &q, const Point_2 &r) const
    {
        RT prx = p.x() - r.x();
        RT pry = p.y() - r.y();
        RT qrx = q.x() - r.x();
        RT qry = q.y() - r.y();
        return CGAL::sign( prx*qry - qrx*pry );
    }
    };
    typedef CGAL::Filtered_predicate<My_orientation_2<EK>,
                                    My_orientation_2<FK>, C2E, C2F> Orientation_2;

    bool direction(const MatrixFr& points)
    {
        std::vector<Point> result = convex_hull::compute_hull(points);
        float p1x = result[0].x();
        float p1y = result[0].y();
        float p2x = result[1].x();
        float p2y = result[1].y();
        float p3x = result[2].x();
        float p4y = result[2].y();

        K::Point_2 p(p1x,p1y), q(p2x,p2y), r(p3x,p4y);
        Orientation_2 orientation;
        return orientation(p, q, r);
    }
}

namespace PyMesh
{
    template<class K>
    void print_point ( CGAL::Point_2<K> const& p )
    {
        std::cout << "(" << p.x() << "," << p.y() << ")" << std::endl;
    }

    bool validate_vertices(Matrix2Fr& computed_vertices)
    {
        std::size_t num_vertices = computed_vertices.rows();

        for (size_t i=0; i<num_vertices; i++) {
            const VectorF& v = computed_vertices.row(i);
            if( v[0] == 0 || v[1] == 0 )
            {
                return false;
            }
        }
        return true;
    }

    void compute_skeleton(const MatrixFr& points,
                          const std::vector<MatrixFr>& holes,
                          const bool direction,
                          Matrix2Fr& computed_vertices, 
                          Matrix2Ir& computed_edges )
    {

        std::size_t num_vertices = points.rows();

        Polygon_2 outer;
        if( !direction ){
            for (size_t i=0; i<num_vertices; i++) {
                const VectorF& v = points.row(i);
                Point point(v[0],v[1]);
                outer.push_back( std::move(point) );
            }
        }
        else
        {
            for (int i=num_vertices-1; i>=0; i--) {
                const VectorF& v = points.row(i);
                Point point(v[0],v[1]);
                outer.push_back( std::move(point) );
            }
        }
        
        Polygon_with_holes poly( outer );

        for(const auto& ei_hole : holes){

            bool hole_direction = true;

            // logic to fix the direction
            {
                std::vector<MatrixFr> tmp_holes;
                Matrix2Fr computed_vertices;
                Matrix2Ir computed_edges;
                compute_skeleton(ei_hole,
                        tmp_holes,
                        hole_direction,
                        computed_vertices,
                        computed_edges);
                /*
                * TODO: THIS IS A HACK
                * FIX IT PROPERLY BY CREATING CONVEX POLYGON DATA STRUCTURE WITH CONNECTIVITY INFORMATION
                */
                if( !validate_vertices(computed_vertices))
                {
                    hole_direction = false;
                }
            }
            
            std::size_t hole_vertices = ei_hole.rows();
            Polygon_2 hole;
            if( !hole_direction ){
                for (size_t i=0; i<hole_vertices; i++) {
                    const VectorF& v = ei_hole.row(i);
                    Point point(v[0],v[1]);
                    hole.push_back( std::move(point) );
                }
            }
            else
            {
                for (int i=hole_vertices-1; i>=0; i--) {
                    const VectorF& v = ei_hole.row(i);
                    Point point(v[0],v[1]);
                    hole.push_back( std::move(point) );
                }
            }
            poly.add_hole( hole );
        }

        std::vector<Point>                          skeleton_points;   //skeliton points
        std::set<std::set<std::size_t>>             skeleton_edges;    //skeleton edges represented as point indices

        //compute skeleton
        SsPtr ss = CGAL::create_interior_straight_skeleton_2(poly);
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
                //print_point(i->vertex()->point());
                //print_point(i->opposite()->vertex()->point());
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
        computed_vertices = Matrix2Fr(num_vertices,2);
        for(size_t index = 0; index < num_vertices; ++index)
        {
            computed_vertices(index,0) = skeleton_points[index].x();
            computed_vertices(index,1) = skeleton_points[index].y();
        }

        size_t num_skeleton_edges = skeleton_edges.size();
        computed_edges = Matrix2Ir(num_skeleton_edges,2); //this is the format that has to be returned
        size_t index = 0;
        for( const auto& edge : skeleton_edges ){
            auto it = edge.begin();
            computed_edges(index,0) = re_index[*it]; it++;
            computed_edges(index,1) = re_index[*it];
            index++;
        }
    }
}

void CreateSkeleton2::run(const MatrixFr& points, const std::vector<MatrixFr>& holes)
{
    bool direction(true);
    {
        // logic to fix the direction
        {
            std::vector<MatrixFr> tmp_holes;
            Matrix2Fr tmp_vertices;
            Matrix2Ir tmp_edges;
            compute_skeleton(points,
                    tmp_holes,
                    direction,
                    tmp_vertices,
                    tmp_edges);
            /*
            * TODO: THIS IS A HACK
            * FIX IT PROPERLY BY CREATING CONVEX POLYGON DATA STRUCTURE WITH CONNECTIVITY INFORMATION
            */
            if(!validate_vertices(tmp_vertices))
            {
                direction = false;
            }
            else if( holes.empty() )
            {
                m_vertices = std::move(tmp_vertices);
                m_edges = std::move(tmp_edges);
                return;
            }
        }
    }

    Matrix2Fr computed_vertices;
    Matrix2Ir computed_edges;

    compute_skeleton(points,
                     holes,
                    direction,
                    computed_vertices,
                    computed_edges);

    /*
    * TODO: THIS IS A HACK
    * FIX IT PROPERLY BY CREATING CONVEX POLYGON DATA STRUCTURE WITH CONNECTIVITY INFORMATION
    */
    if( !validate_vertices(computed_vertices))
    {
        compute_skeleton(points,holes,!direction,computed_vertices,computed_edges);
    }

    m_vertices = std::move(computed_vertices);
    m_edges = std::move(computed_edges);
}

#endif