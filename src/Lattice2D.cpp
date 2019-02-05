#include "Lattice2D.h"

using namespace PyMesh;

const double Lattice2D::DOUBLE_PRECISION = 0.001;


Lattice2D::Lattice2D(const Matrix4Fr& edges,
                     const double& precision)
                     :m_presision(precision)
{
    std::size_t num_edges = edges.rows();
    for (std::size_t index=0; index<num_edges; ++index) 
    {
        const auto& edge = edges.row(index);
        Vector2F point1(edge[0],edge[1]);
        Vector2F point2(edge[2],edge[3]);
        AddEdge(point1,point2);
    }
}

Lattice2D::Lattice2D(const Matrix2Fr& contour, const double& precision)
:m_presision(precision)
{
    std::size_t num_vertices = contour.rows();
    if(num_vertices < 2)
    {
        // too few vertices nothing to be done
        return; 
    }
    for(std::size_t index = 0; index < num_vertices-1; ++index)
    {
        Vector2F point1(contour.row(index)[0],contour.row(index)[1]);
        Vector2F point2(contour.row(index+1)[0],contour.row(index+1)[1]);
        AddEdge(point1,point2);
    }
    Vector2F point1(contour.row(num_vertices-1)[0],contour.row(num_vertices-1)[1]);
    Vector2F point2(contour.row(0)[0],contour.row(0)[1]);
    AddEdge(point1,point2);
}

Lattice2D::Lattice2D(const Matrix2Ir& edges,
          const Matrix2Fr vertices,
          const double& precision)
          :m_presision(precision)
{
    //constructor might crash if there is a missmatch in the edges and vertices
    std::size_t num_edges = edges.rows();

    for(std::size_t index=0; index < num_edges; ++index)
    {
        Vector2F point1(vertices.row(edges.row(index)[0])[0],vertices.row(edges.row(index)[0])[1]);
        Vector2F point2(vertices.row(edges.row(index)[1])[0],vertices.row(edges.row(index)[1])[1]);
        AddEdge(point1,point2);
    }
}

Lattice2D::Lattice2D(const double& precision)
:m_presision(precision)
{
}

int Lattice2D::AddEdge(const Vector2F& point1, const Vector2F& point2)
{
    std::set<unsigned int> edge_set;
    edge_set.insert(AddPoint(point1));
    edge_set.insert(AddPoint(point2));

    auto num_edge_points = edge_set.size();
    if(  num_edge_points !=2 )
    {
        //
        // edge might be smaller than the precision
        // two points might have collapsed to the same point
        //
        if( num_edge_points == 1 )
        {
            //here added point should be deleted
            m_vertices.erase(std::prev(m_vertices.end(), 1));
            m_vertex_indices.erase(std::prev(m_vertex_indices.end(), 1));
        }
        //else
        //added two points might have collapsed to the existing point
        //hence nothing to do
        return -1;
    }
    auto it_set = edge_set.begin();
    std::pair<unsigned int,unsigned int> edge_pair{*it_set,*(it_set++)};

    auto it = m_edge_indicies.find(edge_pair);
    // new edge so add else ignore
    if( it == m_edge_indicies.end())
    {
        std::pair<unsigned int,std::pair<unsigned int, unsigned int>> pr{m_edges.size(),edge_pair};
        m_edges.insert(pr);
        m_edge_indicies.insert({pr.second,pr.first});
        return pr.first;
    }
    return it->second;
}

std::pair<Matrix2Ir,Matrix2Fr> Lattice2D::get_lattice()
{
    
    auto num_vertices = m_vertices.size();
    Matrix2Fr vertices(num_vertices,2);
    for(size_t index = 0; index < num_vertices; ++index)
    {
        vertices(index,0) = m_vertices[index].x();
        vertices(index,1) = m_vertices[index].y();
    }

    size_t num_edges = m_edges.size();
    Matrix2Ir edges(num_edges,2);
    for(size_t index = 0; index < num_edges; ++index)
    {
        edges(index,0) = m_edges[index].first;
        edges(index,1) = m_edges[index].second;
    }
    return {edges,vertices};
}

double Lattice2D::Setprecision(double val)
{
    double ret = m_presision * ceil( val / m_presision );
	if ( std::isnan(ret) || !std::isfinite(ret) )
	{
		ret = 0.0;
	}
    return ret;
}

void Lattice2D::Setprecision(Vector2F& point)
{
    point[0] = Setprecision(point[0]);
    point[1] = Setprecision(point[1]);
}

unsigned int Lattice2D::AddPoint(const Vector2F& point)
{
    Vector2F precise_point = point;
    
    Setprecision(precise_point);

    auto it = m_vertex_indices.find(point);
    //new point
    if(it == m_vertex_indices.end())
    {
        //add new point
        std::pair<unsigned int, Vector2F> pr = {m_vertices.size(),precise_point};
        m_vertices.insert(pr);
        m_vertex_indices.insert({pr.second,pr.first});
        return pr.first;
    }
    // point was already present
    return it->second;
}

int Lattice2D::GetPointIndex(const Vector2F& point)
{
    Vector2F precise_point = point;
    Setprecision(precise_point);
    auto it = m_vertex_indices.find(precise_point);
    if(it != m_vertex_indices.end())
    {
        return it->second;
    }
    return -1;
}