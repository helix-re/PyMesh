#include "Lattice2D.h"
#include <Core/Exception.h>

using namespace PyMesh;

const double Lattice2D::DOUBLE_PRECISION = 0.001;

Lattice2D::Lattice2D(const MatrixFr& mat,
                     const bool& contour,
                     const double& precision)
                     :m_precision(precision)
{
    if(contour)
    {
        PopulateContour(mat);
    }
    else
    {
        PopulateEdges(mat);
    }
    BuildConnections();
}

void Lattice2D::PopulateEdges(const Matrix4Fr& edges)
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

void Lattice2D::PopulateContour(const Matrix2Fr& contour)
{
    std::size_t num_vertices = contour.rows();
    if(num_vertices < 2)
    {
        // too few vertices, nothing to be done
        return; 
    }
    for(std::size_t index = 0; index<num_vertices-1; ++index)
    {
        Vector2F point1(contour.row(index)[0],contour.row(index)[1]);
        Vector2F point2(contour.row(index+1)[0],contour.row(index+1)[1]);
        AddEdge(point1,point2);
    }
    Vector2F point1(contour.row(num_vertices-1)[0],contour.row(num_vertices-1)[1]);
    Vector2F point2(contour.row(0)[0],contour.row(0)[1]);
    AddEdge(point1,point2);
}

Lattice2D::Lattice2D(const MatrixIr& edges,
          const MatrixFr vertices,
          const double& precision)
          :m_precision(precision)
{
    //constructor might crash if there is a missmatch in the edges and vertices
    std::size_t num_edges = edges.rows();

    for(std::size_t index=0; index < num_edges; ++index)
    {
        Vector2F point1(vertices.row(edges.row(index)[0])[0],vertices.row(edges.row(index)[0])[1]);
        Vector2F point2(vertices.row(edges.row(index)[1])[0],vertices.row(edges.row(index)[1])[1]);
        AddEdge(point1,point2);
    }
    BuildConnections();
}

Lattice2D::Lattice2D(const double& precision)
:m_precision(precision)
{
}

int Lattice2D::AddEdge(const VectorF& point1, const VectorF& point2)
{
    std::map<unsigned int,bool> edge_map;
    edge_map.insert(AddPoint(point1));
    edge_map.insert(AddPoint(point2));

    auto num_edge_points = edge_map.size();
    if(  num_edge_points !=2 )
    {
        //
        // edge might be smaller than the precision
        // two points might have collapsed to the same point
        //
        if( num_edge_points == 1 && edge_map.begin()->second )
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

    auto it_map = edge_map.begin();
    std::pair<unsigned int,unsigned int> edge_pair{it_map->first,(++it_map)->first};

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

std::pair<MatrixIr,MatrixFr> Lattice2D::GetLattice()
{
    auto num_vertices = m_vertices.size();
    MatrixFr vertices(num_vertices,2);
    for(size_t index = 0; index < num_vertices; ++index)
    {
        vertices(index,0) = m_vertices[index].x();
        vertices(index,1) = m_vertices[index].y();
    }

    size_t num_edges = m_edges.size();
    MatrixIr edges(num_edges,2);
    for(size_t index = 0; index < num_edges; ++index)
    {
        edges(index,0) = m_edges[index].first;
        edges(index,1) = m_edges[index].second;
    }
    return {edges,vertices};
}

double Lattice2D::Setprecision(double val)
{
    double ret = m_precision * ceil( val / m_precision );
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

std::pair<unsigned int,bool> Lattice2D::AddPoint(const Vector2F& point)
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
        return {pr.first,true};
    }
    // point was already present
    return {it->second,false};
}

void Lattice2D::BuildVertexConnections()
{
    m_vertex_vertex_connections = std::map<unsigned int,std::set<unsigned int>>();

    for( const auto& edge : m_edges )
    {
        const auto& vertices_pr = edge.second;
        auto it1 = m_vertex_vertex_connections.find( vertices_pr.first );
        if( it1 == m_vertex_vertex_connections.end() )
        {
            std::set<unsigned int> connections;
            connections.insert(vertices_pr.second);
            m_vertex_vertex_connections[vertices_pr.first] = connections;
        }
        else
        {
            it1->second.insert(vertices_pr.second);
        }
        auto it2 = m_vertex_vertex_connections.find( vertices_pr.second );
        if( it2 == m_vertex_vertex_connections.end() )
        {
            std::set<unsigned int> connections;
            connections.insert(vertices_pr.first);
            m_vertex_vertex_connections[vertices_pr.second] = connections;
        }
        else
        {
            it2->second.insert(vertices_pr.first);
        }
    }
}

void Lattice2D::BuildEdgeConnections()
{
    m_vertex_edge_connections = std::map<unsigned int,std::set<unsigned int>>();

    for( const auto& edge : m_edges )
    {
        const auto& edge_index = edge.first;
        const auto& vertices_pr = edge.second;
        auto it1 = m_vertex_edge_connections.find( vertices_pr.first );
        if( it1 == m_vertex_edge_connections.end() )
        {
            std::set<unsigned int> connections;
            connections.insert(edge_index);
            m_vertex_edge_connections[vertices_pr.first] = connections;
        }
        else
        {
            m_vertex_edge_connections[vertices_pr.first].insert(edge_index);
        }

        auto it2 = m_vertex_edge_connections.find( vertices_pr.second );
        if( it2 == m_vertex_edge_connections.end() )
        {
            std::set<unsigned int> connections;
            connections.insert(edge_index);
            m_vertex_edge_connections[vertices_pr.second] = connections;
        }
        else
        {
            m_vertex_edge_connections[vertices_pr.second].insert(edge_index);
        }
    }
}

void Lattice2D::BuildConnections()
{
    BuildVertexConnections();
    BuildEdgeConnections();
}

unsigned int Lattice2D::GetVertexIndex(const VectorF& point)
{
    Vector2F precise_point = point;
    Setprecision(precise_point);
    auto it = m_vertex_indices.find(precise_point);
    if(it != m_vertex_indices.end())
    {
        return it->second;
    }
    std::stringstream err_msg;
    err_msg << "Vertex not found";
    throw RuntimeError(err_msg.str());
}

VectorF Lattice2D::GetVertex(unsigned int index)
{
    auto it = m_vertices.find(index);
    if(it != m_vertices.end())
    {
        return it->second;
    }

    std::stringstream err_msg;
    err_msg << "Vertex not found, vertex_index=" << index << " number of vertices : " << m_vertices.size();
    throw RuntimeError(err_msg.str());
}

std::pair<unsigned int, unsigned int> Lattice2D::GetEdge(unsigned int index)
{
    auto it = m_edges.find(index);

    if(it != m_edges.end())
    {
        return it->second;
    }

    std::stringstream err_msg;
    err_msg << "Edge not found, edge_index=" << index << " number of edges : " << m_edges.size();
    throw RuntimeError(err_msg.str());
}

std::set<unsigned int> Lattice2D::GetVertexConnections(unsigned int vertex_index)
{
    auto it = m_vertex_vertex_connections.find(vertex_index);

    if(it != m_vertex_vertex_connections.end())
    {
        return it->second;
    }

    std::stringstream err_msg;
    err_msg << "vertex connections not found, vertex_index=" << vertex_index 
            << " number of vertex to set of vertices connection : " << m_vertex_vertex_connections.size();
    throw RuntimeError(err_msg.str());
}

std::set<unsigned int> Lattice2D::GetEdgeConnections(unsigned int vertex_index)
{
    auto it = m_vertex_edge_connections.find(vertex_index);

    if(it != m_vertex_edge_connections.end())
    {
        return it->second;
    }

    std::stringstream err_msg;
    err_msg << "edge connections not found, vertex_index=" << vertex_index 
            << " number of vertex to set of edge connection : " << m_vertex_edge_connections.size();
    throw RuntimeError(err_msg.str());
}

unsigned int Lattice2D::GetNumVertices()
{
    return m_vertices.size();
}

unsigned int Lattice2D::GetNumEdges()
{
    return m_edges.size();
}