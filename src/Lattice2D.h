/* This file is part of PyMesh. Copyright (c) 2018 by Qingnan Zhou */
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>

#include <Core/EigenTypedef.h>


namespace PyMesh {

class Lattice2D {

// Use Lattice2DFactory class to create Lattice2D object.
friend class Lattice2DFactory;

public:
const static double DOUBLE_PRECISION;
typedef std::shared_ptr<Lattice2D> Ptr;

protected:

Lattice2D(Lattice2D& other) = delete;
Lattice2D& operator=(Lattice2D& other) = delete;

/*
* creates lattice structure with edges given as points
* or closed contour given as points
*/
Lattice2D(const MatrixFr& mat, const bool& contour, const double& precision = DOUBLE_PRECISION);

/*
* creates lattice with 
* points and edges given as point indices
*/
Lattice2D(const MatrixIr& edges,
          const MatrixFr vertices, 
          const double& precision = DOUBLE_PRECISION);

/*
* default constructor
* needed to support AddEdge function
*/
Lattice2D(const double& precision = DOUBLE_PRECISION);

public: 
/*
* adds edge to lattice
* @return edge index, pair of vertex indices
* returns -1 if it fails to add
*/
std::pair<int,std::pair<unsigned int, unsigned int>>
AddEdge(const VectorF& point1, const VectorF& point2);

/*
* builds connections
* NOTE: this function is needed only if AddEdge functions is used to populate lattice
*/
void BuildConnections();

/*
* returns basic representation of contour
*/
std::pair< MatrixIr,MatrixFr > GetLattice() const;
/*
* returns vertex index
* throws exception if vertex is not found
*/
unsigned int GetVertexIndex(const VectorF& point) const;
/*
* returns point
* throws exception if vertex is not found
*/
VectorF GetVertex(unsigned int index) const;
/*
* returns edge
* throws exception if edge is not found
*/
std::pair<unsigned int, unsigned int> GetEdge(unsigned int index) const;
/*
* returns vertex connections of a given vertex index
* set of vertex indices
*/
std::set<unsigned int> GetVertexConnections(unsigned int vertex_index) const;
/*
* returns edge connections of a given vertex index
* set of edge indices
*/
std::set<unsigned int> GetEdgeConnections(unsigned int vertex_index) const;
/*
* returns number of vertices
*/
unsigned int GetNumVertices() const;
/*
* returns number of vertices
*/
unsigned int GetNumEdges() const;
/*
* returns contour indices
*/
std::vector<unsigned int> GetContourIndices() const;
/*
* reverses contour
*/
void ReverseContour();
/*
* converts lattice to string
*/
std::string ToString() const;
protected:
/*
* populates edges
*/
void PopulateEdges(const Matrix4Fr& edges);
/*
* populates contour
*/
void PopulateContour(const Matrix2Fr& contour);

/*
* adds and returns index of added point
*/
std::pair<unsigned int,bool> AddPoint(const Vector2F& point);
/*
* applies precision
*/
void Setprecision(Vector2F& point) const;
/*
* applies precision
*/ 
double Setprecision(double val) const;
/*
* builds vertex connections
*/
void BuildVertexConnections();
/*
* builds edge connections
*/
void BuildEdgeConnections();

protected:
/*
* vertices map
*/
std::map<unsigned int,Vector2F> m_vertices;
/*
* vertex indices map
*/
std::map<Vector2F, 
         unsigned int,
         std::function<bool(const Vector2F&, const Vector2F&)>> m_vertex_indices{
             [](const Vector2F& point1, const Vector2F& point2) { 
                if (point1[0] < point2[0])
                    return true;
                else if (point1[0] == point2[0] && point1[1] < point2[1])
                    return true;
                else
                    return false;
            }
         };
/*
*Edges map
* key is the id of edge
* value is the pair of vertx indices that an edge is made of
*/
std::map<unsigned int,std::pair<unsigned int, unsigned int>> m_edges;
/*
* edge indices
*/
std::map<std::pair<unsigned int, unsigned int>,unsigned int> m_edge_indicies;
/*
* vertex to vertex connetions
* set is connected vertex indicies
*/
std::map<unsigned int,std::set<unsigned int>> m_vertex_vertex_connections;
/*
* vertex to edge connections
* set is connected edge indicies
*/
std::map<unsigned int,std::set<unsigned int>> m_vertex_edge_connections;
/*
* current precision
*/
const double m_precision;
/*
* contour indices
* this is populated if the input is given as contour
*/
std::vector<unsigned int> m_contour_indicies;
};

}