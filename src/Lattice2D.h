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

public:
const static double DOUBLE_PRECISION;

public:
/*
* creates lattice structure with edges given as points
*/
Lattice2D(const Matrix4Fr& edges, const double& precision = DOUBLE_PRECISION);
/*
* creates lattice structure where input is contour represented as points
*/
Lattice2D(const Matrix2Fr& contour, const double& precision = DOUBLE_PRECISION);
/*
* creates lattice with 
* points and edges given as point indices
*/
Lattice2D(const Matrix2Ir& edges,
          const Matrix2Fr vertices, 
          const double& precision = DOUBLE_PRECISION);
/*
* default constructor
* needed to support AddEdge function
*/
Lattice2D(const double& precision = DOUBLE_PRECISION);
/*
* adds edge to lattice
* returns -1 if it fails to add
*/
int AddEdge(const Vector2F& point1, const Vector2F& point2);
/*
* returns basic representation of contour
*/
std::pair< Matrix2Ir,Matrix2Fr > get_lattice();
/*
* returns point index
* if point not found it returns -1
*/
int GetPointIndex(const Vector2F& point);

protected:
/*
* adds and returns index of added point
*/
unsigned int AddPoint(const Vector2F& point);
/*
* applies precision
*/
void Setprecision(Vector2F& point);
/*
* applies precision
*/ 
double Setprecision(double val);

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
*/
std::map<unsigned int,std::set<unsigned int>> m_vertex_vertex_connections;
/*
* vertex to edge connections
*/
std::map<unsigned int,std::set<unsigned int>> m_vertex_edge_connections;
/*
* current precision
*/
const double m_presision;
};

}