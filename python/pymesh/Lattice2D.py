import os.path
import sys
import numpy as np

import PyMesh

class Lattice2D(object):
    """ A generic representation of a 2d lattice

        num_vertices: Number of vertices (:math:`N_v`).
        num_edges:    Number of edges (:math:`N_e`).
    """

    def __init__(self, raw_lattice):
        """ Private constructor

        Please see :py:mod:`create_lattice2d` for methods of creating lattice.
        """
        self.__lattice = raw_lattice;

    def __str__(self):
         return self.__lattice.ToString();

    def __repr__(self):
         return self.__lattice.ToString();

    @property
    def num_vertices(self):
        return self.__lattice.GetNumVertices();

    @property
    def num_edges(self):
        return self.__lattice.GetNumEdges();

    def add_edge(self, point1, point2):
        self.__lattice.AddEdge(point1,point2);

    def build_connections(self):
        self.__lattice.BuildConnections();

    def get_lattice(self):
        return self.__lattice.GetLattice();

    def get_vertex_index(self, point):
        return self.__lattice.GetVertexIndex(point);

    def get_vertex(self, index):
        return self.__lattice.GetVertex(index);
    
    def get_edge(self, index):
        return self.__lattice.GetEdge(index);
    
    def get_vertex_connections(self, vertex_index):
        return self.__lattice.GetVertexConnections(vertex_index);

    def get_edge_connections(self, vertex_index):
        return self.__lattice.GetEdgeConnections(vertex_index);

    def get_contour_indices(self):
        return self.__lattice.GetContourIndices();

    def reverse_contour(self):
        return self.__lattice.ReverseContour();
