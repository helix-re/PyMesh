import numpy as np
import os.path

from .Lattice2D import Lattice2D

import PyMesh

def create_empty_lattice2d( precision=0.0001 ):
    factory = PyMesh.Lattice2DFactory()
    return Lattice2D(factory.create(precision))

def create_contour_lattice2d( edgepoints, precision=0.0001 ):
    factory = PyMesh.Lattice2DFactory()
    return Lattice2D(factory.create(edgepoints,True,precision))

def create_lattice2d( edgepoints, precision=0.0001 ):
    factory = PyMesh.Lattice2DFactory()
    return Lattice2D(factory.create(edgepoints,True,precision))

def create_edge_vertices_lattice2d( edges, vertices, precision=0.0001 ):
    factory = PyMesh.Lattice2DFactory()
    return Lattice2D(factory.create(edges,vertices,precision))
