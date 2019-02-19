import os.path
import sys
import numpy as np

import PyMesh


def create_lattice( precision = 0.001 ):
    factory = PyMesh.Lattice2DFactory()
    return factory.create( precision )