import os.path
import sys
import numpy as np

import PyMesh


def create_lattice():
    factory = PyMesh.Lattice2DFactory()
    return factory.create( 0.001 )

