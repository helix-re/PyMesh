#!/usr/bin/env python

"""
Compute the convex hull of a given mesh.
"""

import argparse
import pymesh
import numpy as np

def main():

    contor = np.array( [[100.0, 100.0],
                        [100.0, 300.0],
                        [300.0, 300.0],
                        [300.0, 100.0]] );
    
    print(contor)

    lattice = pymesh.create_contour_lattice2d(contor)

    print("num vertices : {}".format(lattice.num_vertices));
    print("num edges : {}".format(lattice.num_edges));

    ret = lattice.get_lattice()

    print(ret)

if __name__ == "__main__":
    main();