#!/usr/bin/env python

"""
Compute the convex hull of a given mesh.
"""

import argparse
import pymesh
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(__doc__);
    parser.add_argument("--engine", default="cgal",
            choices=["cgal"]);
    parser.add_argument("--with-timing", help="output timing info",
            action="store_true");
    parser.add_argument("input_file");
    parser.add_argument("output_file");
    return parser.parse_args();

def main():
    """
    TODO: 
    1 . implement svg or other filebased import
    2 . implement exporting skeleton as svg  file
    args = parse_args();
    """

    # contor = np.array( [[100.0,100.0],
    #                     [300.0,100.0],
    #                     [300.0,300.0],
    #                     [100.0,300.0]] );

    # contor = np.array( [[-1,-1],
    #                     [0,-12],
    #                     [1,-1],
    #                     [12,0],
    #                     [1,1],
    #                     [0,12],
    #                     [-1,1],
    #                     [-12,0]] );

    contor = np.array( [[100.0, 100.0],
                        [100.0, 300.0],
                        [300.0, 300.0],
                        [300.0, 100.0]] );

    #reversed_arr = contor[::-1]

    # self intersection contour
    # contor = np.array( [[  100,   100],
    #                     [  200,   100],
    #                     [  200,   200],
    #                     [  150,   50]] );

    print(contor)

    edges, vertices, running_time = pymesh.skeleton(contor, "cgal", True);

    print("Running time: {}s".format(running_time));
    print("vertices : {}".format(vertices));
    print("edges : {}".format(edges));

if __name__ == "__main__":
    main();