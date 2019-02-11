import PyMesh
from time import time
import numpy as np

def skeleton(contour, holes, engine="auto", with_timing=False):
    """
    Compute the skeleton of an input contour.

    Args:
        contour: The input contour, numpy array of N x 2, where N is the number of points
        holes  : holes that are represented as  list of numpy array of N x 2, where N is the number of points
        engine : (``string``): (optional) Convex hull engine name.  Valid names are:
                * `auto`: Using the default engine.
                * `cgal`: CGAL convex hull module (
                    `2D <https://doc.cgal.org/latest/Straight_skeleton_2>`_,
                    `3D <NOT supportd at>`_)
                * other engines are not supported at
        with_timing (``boolean``): (optional) Whether to time the code

    Returns: The output skeleton, vertices numpy array of N x 2, where N is the number of vertices
                                  edges    numpy array of N x 2, where N is the number of edges(indices of vertices)
    (and running time if `with_timing` is true.)
    """

    if engine == "auto":
        engine = "cgal";

    engine = PyMesh.SkeletonEngine.create(2, engine);

    if with_timing:
        start_time = time();

    engine.run(contour,holes);

    if with_timing:
        finish_time = time();
        running_time = finish_time - start_time;

    lattice = engine.get_lattice();
    if with_timing:
        return lattice, running_time;
    else:
        return lattice;
    