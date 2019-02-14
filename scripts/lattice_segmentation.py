#!/usr/bin/env python

import argparse
import pymesh
import numpy as np
import copy
import svgwrite
import pickle

def main():

    file_name = "/home/srikanth/Helix/ipython-notebooks/minimap-generation/simplified_lattice.pkl"

    #loads lattice
    new_lattice = pymesh.create_lattice(4.0)

    with open(file_name, 'rb') as f:
        num_edges = pickle.load(f)
        for edge in range(num_edges):
            ver1 = pickle.load(f)
            ver2 = pickle.load(f)
            try:
                new_lattice.AddEdge(ver1,ver2)
            except:
                print( "failed to add edge : " + str(ver1) + " , " +str(ver2) )
                pass
        
    new_lattice.BuildConnections()

    #print(new_lattice)

    #return

    segments  = pymesh.lattice_detect_segments(new_lattice)

    print( "Number of segments : " + str(len(segments)))
    size = 0
    for seg in segments:
        size += len(seg[0])
    

    print( "Number of edges in segments : " + str(size))

    num_edges = new_lattice.GetNumEdges()

    print( "Total number of edges in lattice : " + str(num_edges))

    #print(new_lattice) 

if __name__ == "__main__":
    main();
