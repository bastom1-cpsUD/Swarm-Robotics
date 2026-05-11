package org.graphs;

import java.util.ArrayList;

/**
 * A strongly connected directed multigraph in which each edge e is labeled with a ridgid 
 * body transformation T(e) and v -> w has an inverse edge w -> v with transformation T(e)^-1.*/
public abstract class LatticeGraph {
    protected ArrayList<OrientedPoint> vertices;
    protected ArrayList<LatticeEdge> edges;

    public LatticeGraph() {
        vertices = new ArrayList<>();
        edges = new ArrayList<>();
    }
}

