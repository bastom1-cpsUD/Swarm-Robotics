package org.graphs;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * A strongly connected directed multigraph in which each edge e is labeled with a ridgid 
 * body transformation T(e) and v -> w has an inverse edge w -> v with transformation T(e)^-1.*/
public abstract class LatticeGraph {
    protected ArrayList<OrientedPoint> vertices;
    protected HashMap<OrientedPoint, List<LatticeEdge>> edges;

    public LatticeGraph() {
        vertices = new ArrayList<>();
        edges = new HashMap<>();
    }

    public ArrayList<OrientedPoint> getVertices() {
        return vertices;
    }

    public List<LatticeEdge> getOutgoingEdges(OrientedPoint vertex) {
        return edges.getOrDefault(vertex, new ArrayList<>());
    }

    public OrientedPoint getPrimaryVertex() {
        return vertices.get(0); // Assuming the first vertex is the primary vertex
    }
}

