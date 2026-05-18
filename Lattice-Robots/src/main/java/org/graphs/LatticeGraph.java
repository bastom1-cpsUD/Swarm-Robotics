package org.graphs;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.stream.Collectors;
/**
 * A strongly connected directed multigraph in which each edge e is labeled with a ridgid 
 * body transformation T(e) and v -> w has an inverse edge w -> v with transformation T(e)^-1.*/
public abstract class LatticeGraph {
    protected HashMap<Vertex, ArrayList<LatticeEdge>> edges;

    public LatticeGraph() {
        edges = new HashMap<>();
    }

    public ArrayList<Vertex> getVertices() {
        return edges.keySet().parallelStream().collect(Collectors.toCollection(ArrayList::new));
    }

    public ArrayList<LatticeEdge> getOutgoingEdges(Vertex vertex) {
        return edges.getOrDefault(vertex, new ArrayList<>());
    }

    public Vertex getPrimaryVertex() {
        return (Vertex) edges.keySet().toArray()[0]; // Assuming the first vertex is the primary vertex
    }
}

