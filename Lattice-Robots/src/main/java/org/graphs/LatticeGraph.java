package org.graphs;

import java.lang.reflect.Array;
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
    /**
     * Returns a list of all vertices in the lattice graph.
     * @return
     */
    public ArrayList<Vertex> getVertices() {
        return edges.keySet().parallelStream().collect(Collectors.toCollection(ArrayList::new));
    }
    /**
     * Returns a list of all the outgoing edges from a given vertex in the lattice graph.
     * @param vertex the vertex for which to retrieve outgoing edges
     * @return a list of all outgoing edges from the specified vertex, or an empty list if the vertex has no outgoing edges or is not represented in the graph
     */
    public ArrayList<LatticeEdge> getOutgoingEdges(Vertex vertex) {
        return edges.getOrDefault(vertex, new ArrayList<>());
    }
    /**
     * Returns the primary vertex of the lattice graph, which serves as the beginning point of the graph for formation purposes.
     * @return the primary vertex of the lattice graph, which is the first vertex in the graph's vertex set
     */
    public Vertex getPrimaryVertex() {
        return (Vertex) edges.keySet().toArray()[0]; // Assuming the first vertex is the primary vertex
    }

    public Vertex getVertexByID(int id) {
        ArrayList<Vertex> vertices = getVertices();
        for(Vertex v : vertices) {
            if(v.getId() == id) {
                return v;
            }
        }

        return null;
    }

    public LatticeEdge getOutgoingEdgeByID(int vertexID, int edgeID) {
        ArrayList<LatticeEdge> outgoingEdges = edges.get(getVertexByID(vertexID));
        for(LatticeEdge e : outgoingEdges) {
            if(e.getId() == edgeID) {
                return e;
            }
        }
        return null;
    }

    abstract int getNumberOfVertices();
}

