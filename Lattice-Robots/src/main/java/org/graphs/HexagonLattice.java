package org.graphs;

import java.util.ArrayList;

public class HexagonLattice extends LatticeGraph {
    private static final double EDGE_LENGTH = 50.0;

    public HexagonLattice() {
        super();

        // Define the vertice of the hexagon lattice
        Vertex v0 = new Vertex(0, new OrientedPoint(0,0,0));
        Vertex v1 = new Vertex(1, new OrientedPoint(EDGE_LENGTH * Math.cos(Math.toRadians(60)), EDGE_LENGTH * Math.sin(Math.toRadians(60)), 0));        
    
        // Define outgoing edges for v0
        ArrayList<LatticeEdge> v0edges = new ArrayList<>();

        LatticeEdge e1 = new LatticeEdge(v0, v1, new OrientedPoint(EDGE_LENGTH * Math.cos(Math.toRadians(60)), EDGE_LENGTH * Math.sin(Math.toRadians(60)), 0)); // UP-RIGHT
        LatticeEdge e2 = new LatticeEdge(v0, v1, new OrientedPoint(-EDGE_LENGTH, 0, 0)); // Left
        LatticeEdge e3 = new LatticeEdge(v0, v1, new OrientedPoint(EDGE_LENGTH * Math.cos(Math.toRadians(60)), -EDGE_LENGTH *EDGE_LENGTH * Math.sin(Math.toRadians(60)), 0)); // DOWN-RIGHT
    
        v0edges.add(e1);
        v0edges.add(e2);
        v0edges.add(e3);
        this.edges.put(v0, v0edges);

        // Define outgoing edges for v1
        ArrayList<LatticeEdge> v1edges = new ArrayList<>();
        LatticeEdge e4 = new LatticeEdge(v1, v0, new OrientedPoint(-EDGE_LENGTH * Math.cos(Math.toRadians(60)), -EDGE_LENGTH * Math.sin(Math.toRadians(60)), 0)); // DOWN-LEFT
        LatticeEdge e5 = new LatticeEdge(v1, v0, new OrientedPoint(EDGE_LENGTH, 0, 0)); // Right
        LatticeEdge e6 = new LatticeEdge(v1, v0, new OrientedPoint(-EDGE_LENGTH * Math.cos(Math.toRadians(60)), EDGE_LENGTH * Math.sin(Math.toRadians(60)), 0)); // UP-LEFT

        v1edges.add(e4);
        v1edges.add(e5);
        v1edges.add(e6);
        
        this.edges.put(v1, v1edges);
    }
}
