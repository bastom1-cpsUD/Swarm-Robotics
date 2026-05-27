package org.graphs;

import java.util.ArrayList;

public class SquareLattice extends LatticeGraph {
    public SquareLattice() {
        super();
        // Define the vertice of the square lattice
        Vertex v0 = new Vertex(0, new OrientedPoint(0, 0, 0));

        // Define the edges of the square lattice with appropriate transformations
        ArrayList<LatticeEdge> v0edges = new ArrayList<>();
        LatticeEdge e1 = new LatticeEdge(v0, new Vertex(0, new OrientedPoint(0, 0, 0)), new OrientedPoint(0,50,0)); // UP
        LatticeEdge e2 = new LatticeEdge(v0, new Vertex(0, new OrientedPoint(0, 0, 0)), new OrientedPoint(50, 0, 0)); // RIGHT
        LatticeEdge e3 = new LatticeEdge(v0, new Vertex(0, new OrientedPoint(0, 0, 0)), new OrientedPoint(0, -50, 0)); // DOWN
        LatticeEdge e4 = new LatticeEdge(v0, new Vertex(0, new OrientedPoint(0, 0, 0)), new OrientedPoint(-50, 0, 0)); // LEFT        

        v0edges.add(e1);
        v0edges.add(e2);
        v0edges.add(e3);
        v0edges.add(e4);

        this.edges.put(v0, v0edges);
    }
}
