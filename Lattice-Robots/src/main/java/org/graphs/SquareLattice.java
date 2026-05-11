package org.graphs;

public class SquareLattice extends LatticeGraph {
    public SquareLattice() {
        super();
        // Define the vertice of the square lattice
        OrientedPoint v0 = new OrientedPoint(0, 0, 0);

        // Define the edges of the square lattice with appropriate transformations
        this.edges.add(new LatticeEdge(v0, new OrientedPoint(40, 0, 0))); // right
        this.edges.add(new LatticeEdge(v0, new OrientedPoint(0, 40, 0))); // up
        this.edges.add(new LatticeEdge(v0, new OrientedPoint(-40, 0, 0))); // left
        this.edges.add(new LatticeEdge(v0, new OrientedPoint(0, -40, 0))); // down
    }
}
