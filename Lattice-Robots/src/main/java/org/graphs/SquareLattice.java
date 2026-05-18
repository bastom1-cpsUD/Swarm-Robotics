package org.graphs;

import java.util.ArrayList;
import java.util.List;

public class SquareLattice extends LatticeGraph {
    public SquareLattice() {
        super();
        // Define the vertice of the square lattice
        OrientedPoint v0 = new OrientedPoint(0, 0, 0);
        this.vertices.add(v0);

        // Define the edges of the square lattice with appropriate transformations
        List<LatticeEdge> v0edges = new ArrayList<>();
        v0edges.add(new LatticeEdge(v0, new OrientedPoint(40, 0, 0))); // right
        v0edges.add(new LatticeEdge(v0, new OrientedPoint(0, 40, 0))); // up
        v0edges.add(new LatticeEdge(v0, new OrientedPoint(-40, 0, 0))); // left
        v0edges.add(new LatticeEdge(v0, new OrientedPoint(0, -40, 0))); // down

        this.edges.put(v0, v0edges);
    }
}
