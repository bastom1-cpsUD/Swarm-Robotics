package org.graphs;

import java.util.ArrayList;

public class SquareLattice extends LatticeGraph {
    private final double EDGE_LENGTH = 50.0;

    public SquareLattice() {
        super();
        // Define the vertice of the square lattice
        Vertex v0 = new Vertex(0, new OrientedPoint(0, 0, 0));

        // Define the edges of the square lattice with appropriate transformations
        ArrayList<LatticeEdge> v0edges = new ArrayList<>();
        LatticeEdge e1 = new LatticeEdge(v0, v0, new OrientedPoint(0, EDGE_LENGTH, 0)); // UP
        LatticeEdge e2 = new LatticeEdge(v0, v0,  new OrientedPoint(EDGE_LENGTH, 0, 0)); // RIGHT
        LatticeEdge e3 = new LatticeEdge(v0, v0, new OrientedPoint(0, -EDGE_LENGTH, 0)); // DOWN
        LatticeEdge e4 = new LatticeEdge(v0, v0, new OrientedPoint(-EDGE_LENGTH, 0, 0)); // LEFT        

        v0edges.add(e1);
        v0edges.add(e2);
        v0edges.add(e3);
        v0edges.add(e4);

        this.edges.put(v0, v0edges);
    }

    public static void main(String[] args) {
        LatticeGraph graph = new SquareLattice();

        //Act as robot parent and get primary vertex
        Vertex parent = graph.getPrimaryVertex();
        
        ArrayList<LatticeEdge> parentEdges = graph.getOutgoingEdges(parent);

        System.out.println(parentEdges.size());

        
        for(LatticeEdge e : parentEdges) {
            System.out.println(e);
        }


        //Assign
        LatticeEdge assignedToChild = parentEdges.get(0);

        Vertex childVertex = assignedToChild.getTo();

        ArrayList<LatticeEdge> childEdges = graph.getOutgoingEdges(childVertex);

        System.out.println(childEdges.size());
        
        
        for(LatticeEdge e : childEdges) {
            System.out.println(e);
        }
    }
}
