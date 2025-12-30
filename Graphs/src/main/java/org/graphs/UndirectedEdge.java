package org.graphs;

public class UndirectedEdge implements Edge {
    
    private Node<?> node1;
    private Node<?> node2;
    private double weight;

    public UndirectedEdge(Node<?> node1, Node<?> node2) {
        this(node1, node2, 0.0);
    }

    public UndirectedEdge(Node<?> node1, Node<?> node2, double weight) {
        this.node1 = node1;
        this.node2 = node2;
        this.weight = weight;

        node1.addOutgoingEdge(this);
        node2.addOutgoingEdge(this);
    }

    public Node<?> getFromNode() {
        return node1;
    }
    
    public Node<?> getToNode() {
        return node2;
    }

    public double getWeight() {
        return weight;
    }

    @Override
    public String toString() {
        return node1.getData() + " --- " + node2.getData() + " (weight: " + weight + ")";
    }
}

