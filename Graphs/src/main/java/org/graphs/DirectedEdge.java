package org.graphs;

public class DirectedEdge implements Edge{
    // Learn Graddle, Maven
    // Step 1: Graddle this and Tranformations
    private Node<?> fromNode;
    private Node<?> toNode;
    private double weight;

    public DirectedEdge(Node<?> fromNode, Node<?> toNode) {
        this(fromNode, toNode, 0.0);
    }

    public DirectedEdge(Node<?> fromNode, Node<?> toNode, double weight) {
        this.fromNode = fromNode;
        this.toNode = toNode;
        this.weight = weight;

        fromNode.addOutgoingEdge(this);
    }

    public Node<?> getFromNode() {
        return fromNode;
    }

    public Node<?> getToNode() {
        return toNode;
    }

    public double getWeight() {
        return weight;
    }

    @Override
    public String toString() {
        return fromNode.getData() + " --> " + toNode.getData() + " (weight: " + weight + ")";
    }
}
