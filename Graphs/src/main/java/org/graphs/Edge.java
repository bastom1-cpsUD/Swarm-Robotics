package org.graphs;

public interface Edge {
    Node<?> getFromNode();
    Node<?> getToNode();
    double getWeight();

    @Override
    String toString();
}
