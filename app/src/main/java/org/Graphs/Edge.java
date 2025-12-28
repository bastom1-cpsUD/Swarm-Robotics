package org.Graphs;

public interface Edge {
    Node<?> getFromNode();
    Node<?> getToNode();
    double getWeight();

    @Override
    String toString();
}
