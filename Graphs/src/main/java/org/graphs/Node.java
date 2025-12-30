package org.graphs;

import java.util.List;
import java.util.ArrayList;

public class Node<T> {
    
    private T data;
    private List<Edge> outgoingEdges;

    public Node(T data) {
        this.data = data;
        this.outgoingEdges = new ArrayList<>();
    }

    public T getData() {
        return data;
    }
    
    public List<Edge> getOutgoingEdges() {
        return outgoingEdges;
    }

    public void addOutgoingEdge(Edge edge) {
        outgoingEdges.add(edge);
    }
}
