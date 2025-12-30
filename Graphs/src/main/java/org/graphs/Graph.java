package org.graphs;

import java.util.List;
import java.util.ArrayList;

public class Graph<T> {
    
    private List<Node<T>> nodes;

    public Graph() {
        this.nodes = new ArrayList<>();
    }
    
    public void addNode(Node<T> node) {
        nodes.add(node);
    }

    public void addDirectedEdge(Node<T> fromNode, Node<T> toNode, double weight) {
        new DirectedEdge(fromNode, toNode, weight);
    }

    public void addDirectedEdge(Node<T> fromNode, Node<T> toNode) {
        new DirectedEdge(fromNode, toNode);
    }

    public void addUndirectedEdge(Node<T> node1, Node<T> node2, double weight) {
        new UndirectedEdge(node1, node2, weight);
    }

    public void addUndirectedEdge(Node<T> node1, Node<T> node2) {
        new UndirectedEdge(node1, node2);
    }

    public List<Node<T>> getNodes() {
        return nodes;
    }

    public int getSize() {
        return nodes.size();
    }

    public void printGraph() {
        for (Node<?> node : nodes) {
            System.out.print(node.getData() + ": ");
            for (Edge edge : node.getOutgoingEdges()) {
                System.out.print(edge.toString() + " ");
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        Graph<String> g = new Graph<>();

        Node<String> a = new Node<>("A");
        Node<String> b = new Node<>("B");
        Node<String> c = new Node<>("C");
        Node<String> d = new Node<>("D");

        g.addNode(a);
        g.addNode(b);
        g.addNode(c);
        g.addNode(d);

        g.addDirectedEdge(a, b, 1.5);
        g.addUndirectedEdge(b, c, 2.0);
        g.addDirectedEdge(d, a, 0);

        g.printGraph();
    }
}