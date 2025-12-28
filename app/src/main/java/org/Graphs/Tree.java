package org.Graphs;

import java.util.List;
import java.util.ArrayList;

public class Tree<T> {
    
    private List<TreeNode<T>> nodes;

    public Tree() {
        nodes = new ArrayList<>();
    }

    public void addNode(TreeNode<T> node) {
        nodes.add(node);
    }

    public void addEdge(TreeNode<T> parent, TreeNode<T> child, double weight) {
        
        new DirectedEdge(parent, child, weight);
        parent.addChild(child);

    }

     public void addEdge(TreeNode<T> parent, TreeNode<T> child) {
        addEdge(parent, child, 0.0);
    }

    public List<TreeNode<T>> getNodes() {
        return nodes;
    }

    public void printTree() {
        for (TreeNode<?> node : nodes) {
            System.out.print(node.getData() + ": ");
            for (Edge edge : node.getOutgoingEdges()) {
                System.out.print(edge.toString() + " ");
            }
            System.out.println();
        }
    }

    public static void main(String[] args) {
        TreeNode<Integer> root = new TreeNode<>(1);
        TreeNode<Integer> node1 = new TreeNode<>(2);
        TreeNode<Integer> node2 = new TreeNode<>(3);
        TreeNode<Integer> node3 = new TreeNode<>(4);
        TreeNode<Integer> node4 = new TreeNode<>(5);

        Tree<Integer> tree = new Tree<>();

        tree.addNode(root);
        tree.addNode(node1);
        tree.addNode(node2);
        tree.addNode(node3);
        tree.addNode(node4);

        tree.addEdge(root, node1, 1.0);
        tree.addEdge(root, node2, 3.0);
        tree.addEdge(root, node3, 5.0);
        tree.addEdge(node2, node4, 3.0);

        tree.printTree();
    }
}
