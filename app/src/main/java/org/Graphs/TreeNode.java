package org.Graphs;

import java.util.List;
import java.util.ArrayList;

public class TreeNode<T> extends Node<T> {
    
    private TreeNode<T> parent;
    private List<TreeNode<T>> children;

    public TreeNode(T data) {
        super(data);
        parent = null;
        children = new ArrayList<>();
    }

    public TreeNode(T data, TreeNode<T> parent) {
        super(data);
        this.parent = parent;
        children = new ArrayList<>();
    }

    public boolean isRootNode() {
        return parent == null;
    }

    public boolean isLeafNode() {
        return (children.size() == 0);
    }

    public void addParent(TreeNode<T> node) {
        this.parent = node;
    }

    public void addChild(TreeNode<T> node) {
        this.children.add((TreeNode<T>) node);
    }

    public List<TreeNode<T>> getChildren() {
        return children;
    }
}
