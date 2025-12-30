package org.robots;

public class Edge {
    private LatticeRobot from;
    private LatticeRobot to;

    public Edge(LatticeRobot from, LatticeRobot to) {
        if(from == to) {
            throw new IllegalArgumentException("An edge cannot connect a robot to itself.");
        }
        this.from = from;
        this.to = to;
    }

    public LatticeRobot getFrom() {
        return from;
    }

    public LatticeRobot getTo() {
        return to;
    }
}
