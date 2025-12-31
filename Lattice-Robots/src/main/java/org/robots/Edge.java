package org.robots;

import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;

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

    public void draw(Graphics2D g2d) {
        Point2D pFrom = from.getPosition();
        Point2D pTo = to.getPosition();
        g2d.setColor(java.awt.Color.GRAY);
        g2d.setRenderingHint(java.awt.RenderingHints.KEY_ANTIALIASING, java.awt.RenderingHints.VALUE_ANTIALIAS_ON);
        
        Path2D edge = new java.awt.geom.Path2D.Double();

        edge.moveTo(pFrom.getX(), pFrom.getY());
        edge.lineTo(pTo.getX(), pTo.getY());
        edge.closePath();

        g2d.setColor(java.awt.Color.LIGHT_GRAY);
        g2d.draw(edge);
    }
}
