package org.robots;

import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;

public class Edge {
    private int fromId;
    private int toId;

    public Edge(int fromId, int toId) {
        if(fromId == toId) {
            throw new IllegalArgumentException("An edge cannot connect a robot to itself.");
        }
        this.fromId = fromId;
        this.toId = toId;
    }

    public int getFromId() {
        return fromId;
    }

    public int getToId() {
        return toId;
    }

    public void draw(Graphics2D g2d, LatticeRobot from, LatticeRobot to) {

        Point2D pFrom = from.getPosition();
        Point2D pTo = to.getPosition();
        g2d.setColor(java.awt.Color.lightGray);
        g2d.setRenderingHint(java.awt.RenderingHints.KEY_ANTIALIASING, java.awt.RenderingHints.VALUE_ANTIALIAS_ON);

        //Calculate midpoint of the edge
        double mx = (pFrom.getX() + pTo.getX()) / 2;
        double my = (pFrom.getY() + pTo.getY()) / 2;

        double dx = (pTo.getX() - pFrom.getX());
        double dy = (pTo.getY() - pFrom.getY());

        //perpendicular vector
        double length = Math.sqrt(dx * dx + dy * dy);
        if(length == 0) {
            length = 1; //Prevent division by zero
        }

        double px = -dy / length;
        double py = dx / length;
        
        double nudgeAmount = 5.0; // Amount to nudge the edge
        mx += px * nudgeAmount;
        my += py * nudgeAmount;

        Path2D edge = new java.awt.geom.Path2D.Double();

        // Draw line from 'from' to midpoint
        edge.moveTo(pFrom.getX(), pFrom.getY());
        edge.lineTo(mx, my);

        // Set color based on trust level
        switch(from.getTrustLevel()) {
            case Friendly -> {
                g2d.setColor(new java.awt.Color(0, 255, 0)); // green
            }
            case Suspected -> {
                g2d.setColor(new java.awt.Color(255, 255, 0)); // yellow
            }
            case Hostile -> {
                g2d.setColor(new java.awt.Color(255, 0, 0)); // red
            }
        }
        // Draw arrowhead at midpoint
        int arrowSize = 6;
        int arrowWidth = 4;
        Polygon arrowHead = new Polygon();

        // Create arrowhead shape
        arrowHead.addPoint(0, 0);                     
        arrowHead.addPoint(-arrowSize, -arrowWidth);  
        arrowHead.addPoint(-arrowSize, arrowWidth);   

        // Apply transformation to position and rotate the arrowhead
        AffineTransform at = new AffineTransform();
        double angle = Math.atan2(dy, dx);
        at.translate(mx, my);
        at.rotate(angle);
        Shape arrow = at.createTransformedShape(arrowHead);
        g2d.fill(arrow);

        // Draw line from midpoint to 'to'
        g2d.setColor(java.awt.Color.lightGray);
        edge.lineTo(pTo.getX(), pTo.getY());
        g2d.draw(edge);
    }
}
