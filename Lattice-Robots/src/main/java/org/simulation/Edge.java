package org.simulation;

import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;

import org.robots.GeometricCycleLatticeRobot;

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

    /**
     * Value equality on the endpoints, <strong>ordered</strong>.
     *
     * <p>{@code GeometricCycleLatticeRobot.addEdge} is a set-add spelled as a list-add:
     * {@code if(!edges.contains(e)) edges.add(e)}. Without this pair that guard compares
     * references, every caller hands it a freshly constructed instance, and it has therefore never
     * once fired. What made that visible was communication tuples becoming permanent -- a link now
     * carries many walks, and each one drew another identical line onto the robot, without bound.
     *
     * <p>Ordered rather than symmetric, because direction is real here. {@link #draw} puts an
     * arrowhead at the midpoint pointing from {@code fromId} to {@code toId}, and each robot keeps
     * its own outgoing view of a link -- a parent holds {@code (parent, child)} while the child
     * holds {@code (child, parent)}, on two different robots' lists. Treating those as equal would
     * be meaningless (they never meet) and would make the arrow direction depend on which was
     * added first.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof Edge)) {
            return false;
        }
        Edge other = (Edge) o;
        return this.fromId == other.fromId && this.toId == other.toId;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(fromId, toId);
    }

    public void draw(Graphics2D g2d, GeometricCycleLatticeRobot from, GeometricCycleLatticeRobot to) {

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

    @Override
    public String toString() {
        return "Edge connects robot " + fromId + " to robot " + toId;
    }
}


