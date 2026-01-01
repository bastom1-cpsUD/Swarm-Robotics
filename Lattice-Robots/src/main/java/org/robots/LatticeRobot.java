package org.robots;

import java.util.Set;
import java.util.HashSet;
import java.util.Objects;
import java.util.Collections;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.RenderingHints;
import org.transformations.OrientedPoint;
import java.awt.Polygon;

public class LatticeRobot extends Polygon {
    //Robot unique identifier
    private final int AuthorityId;
    private OrientedPoint position;

    //Local knowledge & edges
    private Set<LatticeRobot> neighbors;
    private Set<Edge> edges;
    protected double offsetX;
    protected double offsetY;

    public LatticeRobot(int authorityId, OrientedPoint position) {
        this.AuthorityId = authorityId;
        this.position = position;
        this.neighbors = new HashSet<>();
        this.edges = new HashSet<>();
    }

    public void addNeighbor(LatticeRobot other) {
        this.neighbors.add(other);
        other.neighbors.add(this); // Ensure bidirectional connection
        this.edges.add(new Edge(this, other));
    }

    public void removeNeighbor(LatticeRobot neighbor) {
        this.neighbors.remove(neighbor);
        this.edges.removeIf(edge -> edge.getTo() == neighbor);
    }

    public String listNeighbors() {
        String result = "";
        for(LatticeRobot neighbor : neighbors) {
            result += neighbor.toString() + "\n";
        }
        return result;
    }

    public int getAuthorityId() {
        return AuthorityId;
    }

    public OrientedPoint getPosition() {
        return position;
    }

    public void setPosition(OrientedPoint position) {
        this.position = position;
    }

    public Set<LatticeRobot> getNeighbors() {
        return Collections.unmodifiableSet(neighbors);
    }

    public Set<Edge> getEdges() {
        return Collections.unmodifiableSet(edges);
    }

    @Override
    public String toString() {
        return "LatticeRobot[ID=" + AuthorityId + ", Position=" + position + "]";
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof LatticeRobot)) return false;
        LatticeRobot other = (LatticeRobot) obj;
        return AuthorityId == other.AuthorityId;
    }

    @Override
    public int hashCode() {
        return Objects.hash(AuthorityId);
    }

    public void draw(Graphics2D g2d) {
        updatePolygon();
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setColor(Color.BLACK);
        g2d.fill(this);
    }

    private void updatePolygon() {
        // Update polygon points based on current position and orientation
        double s = 40.0;
        double R = s / Math.sqrt(3.0);
        double theta0 = position.getOrientation();
        int[] xcoords = new int[4];
        int[] ycoords = new int[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  (int) Math.round(position.x + radius * Math.cos(angle));
        ycoords[0] =  (int) Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  (int) Math.round(position.x + R * Math.cos(angle));
        ycoords[1] =  (int) Math.round(position.y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  (int) Math.round(position.x + radius * Math.cos(angle));
        ycoords[2] =  (int) Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  (int) Math.round(position.x + R * Math.cos(angle));
        ycoords[3] =  (int) Math.round(position.y + R * Math.sin(angle));

        this.reset();
        for(int i = 0; i < xcoords.length; i++) {
            this.addPoint(xcoords[i], ycoords[i]);
        }

    }

    @Override
    public boolean contains(int x, int y) {
        updatePolygon();
        return super.contains(x, y);
    }
}