package org.robots;

import java.util.Set;
import java.util.HashSet;
import java.util.Collections;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.RenderingHints;
import java.awt.geom.Path2D;
import org.transformations.OrientedPoint;

public class LatticeRobot {
    //Robot unique identifier
    private final int AuthorityId;
    private final OrientedPoint position;

    //Local knowledge & edges
    private Set<LatticeRobot> neighbors;
    private Set<Edge> edges;

    public LatticeRobot(int authorityId, OrientedPoint position) {
        this.AuthorityId = authorityId;
        this.position = position;
        this.neighbors = new HashSet<>();
        this.edges = new HashSet<>();
    }

    public void addNeighbor(LatticeRobot neighbor) {
        this.neighbors.add(neighbor);
        this.edges.add(new Edge(this, neighbor));
    }

    public void removeNeighbor(LatticeRobot neighbor) {
        this.neighbors.remove(neighbor);
        this.edges.removeIf(edge -> edge.getTo() == neighbor);
    }

    public int getAuthorityId() {
        return AuthorityId;
    }

    public OrientedPoint getPosition() {
        return position;
    }

    public Set<LatticeRobot> getNeighbors() {
        return Collections.unmodifiableSet(neighbors);
    }

    public Set<Edge> getEdges() {
        return Collections.unmodifiableSet(edges);
    }

    public void draw(Graphics2D g2d) {
        // Drawing logic for the robot
        double s = 40.0;
        double R = s / Math.sqrt(3.0);
        double theta0 = position.getOrientation();
        long[] xcoords = new long[4];
        long[] ycoords = new long[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  Math.round(position.x + radius * Math.cos(angle));
        ycoords[0] =  Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  Math.round(position.x + R * Math.cos(angle));
        ycoords[1] =  Math.round(position.y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  Math.round(position.x + radius * Math.cos(angle));
        ycoords[2] =  Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  Math.round(position.x + R * Math.cos(angle));
        ycoords[3] =  Math.round(position.y + R * Math.sin(angle));

        //Enable anti-aliasing for smoother rendering
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        //Draw the robot as a filled polygon
        Path2D.Double shape = new Path2D.Double();
        shape.moveTo(xcoords[0], ycoords[0]);
        shape.lineTo(xcoords[1], ycoords[1]);
        shape.lineTo(xcoords[2], ycoords[2]);
        shape.lineTo(xcoords[3], ycoords[3]);
        shape.closePath();

        g2d.setColor(Color.BLACK);
        g2d.fill(shape);
    }

    public static void main(String[] args) {
        System.out.println("LatticeRobot module works!");
    }
}