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


enum TrustLevel {
    Friendly,
    Suspected,
    Hostile
}

public class LatticeRobot extends Polygon {
    //Robot unique identifier
    private final int AuthorityId;
    private OrientedPoint position;

    //Local knowledge & edges
    private Set<Edge> edges;
    private TrustLevel trustLevel;
    protected double offsetX;
    protected double offsetY;

    public LatticeRobot(int authorityId, OrientedPoint position) {
        this.AuthorityId = authorityId;
        this.position = position;
        this.trustLevel = TrustLevel.Friendly;
        this.edges = new HashSet<>();
    }

    public void addNeighbor(LatticeRobot other) {
        this.edges.add(new Edge(this.getAuthorityId(), other.getAuthorityId()));
        other.edges.add(new Edge(other.getAuthorityId(), this.getAuthorityId()));
    }

    public void removeNeighbor(LatticeRobot neighbor) {
        this.edges.removeIf(edge -> edge.getToId() == neighbor.getAuthorityId());
    }

    public int getAuthorityId() {
        return AuthorityId;
    }

    public TrustLevel getTrustLevel() {
        return trustLevel;
    }

    public void setTrustLevel(TrustLevel trustLevel) {
        this.trustLevel = trustLevel;
    }

    public OrientedPoint getPosition() {
        return position;
    }

    public void setPosition(OrientedPoint position) {
        this.position = position;
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
        //Update polygon points before drawing
        updatePolygon();
        //Create trust level polygon
        Polygon trustPolygon = createTrustPolygon();

        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        // Draw trust level polygon
        switch (trustLevel) {
            case Friendly:
                g2d.setColor(new Color(0, 255, 0, 150)); // Semi-transparent green
                break;
            case Suspected:
                g2d.setColor(new Color(255, 255, 0, 150)); // Semi-transparent yellow
                break;
            case Hostile:
                g2d.setColor(new Color(255, 0, 0, 150)); // Semi-transparent red
                break;
        }
        g2d.fill(trustPolygon);

        // Draw robot polygon
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

    private Polygon createTrustPolygon() {

        //Calculate Centroid of the robot polygon
        int centroidX = 0;
        int centroidY = 0;
        for (int i = 0; i < this.npoints; i++) {
            centroidX += this.xpoints[i];
            centroidY += this.ypoints[i];
        }
        centroidX /= this.npoints;
        centroidY /= this.npoints;

        //Create a new polygon for trust level visualization
        Polygon trustPolygon = new Polygon();
        for (int i = 0; i < this.npoints; i++) {
            //Scale points away from centroid
            int scaledX = (int) Math.round(centroidX + 1.2 * (this.xpoints[i] - centroidX));
            int scaledY = (int) Math.round(centroidY + 1.2 * (this.ypoints[i] - centroidY));
            trustPolygon.addPoint(scaledX, scaledY);
        }

        return trustPolygon;
    }

    @Override
    public boolean contains(int x, int y) {
        updatePolygon();
        return super.contains(x, y);
    }
}