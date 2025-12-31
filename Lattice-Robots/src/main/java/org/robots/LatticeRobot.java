package org.robots;

import java.util.Set;
import java.util.HashSet;
import java.util.Objects;
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
        //Create 5 sample robots and connect them
        LatticeRobot robot1 = new LatticeRobot(1, new OrientedPoint(100, 100, 0));
        LatticeRobot robot2 = new LatticeRobot(2, new OrientedPoint(250, 150, Math.PI / 4));
        LatticeRobot robot3 = new LatticeRobot(3, new OrientedPoint(150, 250, Math.PI / 2));
        LatticeRobot robot4 = new LatticeRobot(4, new OrientedPoint(300, 200, Math.PI));

        //Add neighbors for robot 1
        robot1.addNeighbor(robot2);
        robot1.addNeighbor(robot3);
        robot1.addNeighbor(robot4);

        //Add neighbors for robot 2
        robot2.addNeighbor(robot4);

        //Add neighbors for robot 3
        robot3.addNeighbor(robot4); //Note: bidirectional connection ensured in addNeighbor


        //Print robot details
        System.out.println("Robot 1 ID: " + robot1.getAuthorityId());
        System.out.println("Robot 1 Position: " + robot1.getPosition());
        System.out.println("Robot 1 Neighbors: " + robot1.listNeighbors());

        System.out.println("Robot 2 ID: " + robot2.getAuthorityId());
        System.out.println("Robot 2 Position: " + robot2.getPosition());
        System.out.println("Robot 2 Neighbors: " + robot2.listNeighbors());

        //Create simple javaFX application to visualize robots and edge
        javax.swing.JFrame frame = new javax.swing.JFrame("Lattice Robots Visualization");
        frame.setSize(400, 400);
        frame.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);

        javax.swing.JPanel panel = new javax.swing.JPanel() {
            @Override
            protected void paintComponent(java.awt.Graphics g) {
                super.paintComponent(g);
                Graphics2D g2d = (Graphics2D) g;
                
                for(Edge edge : robot1.getEdges()) {
                    edge.draw(g2d);
                }
                for(Edge edge : robot2.getEdges()) {
                    edge.draw(g2d);
                }
                for(Edge edge : robot3.getEdges()) {
                    edge.draw(g2d);
                }
                for(Edge edge : robot4.getEdges()) {
                    edge.draw(g2d);
                }

                robot1.draw(g2d);
                robot2.draw(g2d);
                robot3.draw(g2d);
                robot4.draw(g2d);
            }
        };

        frame.add(panel);

        frame.setVisible(true);

    }
}