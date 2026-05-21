package org.robots;

import java.util.Set;

import javax.swing.TransferHandler;

import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.ArrayList;
import java.util.Collections;
import java.awt.Graphics2D;
import java.awt.Color;
import java.awt.RenderingHints;

import org.graphs.RigidBodyTransformation;
import org.graphs.LatticeEdge;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
import java.awt.Polygon;
import org.graphs.SquareLattice;
import org.graphs.Vertex;
import org.robots.HungarianAlgo.HungarianAlgo;


enum TrustLevel {
    Friendly,
    Suspected,
    Hostile
}

public class LatticeRobot extends Polygon {
    //Robot unique identifier
    private final int AuthorityId;
    private AuthorityList authorityList; //Placeholder for authority list implementation
    private OrientedPoint position;
    private Queue<Message> messageQueue; // Placeholder for message queue implementation

    //Local knowledge & edges
    private static final LatticeGraph LATTICE_GRAPH = new SquareLattice(); // Placeholder for graph implementation
    private LatticeRobot parent; // Placeholder for parent robot reference in the hierarchy
    private Set<Edge> edges;
    private TrustLevel trustLevel;
    private ArrayList<LatticeRobot> neighbors;

    public LatticeEdge assignedEdge;


    private static final int ROBOT_SIZE = 40; // Size of the robot for drawing

    public LatticeRobot(int authorityId, OrientedPoint position) {
        this.AuthorityId = authorityId;
        this.position = position;
        this.trustLevel = TrustLevel.Friendly;
        this.edges = new HashSet<>();
        this.neighbors = new ArrayList<>();
        this.assignedEdge = null;
        this.parent = null;
    }

    public void addNeighbor(LatticeRobot other) {
        //Check if edge already exists to prevent duplicates
        boolean edgeExists = this.edges.stream().anyMatch(edge -> edge.getToId() == other.getAuthorityId());

        if(!edgeExists) {
            this.edges.add(new Edge(this.getAuthorityId(), other.getAuthorityId()));
            other.edges.add(new Edge(other.getAuthorityId(), this.getAuthorityId()));
            this.neighbors.add(other);
            other.neighbors.add(this);
        }
    }

    public void removeNeighbor(LatticeRobot neighbor) {
        neighbor.edges.removeIf(edge -> edge.getToId() == this.getAuthorityId());
        this.edges.removeIf(edge -> edge.getToId() == neighbor.getAuthorityId());
        this.neighbors.remove(neighbor);
        neighbor.neighbors.remove(this);
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
        return AuthorityId == other.AuthorityId && authorityList.equals(other.authorityList);
    }

    @Override
    public int hashCode() {
        return Objects.hash(AuthorityId, authorityList);
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
        double s = ROBOT_SIZE;
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

    //MUST FIX SELECT ROLE WITH REGARDS TO ROLES
    private void selectRole() {
        //Step 1: Discard message where self appears in the authority list
        messageQueue.removeIf(msg -> msg.getAuthority().contains(this.AuthorityId));
        //Step 2: Form own authority list composed of own id
        AuthorityList ownAuthority = new AuthorityList(this.AuthorityId);
        AuthorityList greatestAuthority = ownAuthority;
        //Step 3: For the remaining messages, compare own authority list with the authority list in the message. 
        for(Message msg : messageQueue) {
            AuthorityList msgAuthority = msg.getAuthority();
            if(msgAuthority.compareTo(greatestAuthority) < 0) {
                greatestAuthority = msgAuthority;
            }
        }
        //Decide role based on outcome of comparison
        if (greatestAuthority.equals(ownAuthority)) {
            //Assume leader role
            parent = null;
        } else {
            //Assume follower role
            parent = null;
        }
    }

    //Tomorrow Problem
    public void assignVertices() {
        double[][] costMatrix = calculateCostMatrix();
        
        for(int i = 0; i < costMatrix.length; i++) {
            for(int j = 0; j < costMatrix[0].length; j++) {
                System.out.print(costMatrix[i][j] + " ");
            }
            System.out.println();
        }

        HungarianAlgo hung = new HungarianAlgo(costMatrix);
        int[][] assignment = hung.solve();
        System.out.println("Optimal Assignment:");
        for(int i = 0; i < assignment.length; i++) {
            System.out.println("Robot " + (assignment[i][0] + 1) + " assigned to Edge " + (assignment[i][1] + 1));
        }

    }

    private double[][] calculateCostMatrix() {
        //Determine vertex
        Vertex myVertex = parent == null ? LATTICE_GRAPH.getPrimaryVertex() : assignedEdge.getTo();
        
        //Get outgoing edges of vertex
        ArrayList<LatticeEdge> outgoingEdges = LATTICE_GRAPH.getOutgoingEdges(myVertex);

        //Get global to local matrix for multiplication
        final RigidBodyTransformation GLOBAL_TO_LOCAL = new RigidBodyTransformation(this.position).inverse();
        final int INF = 1000;

        double[][] cost = new double[neighbors.size()][outgoingEdges.size()];
        
        //If robot is a root, simply get neighbors and assign them edges
        if(parent == null) {
            for(int i = 0; i < neighbors.size(); i++) {
                for(int j = 0; j < outgoingEdges.size(); j++) {
                    //Get local position of neighboring robot
                    OrientedPoint localPos = GLOBAL_TO_LOCAL.apply(neighbors.get(i).getPosition());

                    //Get destination of the edge in local coordinates
                    OrientedPoint destination = outgoingEdges.get(j).getEdgeTransformation().apply(new OrientedPoint(0,0,0));
                    
                    //Assign euclidean distance between two points to the cost matrix entry
                    cost[i][j] = localPos.distance(destination);
                }
            }
        } 
        //If robot is a child, ensure child obeys assignment by assigning parent to its prescribed edge
        else {
            for(int i = 0; i < neighbors.size(); i++) {
                //Assign parent correct inverse edge
                if(neighbors.get(i) == parent) {
                    for(int j = 0; j < outgoingEdges.size(); j++) {
                        if(outgoingEdges.get(j).getEdgeTransformation().isInverse(assignedEdge.getEdgeTransformation())) {
                            cost[i][j] = 0;
                        } else {
                            cost[i][j] = INF;
                        }
                    }
                } else {
                    for(int j = 0; j < outgoingEdges.size(); j++) {
                        //Get local position of neighboring robot
                        OrientedPoint localPos = GLOBAL_TO_LOCAL.apply(neighbors.get(i).getPosition());

                        //Get destination of the edge in local coordinates
                        OrientedPoint destination = outgoingEdges.get(j).getEdgeTransformation().apply(new OrientedPoint(0,0,0));
                        
                        //Assign euclidean distance between two points to the cost matrix entry
                        cost[i][j] = localPos.distance(destination);
                    }
                }
            }
        }
    
        return cost;
    }

    //Placeholder for message sending method, to be implemented in future iterations
    public void sendMessage() {
        Message msg = new Message(this.authorityList);

    }

    public static void main(String[] args) {
        LatticeRobot L1 = new LatticeRobot(1,new OrientedPoint(80, 40, 0));
        LatticeRobot L2 = new LatticeRobot(2,new OrientedPoint(40, 0, 0));
        LatticeRobot L3 = new LatticeRobot(3, new OrientedPoint(40, 40, 0));
        LatticeRobot L4 = new LatticeRobot(4, new OrientedPoint(80, 120, 0));
        LatticeRobot L5 = new LatticeRobot(5, new OrientedPoint(120, 80, 0));

        L1.addNeighbor(L2);
        L1.addNeighbor(L3);
        L1.addNeighbor(L4);
        L1.addNeighbor(L5);

        L2.addNeighbor(L3);
        L2.addNeighbor(L4);
        L2.addNeighbor(L5);

        L3.addNeighbor(L4);
        L3.addNeighbor(L5);

        
        L3.parent = L1;
        L3.assignedEdge = LATTICE_GRAPH.getOutgoingEdges(LATTICE_GRAPH.getPrimaryVertex()).get(3);

        L3.assignVertices();
    }
}
