package org.robots;

import java.util.Set;
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


enum TrustLevel {
    Friendly,
    Suspected,
    Hostile
}

public class LatticeRobot extends Polygon {
    //Robot unique identifier
    private Integer parentID; // Placeholder for parent robot reference in the hierarchy
    private final int AuthorityId;
    private AuthorityList authorityList; //Placeholder for authority list implementation
    private OrientedPoint position;
    private Queue<Message> messageQueue; // Placeholder for message queue implementation

    //Local knowledge & edges
    private static final LatticeGraph LATTICE_GRAPH = new SquareLattice(); // Placeholder for graph implementation
    private OrientedPoint destination; // Placeholder for destination vertex assignment
    private Set<Edge> edges;
    private TrustLevel trustLevel;
    private ArrayList<LatticeRobot> neighbors;

    private static final int ROBOT_SIZE = 40; // Size of the robot for drawing

    public LatticeRobot(int authorityId, OrientedPoint position) {
        this.AuthorityId = authorityId;
        this.position = position;
        this.trustLevel = TrustLevel.Friendly;
        this.edges = new HashSet<>();
        this.neighbors = new ArrayList<>();
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
            parentID = null;
        } else {
            //Assume follower role
            parentID = ownAuthority.getMostRecentAuthority();
        }
    }

    //Tomorrow Problem
    public void assignVertices() {
        boolean isRoot = (parentID == null);
        RigidBodyTransformation GlobalToLocalCoords = new RigidBodyTransformation(this.position).inverse();

        double[][] costMatrix = calculateCostMatrix(LATTICE_GRAPH.getOutgoingEdges(LATTICE_GRAPH.getPrimaryVertex()), neighbors);
    }

    private double[][] calculateCostMatrix(List<LatticeEdge> edges, List<LatticeRobot> neighbors) {
        boolean isRoot = (parentID == null);
        int matrixSize = Math.max(edges.size(), neighbors.size());
        RigidBodyTransformation GlobalToLocalCoords = new RigidBodyTransformation(this.position).inverse();
        double[][] costMatrix = new double[matrixSize][matrixSize];

        if(isRoot) {
            //Calculate cost matrix based on distance from neighbors to outgoing edges; neighbors are rows, edges are columns
            for(int i = 0; i < neighbors.size(); i++) {
                for(int j = 0; j < edges.size(); j++) {
                    //Calculate neighbor's relative position in the local coordinate system of the root
                    OrientedPoint neighborLocalPos = GlobalToLocalCoords.apply(neighbors.get(i).getPosition());
                    OrientedPoint desiredPosition = edges.get(j).getEdgeTransformation().apply(new OrientedPoint(0,0,0));
                    //Calculate distance to the edge's target vertex
                    double distanceToEdge = neighborLocalPos.distanceTo(desiredPosition);

                    costMatrix[i][j] = distanceToEdge;
                }
            }

            return costMatrix;
        } else {
            //Calculate cost matrix based on distance from neighbors to incoming edges; neighbors are rows, edges are columns
             for(int i = 0; i < neighbors.size(); i++) {
                for(int j = 0; j < edges.size(); j++) {
                    //Calculate neighbor's relative position in the local coordinate system of the root
                    OrientedPoint neighborLocalPos = GlobalToLocalCoords.apply(neighbors.get(i).getPosition());
                    OrientedPoint desiredPosition = edges.get(j).getEdgeTransformation().apply(new OrientedPoint(0,0,0));
                    //Calculate distance to the edge's target vertex
                    double distanceToEdge = neighborLocalPos.distanceTo(desiredPosition);

                    if(neighbors.get(i).getAuthorityId() == parentID) {
                        /*
                        
                        */
                        
                        continue;
                    }

                    costMatrix[i][j] = distanceToEdge;
                }
            }
        }  
        
        
        //Pad remaining cells if necessary

        //If there are less neighbors than edges, pad remaining rows with high cost to prevent assignment
        if(neighbors.size() < matrixSize) {
            for(int i = neighbors.size(); i < matrixSize; i++) {
                for(int j = 0; j < edges.size(); j++) {
                    costMatrix[i][j] = 1000; // Assign a high cost to discourage assignment
                }
            }
        } else if(edges.size() < matrixSize) {
            //If there are less edges than neighbors, pad remaining columns with high cost to prevent assignment
            for(int i = 0; i < neighbors.size(); i++) {
                for(int j = edges.size(); j < matrixSize; j++) {
                    costMatrix[i][j] = 1000; // Assign a high cost to discourage assignment                    }
                }
            }
        }
        
        return null;
    }

    //Placeholder for message sending method, to be implemented in future iterations
    public void sendMessage() {
        Message msg = new Message(this.authorityList);

    }
}
