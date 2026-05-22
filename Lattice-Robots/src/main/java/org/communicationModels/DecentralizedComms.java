package org.communicationModels;

import java.util.ArrayList;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
import org.graphs.RigidBodyTransformation;
import org.robots.LatticeRobot;
import org.communicationModels.HungarianAlgo.HungarianAlgo;
import org.graphs.LatticeEdge;
import org.graphs.SquareLattice;
import org.graphs.Vertex;

public class DecentralizedComms extends CommunicationSystem{
    protected LatticeRobot self;
    protected LatticeRobot parent;
    protected TrustLevel trustLevel;
    protected ArrayList<LatticeRobot> commPeers;
    protected LatticeEdge assignedEdge;
    protected AuthorityList authorityList;
    protected final LatticeGraph LATTICE_GRAPH = new SquareLattice();

    public DecentralizedComms(int robotId, LatticeRobot self) {
        this.self = self;
        this.parent = null;
        this.commPeers = null;
        assignedEdge = null;
        authorityList = new AuthorityList(robotId);
        trustLevel = TrustLevel.Friendly;
    }

    @Override
    public void processMessages() {

    }

    public boolean isRoot() {
        return parent == null;
    }

    public boolean isAssigned() {
        return assignedEdge == null;
    }

    public void syncPeers(ArrayList<LatticeRobot> neighbors) {
        
    }

    public TrustLevel getTrustLevel() {
        return trustLevel;
    }

    public void setTrustLevel(TrustLevel trust) {
        this.trustLevel = trust;
    }

    //MUST FIX SELECT ROLE WITH REGARDS TO ROLES
    private void selectRole() {
        //Step 1: Discard message where self appears in the authority list
        incomingMessages.removeIf(msg -> msg.getAuthority().contains(this.authorityList.getMostRecentAuthority()));
        //Step 2: Form own authority list composed of own id
        AuthorityList ownAuthority = new AuthorityList(this.authorityList.getMostRecentAuthority());
        AuthorityList greatestAuthority = ownAuthority;
        //Step 3: For the remaining messages, compare own authority list with the authority list in the message. 
        for(Message msg : incomingMessages) {
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
        final RigidBodyTransformation GLOBAL_TO_LOCAL = new RigidBodyTransformation(self.getPosition()).inverse();
        final int INF = 1000;

        double[][] cost = new double[commPeers.size()][outgoingEdges.size()];
        
        //If robot is a root, simply get commPeers and assign them edges
        if(parent == null) {
            for(int i = 0; i < commPeers.size(); i++) {
                for(int j = 0; j < outgoingEdges.size(); j++) {
                    //Get local position of neighboring robot
                    OrientedPoint localPos = GLOBAL_TO_LOCAL.apply(commPeers.get(i).getPosition());

                    //Get destination of the edge in local coordinates
                    OrientedPoint destination = outgoingEdges.get(j).getEdgeTransformation().apply(new OrientedPoint(0,0,0));
                    
                    //Assign euclidean distance between two points to the cost matrix entry
                    cost[i][j] = localPos.distance(destination);
                }
            }
        } 
        //If robot is a child, ensure child obeys assignment by assigning parent to its prescribed edge
        else {
            for(int i = 0; i < commPeers.size(); i++) {
                //Assign parent correct inverse edge
                if(commPeers.get(i) == parent) {
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
                        OrientedPoint localPos = GLOBAL_TO_LOCAL.apply(commPeers.get(i).getPosition());

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

}
