package org.communicationModels;

import java.util.ArrayList;
import org.graphs.LatticeGraph;
import org.graphs.OrientedPoint;
import org.graphs.RigidBodyTransformation;
import org.robots.LatticeRobot;
import org.communicationModels.HungarianAlgo.HungarianAlgo;
import org.communicationModels.HungarianAlgo.HungarianMatrixUtils;
import org.graphs.LatticeEdge;
import org.graphs.SquareLattice;
import org.graphs.Vertex;

public class DecentralizedComms extends CommunicationSystem{
    protected LatticeRobot self;
    protected Integer parentId;
    protected TrustLevel trustLevel;
    protected ArrayList<LatticeRobot> commPeers;
    protected LatticeEdge assignedEdge;
    protected AuthorityList authorityList;
    protected final LatticeGraph LATTICE_GRAPH = new SquareLattice();

    public DecentralizedComms(int robotId, LatticeRobot self) {
        this.self = self;
        this.parentId = null;
        this.commPeers = null;
        assignedEdge = null;
        authorityList = new AuthorityList(self.getRobotId());
        trustLevel = TrustLevel.Friendly;
    }

    @Override
    public void processMessages() {
        //Step 1: Discard message where self appears in the authority list
        incomingMessages.removeIf(msg -> msg.getAuthority().contains(self.getRobotId()));
        //Step 2: Form own authority list composed of own id
        AuthorityList ownAuthority = new AuthorityList(self.getRobotId());
        AuthorityList greatestAuthority = ownAuthority;
        //Step 3: For the remaining messages, compare own authority list with the authority list in the message. 
        for(Message msg : incomingMessages) {
            AuthorityList msgAuthority = msg.getAuthority();
            if(msgAuthority.compareTo(greatestAuthority) < 0) {
                greatestAuthority = msgAuthority;
                assignedEdge = msg.getAssignment();
            }
        }

        //Step 4: Adopt greatest authority
        if(ownAuthority.equals(greatestAuthority)) { 
            parentId = null;
            incomingMessages.clear();
            return;
        }
    
        parentId = greatestAuthority.getMostRecentAuthority();
        greatestAuthority.addAuthority(self.getRobotId());
        authorityList = greatestAuthority;
        incomingMessages.clear();
    }

    public void broadcastAssignment() {
        int[][] assignments = assignVertices();

        //Determine vertex
        Vertex myVertex = isRoot() ? LATTICE_GRAPH.getPrimaryVertex() : assignedEdge.getTo();
        
        //Get outgoing edges of vertex
        ArrayList<LatticeEdge> outgoingEdges = LATTICE_GRAPH.getOutgoingEdges(myVertex);

        for(int i = 0; i < assignments.length; i++) {
           Integer robot = assignments[i][0] >= commPeers.size() ? null : assignments[i][0];
           Integer edge = assignments[i][1] >= outgoingEdges.size() ? null : assignments[i][1];

           if(robot != null) {
                Message msg = new Message(this.authorityList, edge == null ? null : outgoingEdges.get(edge));
                commPeers.get(robot).enqueueMessage(msg);
           }
        }
    }

    public OrientedPoint retrieveAssignmentLocation() {
        if(parentId == null) {
            return null;
        }

        LatticeRobot parent = null;

        for(LatticeRobot robot : commPeers) {
            if(parentId.equals(robot.getRobotId())) {
                parent = robot;
            }
        }

        //Get transformation of parent that translates local coords to global positions
        RigidBodyTransformation parentLocalToGlobal = new RigidBodyTransformation(parent.getPosition());

        //Apply transformation of parent to assigned position to get global position of assigned target
        OrientedPoint assignedLocationGlobal = parentLocalToGlobal.apply(assignedEdge.getTo().getPose());
    
        //Do we need to convert to local if this is just for positioning???

        return assignedLocationGlobal;
    }

    public boolean isRoot() {
        return authorityList.isRoot(self.getRobotId());
    }

    public boolean isAssigned() {
        return assignedEdge == null;
    }

    public void syncPeers(ArrayList<LatticeRobot> neighbors) {
        this.commPeers = neighbors;
    }

    public TrustLevel getTrustLevel() {
        return trustLevel;
    }

    public void setTrustLevel(TrustLevel trust) {
        this.trustLevel = trust;
    }

    private int[][] assignVertices() {
        double[][] costMatrix = calculateCostMatrix();
        
        /*for(int i = 0; i < costMatrix.length; i++) {
            for(int j = 0; j < costMatrix[0].length; j++) {
                System.out.print(costMatrix[i][j] + " ");
            }
            System.out.println();
        }*/

        HungarianAlgo hung = new HungarianAlgo(costMatrix);
        int[][] assignment = hung.solve();
        
        /*System.out.println("Optimal Assignment:");
        for(int i = 0; i < assignment.length; i++) {
            System.out.println("Robot " + (assignment[i][0] + 1) + " assigned to Edge " + (assignment[i][1] + 1));
        }*/

        return assignment;
    }

    private double[][] calculateCostMatrix() {
        //Determine vertex
        Vertex myVertex = isRoot() ? LATTICE_GRAPH.getPrimaryVertex() : assignedEdge.getTo();
        
        //Get outgoing edges of vertex
        ArrayList<LatticeEdge> outgoingEdges = LATTICE_GRAPH.getOutgoingEdges(myVertex);

        //Get global to local matrix for multiplication
        final RigidBodyTransformation GLOBAL_TO_LOCAL = new RigidBodyTransformation(self.getPosition()).inverse();

        double[][] cost = new double[commPeers.size()][outgoingEdges.size()];

        //If robot is a root, simply get commPeers and assign them edges
        if(isRoot()) {
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
                if(commPeers.get(i).getRobotId() == parentId) {
                    for(int j = 0; j < outgoingEdges.size(); j++) {
                        if(outgoingEdges.get(j).getEdgeTransformation().isInverse(assignedEdge.getEdgeTransformation())) {
                            cost[i][j] = 0.0;
                        } else {
                            cost[i][j] = HungarianMatrixUtils.INFINITY;
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

        if(commPeers.size() != outgoingEdges.size()) {
           cost = HungarianMatrixUtils.padToSquare(cost);
        }
    
        return cost;
    }
}
