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

/**
 * A Decentralized communication system based on the proposed methodology outlined in Song & OKane 2014
 */
public class DecentralizedComms extends CommunicationSystem {

    public enum Role {
        root,
        assignedChild,
        unassignedChild
    }

    protected Role role;
    protected LatticeRobot self;
    protected Integer parentId;
    protected TrustLevel trustLevel;
    protected ArrayList<LatticeRobot> commPeers;
    protected LatticeEdge assignedEdge;
    protected AuthorityList authorityList;
    protected static final LatticeGraph LATTICE_GRAPH = new SquareLattice();

    public DecentralizedComms(int robotId, LatticeRobot self) {
        this.self = self;
        this.parentId = -1;
        this.commPeers = new ArrayList<LatticeRobot>();
        assignedEdge = new LatticeEdge();
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

        //Step 4: Adopt greatest authority or retain own authority
        if(ownAuthority.equals(greatestAuthority)) { 
            role = Role.root;
            parentId = -1;
            assignedEdge = new LatticeEdge();
            authorityList = new AuthorityList(self.getRobotId());
            incomingMessages.clear();
            return;
        }
        
        role = (assignedEdge.isNull()) ? Role.unassignedChild : Role.assignedChild;
        parentId = greatestAuthority.getMostRecentAuthority();
        //Make copy of authority list to avoid cycling
        AuthorityList myAuthority = new AuthorityList(greatestAuthority.getAuthorities());
        myAuthority.addAuthority(self.getRobotId());
        authorityList = myAuthority;
        incomingMessages.clear();
    }

    public void broadcastAssignment() {
        if(!isAssigned() && !isRoot() || commPeers.isEmpty()) {
            return;
        }
        int[][] assignments = assignVertices();

        //Determine vertex
        Vertex myVertex = isRoot() ? LATTICE_GRAPH.getPrimaryVertex() : assignedEdge.getTo();
        
        //Get outgoing edges of vertex
        ArrayList<LatticeEdge> outgoingEdges = LATTICE_GRAPH.getOutgoingEdges(myVertex);

        for(int i = 0; i < assignments.length; i++) {
           Integer robot = assignments[i][0] >= commPeers.size() ? null : assignments[i][0];
           Integer edge = assignments[i][1] >= outgoingEdges.size() ? null : assignments[i][1];

           if(robot != null) {
                AuthorityList msgAuthority = new AuthorityList(this.authorityList.getAuthorities());
                Message msg = new Message(msgAuthority, (edge == null) ? new LatticeEdge() : outgoingEdges.get(edge));
                commPeers.get(robot).enqueueMessage(msg);

                //System.out.println("Assigning robot (" +  robot + ") " + commPeers.get(robot).getRobotId() + " to " + ((edge == null) ? new LatticeEdge() : outgoingEdges.get(edge)));
           }
        }
    }

    public OrientedPoint[] retrieveAssignmentLocation() {
        OrientedPoint[] assignment;
        if(role == Role.root) {
            assignment = new OrientedPoint[] {null, self.getPosition()};
        } else if(role == Role.unassignedChild) {
            assignment = new OrientedPoint[] {null, null};
            
        } else {

            LatticeRobot parent = null;

            for(LatticeRobot robot : commPeers) {
                if(parentId.equals(robot.getRobotId())) {
                    parent = robot;
                }
            }

            if(parent == null) {
                System.out.println("I " +self.getRobotId() + "lost connection to " + parentId);
            }

            //Get transformation of parent that translates local coords to global positions
            RigidBodyTransformation parentLocalToGlobal = new RigidBodyTransformation(parent.getPosition());

            //Apply transformation of parent to assigned position to get global position of assigned target
            OrientedPoint assignedLocationGlobal = parentLocalToGlobal.apply(assignedEdge.getToPos());
        
            //Do we need to convert to local if this is just for positioning???
            assignment = new OrientedPoint[] {parent.getPosition(), assignedLocationGlobal};
        }
        return assignment;
    }

    /**
     * Determines whether the robot is a root robot within the authority tree
     * @return status as a root
     */
    public boolean isRoot() {
        return role == Role.root;
    }

    public boolean isAssigned() {
        return role == Role.assignedChild;
    }

    public void syncPeers(ArrayList<LatticeRobot> neighbors) {
        if(neighbors == null || neighbors.isEmpty()) {
            this.commPeers = new ArrayList<>();
        }
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
        if(outgoingEdges.isEmpty()) System.out.println("I DONT HAVE ANY OUTGOING EDGES");

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
            // First pass: calculate all costs normally
            for(int i = 0; i < commPeers.size(); i++) {
                for(int j = 0; j < outgoingEdges.size(); j++) {
                    OrientedPoint localPos = GLOBAL_TO_LOCAL.apply(commPeers.get(i).getPosition());
                    OrientedPoint destination = outgoingEdges.get(j).getEdgeTransformation().apply(new OrientedPoint(0,0,0));
                    cost[i][j] = localPos.distance(destination);
                }
            }

            // Second pass: set parent row and inverse edge column to INFINITY,
            // except the intersection of parent row and inverse edge column
            for(int i = 0; i < commPeers.size(); i++) {
                if(commPeers.get(i).getRobotId() == parentId) {
                    for(int j = 0; j < outgoingEdges.size(); j++) {
                        if(outgoingEdges.get(j).getEdgeTransformation().isInverse(assignedEdge.getEdgeTransformation())) {
                            // Set entire row and column to INFINITY except this intersection
                            for(int k = 0; k < outgoingEdges.size(); k++) {
                                if(k != j) cost[i][k] = HungarianMatrixUtils.INFINITY;
                            }
                            for(int k = 0; k < commPeers.size(); k++) {
                                if(k != i) cost[k][j] = HungarianMatrixUtils.INFINITY;
                            }
                            cost[i][j] = 0.0;
                        }
                    }
                    break;
                }
            }
        }

        if(commPeers.size() != outgoingEdges.size()) {
           cost = HungarianMatrixUtils.padToSquare(cost);
        }
    
        return cost;

    }
}
