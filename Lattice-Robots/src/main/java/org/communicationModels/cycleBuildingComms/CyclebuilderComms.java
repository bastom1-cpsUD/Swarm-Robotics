package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.communicationModels.Observation;
import org.communicationModels.TrustLevel;
import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.AttemptLaterMessage;
import org.communicationModels.cycleBuildingComms.Messages.ChainMemberList;
import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.communicationModels.cycleBuildingComms.Messages.PromotionMessage;
import org.communicationModels.cycleBuildingComms.Messages.RejectAssignmentMessage;
import org.communicationModels.cycleBuildingComms.Messages.StatusMessage;
import org.communicationModels.cycleBuildingComms.Messages.TargetClaimMessage;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.Role;
import org.graphs.voltage.VoltageGraph;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;
import org.utils.MathUtils;
import org.utils.Vec2;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.OutgoingMessageRecord;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;

    private static final boolean VERBOSE = true;
    // Two phases is one full time step: a claim survives the phase it arrived in and the
    // next, then goes. Long enough to tolerate one dropped beacon and to stay immune to
    // the order robots happen to activate in; short enough that a robot which went
    // unassigned and stopped emitting cannot keep provoking a phantom conflict.
    private static final int CLAIM_TTL_PHASES = 2;

    private HashMap<Integer, CycleStatus> completedCycles;
    private final VoltageGraph graph;

    // State-Data
    private int stableID;
    private int pendingChildID;
    private boolean hasBeenAssigned;
    private ChainMemberList chainMemberList;
    private CycleRole role;
    private ArrayList<Integer> unableToDoAssignmentIDs;

    private int assignedVertexID;
    private int assignedOutgoingEdgeID;
    private int originVertexID;
    private int originOutgoingEdgeID;

    //Simulation Support
    private GeometricCycleLatticeRobot self;

    // Time-Step Data
    private HashMap<Integer, Observation> phaseOneObservations;
    // Re-observed in phase two rather than reusing phase one's: a tick of motion happens
    // between the two, so phase one's relative poses are stale by the time an incoming
    // claim has to be converted into this robot's frame.
    private HashMap<Integer, Observation> phaseTwoObservations;
    private boolean waitThisTimeStep;

    // Logging / instrumentation support (see CommsSnapshot, org.logging package)
    private ArrayList<OutgoingMessageRecord> sentThisTick;
    // Visualization Support
    private Edge pendingChildEdge;

    public CyclebuilderComms(GeometricCycleLatticeRobot self, VoltageGraph graph) {
        this.graph = graph;
        this.trust = TrustLevel.Friendly;
        this.stableID = -1;
        this.pendingChildID = -1;
        this.pendingChildEdge = null;
        this.hasBeenAssigned = false;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.self = self;
        this.phaseOneObservations = new HashMap<>();
        this.phaseTwoObservations = new HashMap<>();
        this.incomingMessages = new ConcurrentLinkedQueue<>();
        this.unableToDoAssignmentIDs = new ArrayList<>();
        this.sentThisTick = new ArrayList<>();
        this.waitThisTimeStep = false;
        assignedVertexID = -1;
        assignedOutgoingEdgeID = -1;
        originVertexID = -1;
        originOutgoingEdgeID = -1;
    }

    /*
        ////////////////
        PHASE ONE LOGIC
        ////////////////
     */

    public HashMap<Integer, Observation> makeFirstPhaseObservations() {
        //Observe neighbors and their positions
        ArrayList<GeometricCycleLatticeRobot> neighbors = self.getNeighbors();
        phaseOneObservations.clear();
        if(neighbors == null || neighbors.isEmpty()) {
            phaseOneObservations = new HashMap<>();
            return phaseOneObservations;
        }

        RigidBodyTransformation globalToLocal = new RigidBodyTransformation(self.getPosition()).inverse();

        //Check if pendingChild has left communication range; if cyclebuilder, check if someone occupies my spot
        boolean childHasLeft = true;
        boolean assignmentOccupied = false;
        OrientedPoint myAssignment = getAssignedLocalPosition();

        for(GeometricCycleLatticeRobot neighbor : neighbors) {
            if(neighbor.getRobotId() == pendingChildID) {
                childHasLeft = false;
            }
            if(role == CycleRole.cycleBuilder) {
                OrientedPoint neighborPosition = globalToLocal.apply(neighbor.getPosition());
                //EDIT FOR PROPER ANGLE PRESERVATION (NEW ANGLE PRESERVATION EXISTS)
                // "My assignment is already occupied" fires on position coincidence
                // alone. neighborPosition now carries the neighbor's heading in this
                // robot's frame, so occupancy can require a pose match.
                if(MathUtils.isZero(myAssignment.distance(neighborPosition), MathUtils.EPSILON)) {
                    assignmentOccupied = true;
                }
            }
            // The neighbor's declared target, in its own frame -- see Observation's
            // three-argument constructor. Null for anything without a live assignment.
            Observation obs = new Observation(neighbor, globalToLocal);
            phaseOneObservations.put(neighbor.getRobotId(), obs);
        }

        if(childHasLeft && pendingChildID != -1) {
            log("-> child " + pendingChildID + " has left, clearing pendingChildID");
            self.getEdges().remove(pendingChildEdge);
            pendingChildEdge = null;
            pendingChildID = -1; // HAS MUTATED STATE
        }

        if(assignmentOccupied && role == CycleRole.cycleBuilder) {
            forwardRejectionToParent(false);
            resetToUnassigned();
            hasBeenAssigned = false;
            self.clearEdges();
            log("-> Assignment already occupied. Forwarding rejection to parent.");
            return phaseOneObservations;
        }

        return phaseOneObservations;
    }

    @Override
    public void processMessages() {
        processMessages(-1);
    }

    /**
     * Canonical message-processing implementation. Pops (at most) one message
     * from the priority queue, applies the role-specific state transition,
     * and returns a human-readable description of what happened — used both
     * for the VERBOSE console log and for HtmlLog-based tick logging.
     *
     * <p>{@code tick} is accepted purely for description text / caller
     * context; it does not affect behavior. Pass {@code -1} when no tick
     * context is available.</p>
     */
    public String processMessages(int tick) {
        if(incomingMessages.isEmpty()) {
            return "N/A (EMPTY)";
        }

        AbstractMessage peek = incomingMessages.peek();

        // Root/stable must always accept a closing PositioningMessage — don't gate on pendingChildID
        boolean bypassPendingGate = (role == CycleRole.root || role == CycleRole.stable)
                && (peek instanceof PositioningMessage || peek instanceof StatusMessage || peek instanceof RejectAssignmentMessage || peek instanceof AttemptLaterMessage);

        if (!bypassPendingGate && pendingChildID != -1 && peek.getSenderId() != pendingChildID) {
            incomingMessages.add(incomingMessages.poll());
            return "N/A (Waiting for pending child " + pendingChildID + ")";
        }
        if(!validateSenderIsNeighbor(peek.getSenderId())) {
            log("-> sender " + peek.getSenderId() + " is not a neighbor, discarding message");
            incomingMessages.poll();
            return "N/A (Discarded message from non-neighbor " + peek.getSenderId() + ")";
        }

        AbstractMessage next = incomingMessages.poll();
        log("Received " + next.getMessageType() + " from robot " + next.getSenderId());

        switch(role) {
            case unassigned:
                if(next instanceof PositioningMessage pm) {
                    if(hasBeenAssigned) {
                        log("Checking for formation-breaking assignment...");
                        if(!checkAssignmentForCurrentPosition(pm)) {
                            forwardRejectionUpstream(pm, false);
                            log("-> assignment REJECTED by " + pm.getSenderId());
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                        }
                    }

                    resetToCycleBuilder();
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    unableToDoAssignmentIDs.add(pm.getSenderId());
                    chainMemberList = pm.getChainList();
                    self.addEdge(new Edge(self.getRobotId(), chainMemberList.getSenderID()));
                    log("-> became cycleBuilder: edge id=" + getAssignedEdge().getId()
                            + " from vertex " + getAssignedEdge().getOrigin().getId()
                            + ", root=" + chainMemberList.getRootID()
                            + ", chain=" + chainMemberList.getIDList());
                    return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else if(next instanceof PromotionMessage pm) {
                    resetToRoot();
                    stableID = pm.getSenderId();
                    setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                    initializeEdgeMap();
                    log("-> promoted to root by robot " + stableID);
                    return "Promotion Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else {
                    log("-> unassigned robot received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case cycleBuilder:
                if (next instanceof StatusMessage sm && sm.isSuccessful()) {
                    log("-> child reported SUCCESS, forwarding upstream");
                    forwardSuccessUpstream();
                    return "Status Message from " + sm.getSenderId() + "(SUCCESS)";
                } else if (next instanceof StatusMessage sm && !sm.isSuccessful()) {
                    log("-> child reported FAILURE, forwarding upstream");
                    forwardFailureUpstream();
                    return "Status Message from " + sm.getSenderId() + "(FAILURE)";
                } else if(next instanceof RejectAssignmentMessage rm) {
                    pendingChildID = -1; // HAS MUTATED STATE
                    removeEdgeToChild(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    if(rm.isRetryable()) {
                        forwardRejectionToParent(true);
                        resetToUnassigned();
                        log("-> assignment is retryable, will attempt to reassign");
                    } else if(isChainRoot(rm.getSenderId())) {
                        // Never ban the chain's root. unableToDoAssignmentIDs gates only the
                        // cycle-closing test at the top of findBestNeighborForEdge -- the
                        // general candidate loop underneath it already excludes the root by
                        // id -- so banning the root buys nothing when the mismatch is real
                        // (a genuinely misplaced root fails that closure test on its own)
                        // and forfeits the close permanently when the mismatch was merely
                        // numerical. Nothing short of a reset clears the ban and this robot
                        // stays a cycleBuilder, so from that point on it could only ever
                        // pick a stranger for the closing edge and drive it onto the root.
                        log("-> assignment is NOT retryable, but sender is the chain root; not banning it, will retry the close");
                    } else {
                        unableToDoAssignmentIDs.add(rm.getSenderId()); // HAS MUTATED STATE (single-edge-scoped for cycleBuilder, unlike root's equivalent below)
                        log("-> assignment is NOT retryable, will not attempt to reassign");
                    }
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED, " + (rm.isRetryable() ? "RETRYABLE)" : " NOT RETRYABLE)");
                } else if(next instanceof PromotionMessage pm) {
                    incomingMessages.add(pm);
                    return "Promotion Message from " + pm.getSenderId() + "(DEFERRED - already cycleBuilder)";
                } else if(next instanceof PositioningMessage pm) {
                    // If the sender is our pending child, we can accept the message and process it normally.
                    if(pm.getSenderId() == pendingChildID) {
                        //Check if assignment is to current location
                        if(!checkAssignmentForCurrentPosition(pm)) {
                            forwardRejectionUpstream(pm, false);
                            log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                        }

                        //Check chain list to see who's list is larger, and if the incoming message has a larger list, we should accept it and send rejection to current parent with retryable
                        if(pm.getChainList().size() > chainMemberList.size()) {
                            forwardRejectionToParent(true);
                            resetToCycleBuilder();
                            setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                            setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                            unableToDoAssignmentIDs.add(pm.getSenderId());
                            chainMemberList = pm.getChainList();
                            log("-> Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming chain list is larger)");
                            return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming chain list is larger)";

                        //If incoming list is smaller, we should wait for other to complete our task as it will override
                        } else if(pm.getChainList().size() < chainMemberList.size()) {
                            //If a root sent the message, we should tell the root that we are rejecting the assignment and that it is retryable so that the root can reassign us
                            if(pm.getChainList().getRootID() == pm.getSenderId()) {
                                forwardRejectionUpstream(pm, true);
                            }
                            log("-> Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming chain list is smaller)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming chain list is smaller)";
                        } else {
                            //Accept message if root is smaller
                            if(pm.getChainList().getRootID() < chainMemberList.getRootID()) {
                                forwardRejectionToParent(true);
                                resetToCycleBuilder();
                                setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                                setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                                unableToDoAssignmentIDs.add(pm.getSenderId());
                                chainMemberList = pm.getChainList();
                                log("-> Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming root ID is smaller");
                                return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming root ID is smaller)";
                            
                            //Send rejection retryable if root is larger
                            } else {
                                forwardRejectionUpstream(pm, true);
                                return "Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming root ID is larger)";
                            }
                        }
                    }
                    // Already a cycleBuilder — a second parent is racing to assign us.
                    // Don't accept, don't reject: just defer. Re-queue untouched and
                    // let it be reconsidered on a future tick (e.g. once we free up
                    // via forwardSuccessUpstream/forwardFailureUpstream -> reset()).
                    incomingMessages.add(pm);
                    log("-> already assigned, deferring Positioning Message from " + pm.getSenderId());
                    return "Positioning Message from " + pm.getSenderId() + " (DEFERRED - already cycleBuilder)";
                } else {
                    log("-> cycleBuilder received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case root:
                if(next instanceof StatusMessage sm) {
                    resetToRoot();
                    if (sm.isSuccessful()) {
                        setCycleStatusOf(sm.getOriginOutgoingEdgeID(), CycleStatus.complete);
                        log("-> cycle on edge " + sm.getOriginOutgoingEdgeID() + " COMPLETED");
                        return "Status Message from " + sm.getSenderId() + "(SUCCESS)";
                    } else {
                        setCycleStatusOf(sm.getOriginOutgoingEdgeID(), CycleStatus.failed);
                        log("-> cycle on edge " + sm.getOriginOutgoingEdgeID() + " FAILED, moving on");
                        return "Status Message from " + sm.getSenderId() + "(FAILURE)";
                    }
                } else if(next instanceof RejectAssignmentMessage rm) {
                    pendingChildID = -1; // HAS MUTATED STATE
                    removeEdgeToChild(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    if(rm.isRetryable()) {
                        log("-> assignment is retryable, will attempt to reassign");
                    } else {
                        unableToDoAssignmentIDs.add(rm.getSenderId()); // HAS MUTATED STATE (mid-edge exclusion -- do NOT swap for resetToRoot() here, that would wipe the exclusion this same edge still needs on retry)
                        log("-> assignment is NOT retryable, will not attempt to reassign");
                    }
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED, " + (rm.isRetryable() ? "RETRYABLE)" : " NOT RETRYABLE)");
                } else if(next instanceof AttemptLaterMessage am) {
                    pendingChildID = -1;
                    CycleStatus previousStatus = setCycleStatusOf(am.getOriginOutgoingEdgeID(), CycleStatus.attempted);
                    if(previousStatus == CycleStatus.attempted) {
                        waitThisTimeStep = true;
                    }
                    log("-> " + am.getSenderId() + " asked to attempt edge " + am.getOriginOutgoingEdgeID() + " later");
                    return "Attempt Later Message from " + am.getSenderId() + "(DEFERRED)";
                } else if(next instanceof PositioningMessage pm) {
                    if(!checkAssignmentForCurrentPosition(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                        return "Positioning Message from " + pm.getSenderId() + " (REJECTED, WILL BREAK FORMATION)";

                        //If a root sent the message, check if the next edge's cycle is complete
                    } else if(pm.getChainList().getRootID() == pm.getSenderId()){
                        HalfEdge nextEdge = inferNextEdge(getAssignedEdgeFromMessage(pm));

                        if(completedCycles.get(nextEdge.getId()) == CycleStatus.complete) {
                            forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                            log("-> root received message from another root, but next edge's cycle is complete, forwarding SUCCESS upstream");
                            return "Positioning Message from " + pm.getSenderId() + " (ACCEPTED, CYCLE COMPLETE, forwarding SUCCESS upstream)";
                        } else if(!hasFailed()) {
                            log("-> root received message from another root, but next edge's cycle is NOT complete, forwarding ATTEMPT LATER upstream");
                            forwardAttemptLaterUpstream(pm);
                            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, next edge's cycle is NOT complete, forwarding ATTEMPT LATER upstream)";

                        } else {
                            forwardFailureUpstream(pm);
                            log("-> root received message from another root, but next edge's cycle is NOT complete, forwarding FAILURE upstream (NOT RETRYABLE)");
                            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, next edge's cycle is NOT complete, forwarding FAILURE upstream)";
                        }

                    } else {
                        log("-> reached root, reporting SUCCESS, forwarding upstream");
                        //If new robot appears in vicinity and claims part of a cycle, retry that cycle if it was earlier deemed failed
                        HalfEdge incomingEdge = retrieveEdgeFromGraph(pm.getAssignedOutgoingEdgeID());
                        int outgoingEquivelentID = incomingEdge.getTwin().getId();
                        if(completedCycles.get(outgoingEquivelentID) != CycleStatus.complete) {
                            setCycleStatusOf(outgoingEquivelentID, CycleStatus.unattempted);
                        }
                        self.addEdge(new Edge(pm.getRecipient(), pm.getSenderId()));
                    forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                    return "Positioning Message from " + pm.getSenderId() + " (ACCEPTED)";
                    }
                } else if(next instanceof PromotionMessage pm) {
                    //Neighbor promoted to stable, try again for cycle building
                    if(pm.hasReachedStable()) {
                        resetToRoot();
                        reattemptFailedCycles();
                        log("-> root received Promotion Message from " + pm.getSenderId() + ", neighbor has reached stable, will attempt to reassign");
                         return "Promotion Message from " + pm.getSenderId() + "(REACTIVATED, WILL ATTEMPT TO COMPLETE CYCLES)";
                    } else {
                        log("-> root received Promotion Message from " + pm.getSenderId() + ", neighbor has NOT reached stable, will NOT attempt to reassign");
                        return "Promotion Message from " + pm.getSenderId() + "(NOT REACTIVATED, WILL NOT ATTEMPT TO COMPLETE CYCLES)";
                    }
                } else {
                    log("-> root received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case stable:
                if(next instanceof PositioningMessage pm) {
                    if(!checkAssignmentForCurrentPosition(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                        return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                    } else {
                        log("-> reached stable, reporting SUCCESS, forwarding upstream");
                        forwardSuccessUpstream(pm.getChainList().getSenderID(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                        return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                    }
                } else {
                    log("-> stable received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
        }

        return "N/A (Unhandled message type)";
    }

    public void sendMessage(boolean alreadyInPosition) {
        sendMessage(alreadyInPosition, -1);
    }

    /**
     * Canonical broadcast/assignment implementation. Decides, based on role,
     * whether to assign a child, forward success/failure, promote neighbors,
     * or do nothing this tick — and returns a human-readable description of
     * the resulting action.
     *
     * <p>{@code tick} is accepted purely for description text / caller
     * context; it does not affect behavior. Pass {@code -1} when no tick
     * context is available.</p>
     */
    public String sendMessage(boolean alreadyInPosition, int tick) {
        switch (role) {
            case CycleRole.root: {
                if(pendingChildID != -1) {
                    return "N/A (Waiting for Status Update to complete)";
                } else if(hasFailed()) {
                    return "N/A (Ceased operations due to failure)";
                } else if(waitThisTimeStep) {
                    waitThisTimeStep = false;
                    return "N/A (Waiting for timestep)";
                }
                log("Sending Message...");

                // Find the first outgoing edge that doesn't have a completed cycle yet
                int targetEdgeID = determineNextCycleToComplete();
                

                if (targetEdgeID == -1) {
                    promoteAdjacentVerticesToRoots();
                    if(!hasFailed()) {
                        promoteSelfToStable();
                        log("-> all cycles completed, promoting self to stable");
                        return "Done (All cycles completed, promoted to stable)";
                    }
                    
                    return "Done (Not all cycles completed, but no valid neighbors for cycle, ceasing operations)";
                }

                HalfEdge targetEdge = retrieveEdgeFromGraph(targetEdgeID);

                // 2. Cycle does not exist. Build it!
                GeometricCycleLatticeRobot childToBuild = findBestNeighborForEdge(targetEdge);
                ChainMemberList chainList = new ChainMemberList(self.getRobotId());
                if (childToBuild != null) {
                    PositioningMessage pm = new PositioningMessage(self.getRobotId(), childToBuild.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), chainList);
                    send(childToBuild, pm);
                    pendingChildEdge = new Edge(self.getRobotId(), childToBuild.getRobotId());
                    self.addEdge(pendingChildEdge);
                    pendingChildID = childToBuild.getRobotId(); // Wait for status
                } else {
                    log("Ran out of options for building cycle on edge " + targetEdgeID + ", failing edge and moving on");
                    setCycleStatusOf(targetEdgeID, CycleStatus.failed);
                    resetToRoot();
                    return "Failed (No valid neighbors for cycle, ceasing operations)";
                }
                if(VERBOSE) {
                    log("Message sent to " + childToBuild.getRobotId());
                }
                return "Assigned position to robot " + childToBuild.getRobotId()
                        + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
            }
            case CycleRole.cycleBuilder: {
                if(pendingChildID != -1) {
                    return "N/A (Waiting for Status Update to complete)";
                }
                if(!alreadyInPosition) {
                    return "N/A (Not in position to broadcast)";
                }
                if(VERBOSE) {
                    log("Sending Message...");
                }

                HalfEdge targetEdge = inferNextEdge();

                GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge);

                if(child == null) {
                    log("-> NO candidate found, forwarding FAILURE upstream");
                    forwardFailureUpstream();
                    return "Reporting Failure (No candidate found)";
                } 
                //ADD BACK HERE
                log("-> assigning robot " + child.getRobotId() + " to edge " + targetEdge.getId());
                ChainMemberList childChainList = new ChainMemberList(chainMemberList, self.getRobotId());
                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), originVertexID, originOutgoingEdgeID, childChainList);
                send(child, pm);
                pendingChildEdge = new Edge(self.getRobotId(), child.getRobotId());
                self.addEdge(pendingChildEdge);
                pendingChildID = child.getRobotId();
                return "Assigned position to robot " + child.getRobotId() + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
            }
            case CycleRole.unassigned:
                return "N/A (Unassigned robot do not broadcast)";
            default:
                return "N/A (Unhandled)";
        }
    }

    /*
        ////////////////////////
        ASSIGNMENT CONTENTION
        ////////////////////////
     */

    /**
     * Re-observes neighbours for phase two.
     *
     * <p>Uses the same frame convention as {@link #makeFirstPhaseObservations()} --
     * this robot's <em>actual</em> pose. That matters: an incoming claim is converted
     * into this robot's frame by composing it with the observation of its sender, so the
     * observation and this robot's own assigned local pose must live in one frame or the
     * comparison is meaningless.
     *
     * <p>Separate observations rather than reusing phase one's, because a tick of motion
     * happens in between and every relative pose has moved.
     */
    public HashMap<Integer, Observation> makeSecondPhaseObservations() {
        ArrayList<GeometricCycleLatticeRobot> neighbors = self.getNeighbors();
        phaseTwoObservations.clear();
        if(neighbors == null || neighbors.isEmpty()) {
            return new HashMap<>();
        }

        RigidBodyTransformation globalToLocal = new RigidBodyTransformation(self.getPosition()).inverse();
        for(GeometricCycleLatticeRobot neighbor : neighbors) {
            phaseTwoObservations.put(neighbor.getRobotId(), new Observation(neighbor, globalToLocal));
        }
        return phaseTwoObservations;
    }

    /**
     * This robot's assigned pose expressed in its own frame -- the form it can declare
     * to neighbors without either side needing a shared origin. Null whenever there is
     * nothing to declare: {@code root} and {@code stable} are static anchors (their
     * "target" is wherever they already are, and a neighbor heading onto one of them is
     * already caught by the occupancy check in
     * {@link #makeFirstPhaseObservations()}), and an unassigned robot has no target.
     *
     * <p>Deliberately distinct from {@link #getAssignedLocalPosition()}, which
     * substitutes the origin when there is no live assignment. That substitution is
     * harmless for a self-query but would be a false claim on this robot's own position
     * if handed to a neighbor.
     *
     * @return the assigned pose in this robot's local frame, or null if there is none
     */
    public OrientedPoint getClaimedLocalTarget() {
        if (role != CycleRole.cycleBuilder) {
            return null;
        }

        OrientedPoint globalTarget = getAssignedGlobalPosition();
        if (globalTarget == null) {
            return null;
        }

        return new RigidBodyTransformation(self.getPosition()).inverse().apply(globalTarget);
    }

    /**
     * Re-expresses a neighbour's claim in this robot's own frame.
     * @param senderInMyFrame where this robot observes the sender
     * @param claimInSenderFrame the target the sender declared, in the sender's own frame
     * @return the sender's declared target, in this robot's frame
     */
    static OrientedPoint claimInMyFrame(OrientedPoint senderInMyFrame, OrientedPoint claimInSenderFrame) {
        return new RigidBodyTransformation(senderInMyFrame).apply(claimInSenderFrame);
    }

    /**
     * Ages every claim heard from a neighbour by one phase and drops any that have
     * expired. The host calls this once at the start of each phase.
     */
    public void expireStaleClaims() {
        ageClaims(CLAIM_TTL_PHASES);
    }

    /**
     * Broadcasts this robot's target to every neighbour in range, if it has one to
     * declare. Called once per tick regardless of phase: robots activate staggered and
     * asynchronously, so one robot's phase two routinely lands during a neighbour's phase
     * one, and emitting only in phase two would silently drop claims to that drift.
     *
     * @return a description of what was emitted, or null if there was nothing to declare
     */
    public String broadcastTargetClaim() {
        OrientedPoint claim = getClaimedLocalTarget();
        if (claim == null) {
            return null;
        }
        HalfEdge assignedEdge = getAssignedEdge();
        if (assignedEdge == null) {
            log("-> no assigned edge, cannot broadcast claim");
            return "N/A (No assigned edge, cannot broadcast claim)";
        }
        int targetRoleID = getCurrentRole().getId();

        broadcast(new TargetClaimMessage(self.getRobotId(), claim, targetRoleID));
        return "Broadcast target claim " + claim;
    }

    /** True if (aParked, aID) outranks (bParked, bID): possession first, then lower id. */
    private boolean outranks(boolean aParked, int aID, boolean bParked, int bID) {
        if (aParked != bParked) return aParked;
        return aID < bID;
    }
    /**
     * Whether a claim declares its sender already standing on the spot it claims.
     *
     * <p>Must be handed the claim in its <em>sender's own</em> frame -- the form it is
     * transmitted in -- not the version composed into this robot's frame. A sender standing
     * on its target declares exactly the origin, and the origin is fixed by a change of
     * frame, so possession is the one thing in a claim that survives transport with no
     * observation error on it. Recovering it after the composition instead, from the gap
     * between the sender's observed pose and its claim in this frame, would measure a tick
     * of drift -- order gamma -- against a tolerance of {@link MathUtils#EPSILON}, and so
     * report "still moving" for a robot that has been parked for minutes.
     *
     * <p>Position only, deliberately: a robot part-way through its final rotation is
     * standing on the spot and occupying it, whatever its heading.
     *
     * @param claimInOwnFrame a claim, in the frame of the robot that made it
     * @return true if the claiming robot is already at the pose it claims
     */
    private boolean isParked(OrientedPoint claimInOwnFrame) {
        return MathUtils.isZero(claimInOwnFrame.distance(new OrientedPoint(0, 0, 0)));
    }

    /**
     * Detects two robots converging on the same lattice spot -- the failure mode when two
     * roots independently hand out the same position -- and resolves it before either has
     * arrived.
     *
     * <p>Each neighbour broadcasts its target (see {@link #broadcastTargetClaim()}); this
     * compares those declarations against this robot's own. The predicate is symmetric:
     * once both robots hold each other's claim they compare the same pair of points and
     * reach the same verdict, so the tie-break below picks exactly one winner.
     * Inferring intent from observed motion would not be symmetric -- each robot would
     * test the other's trajectory against a <em>different</em> target, so both could
     * yield, or neither.
     *
     * <p>That tie-break is a total order over {@code (parked, id)} -- see
     * {@link #outranks(boolean, int, boolean, int)}. Possession comes first because id
     * alone has no notion of who is already there: an arrived robot holding the higher id
     * would tear itself down, rejecting to its parent and dropping its subtree, in favour
     * of an interloper that had not got there yet. Possession is read off a claim
     * <em>before</em> it is transformed into this frame, via {@link #isParked}, for the
     * reason given there.
     *
     * <p>Symmetry is a property of the steady state, not of any single phase. In the first
     * phase two after an assignment lands, whichever robot activates earlier may not have
     * heard its rival yet and will keep the spot; the rival yields on its own turn, or one
     * time step later if neither had emitted in time. Contention therefore resolves within
     * two time steps rather than one. Closing that window entirely would need a global
     * barrier between emission and evaluation, which is precisely what the asynchronous
     * scheduler rules out -- but the gap only ever delays a yield, never produces two.
     *
     * <p>This is early warning, not the guarantee. A claim that was lost
     * or arrived too late leaves the occupancy check in
     * {@link #makeFirstPhaseObservations()} as the backstop, so the worst case degrades to
     * the previous behaviour rather than to an overlap.
     *
     * @return a description of the contention for the tick log, or null if there was none
     */
    public String detectAssignmentContention(HashMap<Integer, Observation> phaseObservations) {
        OrientedPoint myClaim = getClaimedLocalTarget();
        if (myClaim == null) {
            return null;
        }

        boolean iAmParked = isParked(myClaim);

        // The maximum possible distance a robot can move in one tick/phase; radius of the "contention zone" around a lattice spot. If two robots individual
        // claims fall within this distance, they can be consider equal if their claimed role IDS are the same.
        double gamma = self.getMaxSpeed() / GeometricCycleLatticeRobot.TICK_RATE;

        // Resolve against the strongest rival rather than the first one found: iteration
        // order over the claim map must not decide who keeps the assignment. "Strongest"
        // is parked-before-moving, then lowest id.
        int bestRivalID = -1;
        boolean bestRivalParked = false;
        for (ClaimEntry entry : incomingClaims.values()) {
            int senderID = entry.claim().getSenderId();

            // The claim arrives in its sender's frame; the observation locates that
            // sender in ours. Composing them gives the sender's target in our frame:
            //   (T_self^-1 * T_sender) * (T_sender^-1 * T_target) = T_self^-1 * T_target
            // Both halves are local -- one communicated, one sensed -- so no shared
            // origin is assumed anywhere.
            Observation obs = phaseObservations.get(senderID);
            if (obs == null) {
                continue;
            }

            OrientedPoint claimInSenderFrame = entry.claim().getClaimInSenderFrame();
            OrientedPoint theirClaim = claimInMyFrame(obs.getLocalPosition(), claimInSenderFrame);
            
            if (!MathUtils.isPointInBoundingCircle(myClaim, gamma, theirClaim)
                    || entry.claim().getTargetRoleID() != getCurrentRole().getId()) {
                continue;
            }

            // Read off the claim as it arrived, before the frame change above -- see
            // isParked. The rival is a genuine contender for this spot either way; this
            // only decides which of us gives it up.
            boolean rivalParked = isParked(claimInSenderFrame);

            if (bestRivalID == -1 || outranks(rivalParked, senderID, bestRivalParked, bestRivalID)) {
                bestRivalID = senderID;
                bestRivalParked = rivalParked;
            }
        }

        if (bestRivalID == -1) {
            return null;
        }

        if (!outranks(bestRivalParked, bestRivalID, iAmParked, self.getRobotId())) {
            boolean heldByPossession = iAmParked && !bestRivalParked;
            log("-> Contention with robot " + bestRivalID + " over my assignment. "
                    + (heldByPossession ? "I am already parked on it" : "I hold the lower id")
                    + "; keeping it.");
            return "Assignment contention with robot " + bestRivalID
                    + (heldByPossession ? " (KEPT, parked)" : " (KEPT, lower id)");
        }

        // Not retryable: this robot genuinely cannot take this spot, so the parent should
        // record that and offer the edge to a different candidate rather than re-offering
        // it here. forwardRejectionToParent must run first -- resetToUnassigned clears the
        // chainMemberList it reads the parent from.
        forwardRejectionToParent(false);
        resetToUnassigned();
        hasBeenAssigned = false;
        self.clearEdges();
        boolean lostToPossession = bestRivalParked && !iAmParked;
        log("-> Contention with robot " + bestRivalID + " over my assignment. "
                + (lostToPossession ? "It is already parked there" : "I hold the higher id")
                + "; yielding and forwarding rejection to parent.");
        return "Assignment contention with robot " + bestRivalID
                + (lostToPossession ? " (YIELDED, rival parked)" : " (YIELDED, higher id)");
    }

    //MESSAGE-PROCESSING UTIL
    private void forwardSuccessUpstream() {
        forwardSuccessUpstream(chainMemberList.getSenderID(), originVertexID, assignedOutgoingEdgeID);
        resetToUnassigned();
    }

    private void forwardSuccessUpstream(int parentId, int originVertexID, int originEdgeID) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        StatusMessage sm = new StatusMessage(self.getRobotId(), parentId, true, originVertexID, originEdgeID);
        send(parent, sm);
    }

    private void forwardFailureUpstream() {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID);
        send(parent, sm);
        resetToUnassigned();
    }

    private void forwardFailureUpstream(PositioningMessage pm) {
        GeometricCycleLatticeRobot parent = getNeighborByID(pm.getSenderId());
        int originVertexID = pm.getOriginVertexID();
        int originOutgoingEdgeID = pm.getOriginOutgoingEdgeID();

        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID);
        send(parent, sm);
    }

    private void forwardRejectionUpstream(PositioningMessage pm, boolean isRetryable) {
        RejectAssignmentMessage rm = new RejectAssignmentMessage(pm.getRecipient(), pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), isRetryable);
        GeometricCycleLatticeRobot robot = getNeighborByID(pm.getSenderId());
        if(!(robot == null)) send(robot, rm);
    }

    private void forwardRejectionToParent(boolean isRetryable) {
        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        RejectAssignmentMessage rm = new RejectAssignmentMessage(self.getRobotId(), parent.getRobotId(), originVertexID, originOutgoingEdgeID, isRetryable);
        send(parent, rm);
    }

    private void forwardAttemptLaterUpstream(PositioningMessage pm) {
        AttemptLaterMessage am = new AttemptLaterMessage(pm.getRecipient(), pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
        GeometricCycleLatticeRobot robot = getNeighborByID(pm.getSenderId());
        send(robot,am);
    }

    //ROOT-RELATED UTIL
    private int determineNextCycleToComplete() {
        //Prioritize unattempted values first
        for (Entry<Integer, CycleStatus> entry : completedCycles.entrySet()) {
            CycleStatus status = entry.getValue();
            if(status == CycleStatus.unattempted) {
                return entry.getKey();
            }
        }

        //Next, iterate through attempted cycles
        for(Entry<Integer, CycleStatus> entry : completedCycles.entrySet()) {
            CycleStatus status = entry.getValue();
            if(status == CycleStatus.attempted) {
                return entry.getKey();
            }
        }

        return -1;
    }

    private void initializeEdgeMap() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);
        for(HalfEdge edge : edges) {
            completedCycles.put(getEdgeIDof(edge), CycleStatus.unattempted);
        }
    }

    private void promoteAdjacentVerticesToRoots() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);

        for(HalfEdge edge : edges) {
            if(completedCycles.get(getEdgeIDof(edge)) == CycleStatus.complete) {
                GeometricCycleLatticeRobot neighbor = findBestNeighborForEdge(edge);
                PromotionMessage pm = new PromotionMessage(self.getRobotId(), neighbor.getRobotId(), getVertexIDof(edge), getEdgeIDof(edge), !hasFailed());
                send(neighbor, pm);
                log("-> promoting neighbor on edge " + getEdgeIDof(edge) + " to root");
            }
            
        }
    }

    //ASSIGNMENT-RELATED UTIL

    private boolean validateSenderIsNeighbor(int senderID) {
        for(Observation obs : phaseOneObservations.values()) {
            if(obs.getId() == senderID) {
                return true;
            }
        }
        return false;
    }

    private GeometricCycleLatticeRobot getNeighborByID(int robotID) {
        for(GeometricCycleLatticeRobot neighbor : self.getNeighbors()) {
            if(neighbor.getRobotId() == robotID) {
                return neighbor;
            }
        }
        return null;
    }

    /**
     * Whether the given robot is the root of this robot's current chain. Guards the
     * empty-chain case, which a cycleBuilder holds briefly between resetToCycleBuilder()
     * and the caller's own chainMemberList assignment, and which an unassigned robot
     * holds permanently.
     *
     * @param robotID the id to test
     * @return true if the chain is non-empty and this id is its root
     */
    private boolean isChainRoot(int robotID) {
        return !chainMemberList.isEmpty() && chainMemberList.getRootID() == robotID;
    }

    private GeometricCycleLatticeRobot findBestNeighborForEdge(HalfEdge targetEdge) {
        OrientedPoint targetLocal  = getTargetInLocalCoordinates(targetEdge);
            log("Beginning decision process");

        int rootID = chainMemberList.isEmpty() ? -1 : chainMemberList.getRootID();

        // Root is only a candidate if it's not banned/parent, and check it separately from the rest
        Observation rootObs = phaseOneObservations.get(rootID);
        if (rootObs != null && !unableToDoAssignmentIDs.contains(rootID)) {
            //EDIT FOR PROPER ANGLE PRESERVATION (NEW ANGLE PRESERVATION EXISTS)
            // Cycle closure on position alone. Both operands now carry meaningful
            // headings -- targetLocal from the edge's voltage, rootObs from the
            // observation -- so this can become a pose match rather than a point match.
            double rootDistance = targetLocal.distance(rootObs.getLocalPosition());
            if (MathUtils.isZero(rootDistance, MathUtils.EPSILON)) {
                log("-> root is exactly at target position, closing cycle");
                return getNeighborByID(rootID);
            }
        }

        // Root excluded from here on — never lets a near-but-not-exact root suppress a real candidate
        ArrayList<Observation> validObservations = new ArrayList<>();
        ArrayList<Observation> priorityObservations = new ArrayList<>();

        for (Observation obs : phaseOneObservations.values()) {
            int robotID = obs.getId();
            if (robotID != rootID && !chainMemberList.isInList(robotID) && !unableToDoAssignmentIDs.contains(robotID)) {
                // A neighbor already sitting exactly at the target position must win outright,
                // same as the root case above, and before the wedge test runs: that test's
                // angle math is ill-conditioned right at zero distance (the target->candidate
                // vector degenerates toward (0,0), so its angle is numerically meaningless),
                // and this is exactly the case -- an already-placed neighbor from an adjoining
                // face/edge -- that must be picked to avoid placing a duplicate robot on top
                // of it.
                //EDIT FOR PROPER ANGLE PRESERVATION (NEW ANGLE PRESERVATION EXISTS)
                // Position-only match, as above: a neighbor sitting on the right spot
                // with the wrong heading is currently reused as though it fit.
                if (MathUtils.isZero(targetLocal.distance(obs.getLocalPosition()), MathUtils.EPSILON)) {
                    log("-> " + robotID + " is exactly at target position, closing onto existing neighbor");
                    return getNeighborByID(robotID);
                }

                if(observationIsWithinFormingFace(obs, targetEdge)) {
                    priorityObservations.add(obs);
                    log("Added robot " + obs.getId() + " to the priority selection");
                }
                validObservations.add(obs);
            }
        }

        int bestNeighborID = -1;
        double smallestDistance = Double.MAX_VALUE;

        if(!priorityObservations.isEmpty()) {
            validObservations = priorityObservations;
        }

        for (Observation obs : validObservations) {
            double distance = targetLocal.distance(obs.getLocalPosition());
                if (distance < smallestDistance) {
                    smallestDistance = distance;
                    bestNeighborID = obs.getId();
                }
        }

        return getNeighborByID(bestNeighborID); // null if truly none — genuine dead end
    }

    private boolean observationIsWithinFormingFace(Observation obs, HalfEdge targetEdge) {
        HalfEdge previousEdge = getAssignedEdge();
        //If robot is without assignment, there is no previous edge and thus no formation restrictions
        if(previousEdge == null) {
            return false;
        }
        //Point of self
        OrientedPoint p1 = new OrientedPoint(0,0,0);
        //Point of target
        OrientedPoint p2 = getTargetInLocalCoordinates(targetEdge);
        //Point of candidate
        OrientedPoint p3 = new OrientedPoint(obs.getLocalPosition());
        //Point of future target
        //EDIT FOR PROPER ANGLE PRESERVATION (NEW ANGLE PRESERVATION EXISTS)
        OrientedPoint p4 = Vec2.of(p2).plus(Vec2.of(getTargetInLocalCoordinates(inferNextEdge(targetEdge)))).asOrientedPoint();
        //Point of parent
        OrientedPoint p5 = getTargetInLocalCoordinates(getAssignedEdge().getTwin());

        //Candidate orientation follows target and future target
        int cycleOrientation1 = MathUtils.threePointClockwiseCounterClockwiseTest(p1, p2, p4);
        //Candidate orientation follows self and target
        int cycleOrientation2 = MathUtils.threePointClockwiseCounterClockwiseTest(p1, p2, p3);
        //Candidate orientation follows parent and self
        int cycleOrientation3 = MathUtils.threePointClockwiseCounterClockwiseTest(p5, p1, p3);

        return Integer.signum(cycleOrientation1) == Integer.signum(cycleOrientation2) && Integer.signum(cycleOrientation1) == Integer.signum(cycleOrientation3);
    }

    private OrientedPoint getTargetInLocalCoordinates(HalfEdge edge) {
        return edge.getVoltage().asPose();
    }

    private boolean checkAssignmentForCurrentPosition(PositioningMessage pm) {
        OrientedPoint localAssignment = getAssignedLocalPosition(pm);

        if(localAssignment == null) return false;

        log("Assigned local position is: " + localAssignment);

        // Self, expressed in self's own frame, is exactly the origin.
        OrientedPoint positionInLocal = new OrientedPoint(0, 0, 0);

        //EDIT FOR PROPER ANGLE PRESERVATION (NEW ANGLE PRESERVATION EXISTS)
        return MathUtils.approxEquals(localAssignment.x, positionInLocal.getX(), MathUtils.REASSIGNMENT_POSITION_EPSILON)
                && MathUtils.approxEquals(localAssignment.y, positionInLocal.getY(), MathUtils.REASSIGNMENT_POSITION_EPSILON)
                && MathUtils.isZero(MathUtils.angleDifference(localAssignment.getOrientation(), positionInLocal.getOrientation()), MathUtils.REASSIGNMENT_ANGLE_EPSILON);
    }

    /**
     * EDGE AND ROLE UTIL
     */
    private HalfEdge inferNextEdge() {
        return inferNextEdge(getAssignedEdge());
    }

    private HalfEdge inferNextEdge(HalfEdge assignedEdge) {
        if (assignedEdge == null) return null;

        // next(h) is already "cross to twin(h), then rotate" -- Edmonds' rule
        // -- resolved once by VoltageGraphBuilder.build() and stored on the
        // half-edge itself. See DCEL-Implementation-Plan.md sec 2.2 / 4.
        return graph.getNext(assignedEdge);
    }

    private Role getCurrentRole() {
        HalfEdge assignedEdge = getAssignedEdge();

        if(assignedEdge == null) {
            return graph.getPrimaryRole();
        }

        return assignedEdge.getTarget();
    }

    public HalfEdge getAssignedEdge() {
        if(assignedVertexID == -1 || assignedOutgoingEdgeID == -1) {
            return null;
        }
        return retrieveEdgeFromGraph(assignedOutgoingEdgeID);
    }

    public HalfEdge getAssignedEdgeFromMessage(PositioningMessage pm) {
        if(pm.getAssignedVertexID() == -1 || pm.getAssignedOutgoingEdgeID() == -1) {
            return null;
        }
        return retrieveEdgeFromGraph(pm.getAssignedOutgoingEdgeID());
    }

    public HalfEdge getOriginEdge() {
        if(originVertexID == -1 || originOutgoingEdgeID == -1) {
            return null;
        }

        return retrieveEdgeFromGraph(originOutgoingEdgeID);
    }

    private HalfEdge retrieveEdgeFromGraph(int edgeID) {
        return graph.getHalfEdgeById(edgeID);
    }

    public void setAssignedEdge(int vertexID, int edgeID) {
        assignedVertexID = vertexID;
        assignedOutgoingEdgeID = edgeID;
    }

    public void setOriginEdge(int vertexID, int edgeID) {
        originVertexID = vertexID;
        originOutgoingEdgeID = edgeID;
    }

    public int getVertexIDof(HalfEdge e) {
        return e.getOrigin().getId();
    }

    public int getEdgeIDof(HalfEdge e) {
        return e.getId();
    }

    private void removeEdgeToChild(int childID) {
        if (childID == -1) return;
        // Reference removal, not toId match: only undoes the specific
        // speculative edge for this rejected offer, leaving any other
        // (already-valid) edge to the same robot ID untouched.
        if (pendingChildEdge != null && pendingChildEdge.getToId() == childID) {
            self.getEdges().remove(pendingChildEdge);
        }
        pendingChildEdge = null;
    }

    public void promoteToPrimaryRoot() {
        role = CycleRole.root;
        initializeEdgeMap();
    }

    private void promoteSelfToStable() {
        role = CycleRole.stable;
        this.pendingChildID = -1;
        this.pendingChildEdge = null;
    }

    //ACCESSORS, MUTATORS, AND UTIL ----------------------------------------------------
    public CycleRole   getRole()         { return role; }
    public boolean     isRoot()          { return role == CycleRole.root; }
    public boolean     isStable()        { return role == CycleRole.stable; }
    public boolean     isCycleBuilder()  { return role == CycleRole.cycleBuilder; }
    public boolean     isUnassigned()    { return role == CycleRole.unassigned; }

    public TrustLevel  getTrustLevel()             { return trust; }
    public void        setTrustLevel(TrustLevel t) { this.trust = t; }

    public CycleStatus getCycleStatusOf(int outgoingEdgeID) { return completedCycles.get(outgoingEdgeID); }
    public CycleStatus setCycleStatusOf(int outgoingEdgeID, CycleStatus status) { return completedCycles.put(outgoingEdgeID, status);}
    public void reattemptFailedCycles() {
        for(Entry<Integer, CycleStatus> entry : completedCycles.entrySet()) {
            if(entry.getValue() == CycleStatus.failed) {
                setCycleStatusOf(entry.getKey(), CycleStatus.unattempted);
            }
        }
    }
    public boolean hasFailed() { 
        int numOfCycleStillWorkingOn = 0;
        int numOfFailedCycles = 0;
        for(Entry<Integer, CycleStatus> entry : completedCycles.entrySet()) {
            if(entry.getValue() == CycleStatus.attempted || entry.getValue() == CycleStatus.unattempted) {
                numOfCycleStillWorkingOn++;
            } else if (entry.getValue() == CycleStatus.failed){
                numOfFailedCycles++;
            }
        }
        return numOfCycleStillWorkingOn == 0 && numOfFailedCycles != 0;
     }

    public OrientedPoint getAssignedGlobalPosition() {
        if (role == CycleRole.root || role == CycleRole.stable) {
            return self.getPosition();
        }

        HalfEdge assignedEdge = getAssignedEdge();

        if (assignedEdge == null) return null;

        GeometricCycleLatticeRobot parent = getNeighborByID(chainMemberList.getSenderID());
        if (parent == null) return null;

        return globalTransformOf(parent.getPosition(), assignedEdge.getVoltage())
                .asPose();
    }

    private OrientedPoint getAssignedLocalPosition() {
        OrientedPoint globalPos = getAssignedGlobalPosition();
        if(globalPos == null) {
            return new OrientedPoint(0,0,0);
        }
        return new RigidBodyTransformation(self.getPosition()).inverse().apply(globalPos);
    }

    private OrientedPoint getAssignedLocalPosition(PositioningMessage pm) {
        HalfEdge assignedEdge = retrieveEdgeFromGraph(pm.getAssignedOutgoingEdgeID());

        if(assignedEdge == null) return null;

        GeometricCycleLatticeRobot parent = getNeighborByID(pm.getSenderId());

        if(parent == null) {
            return null;
        }

        RigidBodyTransformation assignedGlobalTransform = globalTransformOf(parent.getPosition(), assignedEdge.getVoltage());

        return new RigidBodyTransformation(self.getPosition()).inverse()
                .compose(assignedGlobalTransform)
                .asPose();
    }

    /**
     * Composes a base pose with a transform relative to it (e.g. an edge's voltage)
     * into the resulting global transform, accumulating rotation along the way.
     *
     * <p>This originally existed to route around
     * {@code RigidBodyTransformation.apply()} discarding the orientation of whatever
     * point it was given. That is fixed -- {@code apply()} now composes the pose's own
     * heading -- so the workaround is no longer load-bearing. Composing transforms and
     * reading the result once with {@code asPose()} is still the clearer spelling than
     * chaining point-applications, so the method stays.
     */
    private static RigidBodyTransformation globalTransformOf(OrientedPoint basePose, RigidBodyTransformation relative) {
        return new RigidBodyTransformation(basePose).compose(relative);
    }

    public void reset() {
        this.pendingChildID = -1;
        this.pendingChildEdge = null;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        unableToDoAssignmentIDs.clear();

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;

        resetObservations();
    }

    public void resetObservations() {
        this.phaseOneObservations.clear();
        this.phaseTwoObservations.clear();
    }

    /**
     * Clean slate for "unassigned": stableID, pendingChildID, chainMemberList,
     * unableToDoAssignmentIDs, and the assigned/origin edge references are all
     * cleared, and role is set to unassigned. hasBeenAssigned is the one
     * state-data field deliberately left untouched. Differs from the existing
     * reset() above only in that it also clears stableID.
     */
    public void resetToUnassigned() {
        this.stableID = -1;
        this.pendingChildID = -1;
        this.pendingChildEdge = null;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.unassigned;
        this.unableToDoAssignmentIDs.clear();

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Clean slate for "cycleBuilder": stableID, pendingChildID, chainMemberList,
     * unableToDoAssignmentIDs, and both the assigned and origin edge references
     * are cleared, and role is set to cycleBuilder. Meant to be immediately
     * followed by setAssignedEdge/setOriginEdge and the caller's own
     * chainMemberList/unableToDoAssignmentIDs setup, the same way the
     * unassigned -> cycleBuilder PositioningMessage handling does today.
     */
    public void resetToCycleBuilder() {
        this.stableID = -1;
        this.pendingChildID = -1;
        this.pendingChildEdge = null;
        this.hasBeenAssigned = true;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.cycleBuilder;
        this.unableToDoAssignmentIDs.clear();

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Clean slate for a root moving on to its next outgoing edge: pendingChildID,
     * chainMemberList, unableToDoAssignmentIDs, and the origin edge reference are
     * cleared, role is (re-)set to root, and hasFailed is forced back to false.
     * stableID and the assigned edge are deliberately left untouched -- they're
     * the root's own identity/position in the graph, not per-edge state. Same
     * reasoning excludes completedCycles entirely: it's the record of progress
     * this reset exists to avoid corrupting.
     */
    public void resetToRoot() {
        this.pendingChildID = -1;
        this.pendingChildEdge = null;
        this.chainMemberList = new ChainMemberList();
        this.role = CycleRole.root;
        this.unableToDoAssignmentIDs.clear();

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    //LOGGING / SNAPSHOT SUPPORT ---------------------------------------------

    /**
     * Clears the record of messages sent since the last call. Must be called
     * once at the very start of each robot activation (before processMessages
     * / sendMessage run for that tick) so that {@link #sentThisTick()}
     * reflects only messages sent during the current tick.
     */
    public void beginTick() {
        sentThisTick.clear();
    }

    /**
     * Returns the messages this comms system has enqueued to other robots
     * since the last {@link #beginTick()} call.
     */
    public List<OutgoingMessageRecord> sentThisTick() {
        return List.copyOf(sentThisTick);
    }

    /**
     * Captures a read-only, defensively-copied snapshot of this comms
     * system's current state. Cheap enough to call twice per tick (once
     * before dispatch, once after) for before/after diffing.
     */
    public CommsSnapshot snapshot() {
        return new CommsSnapshot(
                role,
                trust,
                hasFailed(),
                pendingChildID,
                stableID,
                chainMemberList,
                getAssignedEdge(),
                getOriginEdge(),
                Map.copyOf(completedCycles),
                snapshotQueueInOrder(),
                Map.copyOf(phaseOneObservations),
                List.copyOf(unableToDoAssignmentIDs)
        );
    }

    private List<AbstractMessage> snapshotQueueInOrder() {
        List<AbstractMessage> copy = new ArrayList<>(incomingMessages);
        return copy;
    }

    /**
     * Enqueues a message to a recipient and records it for logging. Every
     * outgoing send in this class should go through here rather than calling
     * {@code recipient.enqueueMessage(...)} directly, so
     * {@link #sentThisTick()} stays a complete and accurate audit trail.
     */
    private void send(GeometricCycleLatticeRobot recipient, AbstractMessage msg) {
        recipient.enqueueMessage(msg);
        sentThisTick.add(new OutgoingMessageRecord(
                recipient.getRobotId(), msg.getMessageType(), String.valueOf(msg)));
    }

    /**
     * Emits a claim to every neighbour in range.
     *
     * <p>Logged as a <em>single</em> outgoing record, not one per neighbour: a radio
     * broadcast is one transmission that everyone in range happens to hear, and counting
     * it N times would overstate the protocol's communication cost by the neighbour
     * degree.
     */
    private void broadcast(TargetClaimMessage msg) {
        for (GeometricCycleLatticeRobot neighbor : self.getNeighbors()) {
            neighbor.receiveClaim(msg);
        }
        sentThisTick.add(new OutgoingMessageRecord(
                AbstractMessage.BROADCAST, msg.getMessageType(), String.valueOf(msg)));
    }

    private void log(String msg) {
        if (VERBOSE) {
            System.out.println("[Robot " + self.getRobotId() + " | " + role + "] " + msg);
        }
    }

}