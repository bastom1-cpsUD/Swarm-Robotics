package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.communicationModels.Observation;
import org.communicationModels.TrustLevel;
import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.AttemptLaterMessage;
import org.communicationModels.cycleBuildingComms.Messages.CertificateLostMessage;
import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
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
import org.robots.AvoidanceState;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;
import org.utils.MathUtils;
import org.utils.Vec2;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.OutgoingMessageRecord;

public class CyclebuilderComms extends CommunicationSystem {
    private TrustLevel trust;

    private static final boolean VERBOSE = true;
    // A claim survives the tick it arrived in and the
    // next, then goes.
    private static final int CLAIM_TTL_PHASES = 2;

    private HashMap<Integer, CycleStatus> completedCycles;
    private final VoltageGraph graph;

    // State-Data
    private int stableID;
    private boolean hasBeenAssigned;
    private VoltageCertificate certificate;

    /**
     * The faces this robot currently owes an offer or a response on -- see
     * {@link FaceObligation}.
     *
     * <p><strong>Capped at one entry for now</strong> ({@link #MAX_CONCURRENT_OBLIGATIONS}),
     * which makes this an exact stand-in for the {@code pendingChildID} /
     * {@code pendingChildEdge} pair it replaces: a robot still serves one face at a time,
     * still gates its inbox while waiting on a child, still retries the same edge. The cap
     * is what makes this migration falsifiable -- lifting it is a separate, later change,
     * so if the simulation misbehaves after this one it is the representation that is
     * wrong, not the new concurrency.
     *
     * <p>Outstanding is per-edge and means {@link FaceObligation#isUnfulfilled()} -- a null
     * child. A filled slot leaves the robot free, because a response will arrive, route
     * through that tuple, and clear it.
     */
    private final FaceObligationSet obligations = new FaceObligationSet();

    /**
     * The robot that assigned this one its lattice site, and the frame every derived
     * target is expressed against. {@code -1} when unassigned.
     *
     * <p>Introduced when {@link VoltageCertificate} stopped carrying a chain member list.
     * The parent used to be read back out of that list as its last element, which coupled
     * two unrelated things: <em>who anchors my pose</em> and <em>which walk am I relaying
     * right now</em>. Those are the same robot today only because a robot serves one face
     * at a time. Once it serves several, the certificate at hand belongs to whichever walk
     * arrived most recently, and re-deriving the anchor from it would move the robot to a
     * different lattice site every time a new face came through -- with no compile error
     * and no test failure, just robots drifting onto wrong sites.
     *
     * <p>So this is set exactly once, on the transition into {@code cycleBuilder}, and
     * never from a relayed message.
     */
    private int anchorParentID;
    private CycleRole role;

    private int assignedVertexID;
    private int assignedOutgoingEdgeID;
    private int originVertexID;
    private int originOutgoingEdgeID;

    //Simulation Support
    private GeometricCycleLatticeRobot self;

    // Time-Step Data
    private HashMap<Integer, Observation> observations;
    private boolean waitThisTimeStep;

    /**
     * Tick-rate collision policy. Held by composition: it decides where to aim when
     * something is in the way, and knows nothing about the cycle-building protocol.
     */
    private final AvoidancePolicy policy;

    // Logging / instrumentation support (see CommsSnapshot, org.logging package)
    private ArrayList<OutgoingMessageRecord> sentThisTick;

    /**
     * How many faces this robot may serve at once.
     *
     * <p>One, deliberately. The tuple representation supports any number, but holding the
     * cap here keeps the migration to it separable from the concurrency it enables: with
     * the cap in place this class behaves exactly as the {@code pendingChildID} version
     * did, so a regression after this change is a fault in the representation rather than
     * in newly-parallel face building. Raising it is a later, isolated edit.
     */
    private static final int MAX_CONCURRENT_OBLIGATIONS = 1;

    public CyclebuilderComms(GeometricCycleLatticeRobot self, VoltageGraph graph) {
        this.graph = graph;
        this.trust = TrustLevel.Friendly;
        this.stableID = -1;
        this.hasBeenAssigned = false;
        this.certificate = null;
        this.anchorParentID = -1;
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.self = self;
        this.observations = new HashMap<>();
        this.policy = new AvoidancePolicy(self);
        this.incomingMessages = new ConcurrentLinkedQueue<>();
        this.sentThisTick = new ArrayList<>();
        this.waitThisTimeStep = false;
        assignedVertexID = -1;
        assignedOutgoingEdgeID = -1;
        originVertexID = -1;
        originOutgoingEdgeID = -1;
    }

    public HashMap<Integer, Observation> makeObservations() {
        //Observe neighbors and their positions
        ArrayList<GeometricCycleLatticeRobot> neighbors = self.getNeighbors();
        observations.clear();
        if(neighbors == null || neighbors.isEmpty()) {
            observations = new HashMap<>();
            return observations;
        }

        RigidBodyTransformation globalToLocal = new RigidBodyTransformation(self.getPosition()).inverse();

        //Check if a recorded child has left communication range; if cyclebuilder, check if someone occupies my spot
        Set<Integer> childrenStillPresent = new HashSet<>();
        boolean assignmentOccupied = false;
        OrientedPoint myAssignment = getAssignedLocalPosition();

        for(GeometricCycleLatticeRobot neighbor : neighbors) {
            childrenStillPresent.add(neighbor.getRobotId());
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
            observations.put(neighbor.getRobotId(), obs);
        }

        collectDepartedChildren(childrenStillPresent);

        if(assignmentOccupied && role == CycleRole.cycleBuilder) {
            forwardRejectionToParent(false);
            resetToUnassigned();
            hasBeenAssigned = false;
            self.clearEdges();
            log("-> Assignment already occupied. Forwarding rejection to parent.");
            return observations;
        }

        return observations;
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

        // Root/stable must always accept a closing PositioningMessage — don't gate on a pending child
        boolean bypassPendingGate = (role == CycleRole.root || role == CycleRole.stable)
                && (peek instanceof PositioningMessage || peek instanceof StatusMessage
                    || peek instanceof RejectAssignmentMessage || peek instanceof AttemptLaterMessage
                    || peek instanceof CertificateLostMessage);

        // At a cap of one this is the old pendingChildID gate exactly. It survives the
        // migration unchanged on purpose: lifting it belongs with lifting the cap, since a
        // robot serving several faces has to answer for the ones it is not waiting on.
        int pendingChild = pendingChildID();
        if (!bypassPendingGate && pendingChild != -1 && peek.getSenderId() != pendingChild) {
            incomingMessages.add(incomingMessages.poll());
            return "N/A (Waiting for pending child " + pendingChild + ")";
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
                    // The parent used to be banned here, into a robot-scoped exclusion list.
                    // It is now excluded structurally by anchorParentID in
                    // findBestNeighborForEdge, which is both narrower and impossible to
                    // forget -- and the ban could not live on an obligation anyway, since the
                    // one this robot will owe is not created until it knows which edge that
                    // is, on arrival.
                    certificate = pm.getCertificate();
                    // Set here and nowhere else: this is the only transition that decides
                    // which robot this one is anchored to. See the anchorParentID javadoc.
                    anchorParentID = pm.getSenderId();
                    self.addEdge(new Edge(self.getRobotId(), anchorParentID));
                    log("-> became cycleBuilder: edge id=" + getAssignedEdge().getId()
                            + " from vertex " + getAssignedEdge().getOrigin().getId()
                            + ", parent=" + anchorParentID
                            + ", cert=" + certificate);
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
                } else if(next instanceof CertificateLostMessage cm) {
                    // The walk broke below this robot. Its own tuple stays intact -- the
                    // topology it describes is still correct, only the message in flight was
                    // lost -- and the report keeps travelling to the initiator, the only
                    // robot that can mint a replacement certificate.
                    log("-> certificate lost below " + cm.getSenderId() + ", forwarding upstream");
                    forwardCertificateLostUpstream(cm);
                    return "Certificate Lost Message from " + cm.getSenderId() + "(FORWARDED)";
                } else if(next instanceof RejectAssignmentMessage rm) {
                    // Release, not remove: the child declined, but this robot still owes the
                    // same edge to somebody, and the certificate came back on the rejection
                    // so there is something to re-offer with.
                    releaseObligationOfChild(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    if(rm.isRetryable()) {
                        forwardRejectionToParent(true);
                        resetToUnassigned();
                        log("-> assignment is retryable, will attempt to reassign");
                    } else if(isChainRoot(rm.getSenderId())) {
                        // Never ban the chain's root. The ban list gates only the
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
                        banOnCurrentFace(rm.getSenderId());
                        log("-> assignment is NOT retryable, will not attempt to reassign");
                    }
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED, " + (rm.isRetryable() ? "RETRYABLE)" : " NOT RETRYABLE)");
                } else if(next instanceof PromotionMessage pm) {
                    incomingMessages.add(pm);
                    return "Promotion Message from " + pm.getSenderId() + "(DEFERRED - already cycleBuilder)";
                } else if(next instanceof PositioningMessage pm) {
                    // If the sender is our pending child, we can accept the message and process it normally.
                    if(pm.getSenderId() == pendingChildID()) {
                        //Check if assignment is to current location
                        if(!checkAssignmentForCurrentPosition(pm)) {
                            forwardRejectionUpstream(pm, false);
                            log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                        }

                        //Check chain list to see who's list is larger, and if the incoming message has a larger list, we should accept it and send rejection to current parent with retryable
                        if(pm.getCertificate().getHops() > certificate.getHops()) {
                            forwardRejectionToParent(true);
                            resetToCycleBuilder();
                            setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                            setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                            certificate = pm.getCertificate();
                            anchorParentID = pm.getSenderId();
                            log("-> Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming chain list is larger)");
                            return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED, incoming chain list is larger)";

                        //If incoming list is smaller, we should wait for other to complete our task as it will override
                        } else if(pm.getCertificate().getHops() < certificate.getHops()) {
                            //If a root sent the message, we should tell the root that we are rejecting the assignment and that it is retryable so that the root can reassign us
                            if(pm.getCertificate().getInitiatorID() == pm.getSenderId()) {
                                forwardRejectionUpstream(pm, true);
                            }
                            log("-> Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming chain list is smaller)");
                            return "Positioning Message from " + pm.getSenderId() + "(REJECTED, incoming chain list is smaller)";
                        } else {
                            //Accept message if root is smaller
                            if(pm.getCertificate().getInitiatorID() < certificate.getInitiatorID()) {
                                forwardRejectionToParent(true);
                                resetToCycleBuilder();
                                setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
                                setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
                                certificate = pm.getCertificate();
                                anchorParentID = pm.getSenderId();
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
                } else if(next instanceof CertificateLostMessage cm) {
                    // This root IS the initiator, so the report stops here. The corner is
                    // marked attempted rather than unattempted so determineNextCycleToComplete
                    // tries the other edges first and comes back to it, instead of spinning
                    // on a corner that may be short of candidates.
                    releaseObligationOfChild(cm.getSenderId());
                    setCycleStatusOf(cm.getOriginOutgoingEdgeID(), CycleStatus.attempted);
                    log("-> certificate for edge " + cm.getOriginOutgoingEdgeID()
                            + " was lost below " + cm.getSenderId() + "; will relaunch it later");
                    return "Certificate Lost Message from " + cm.getSenderId() + "(WILL RELAUNCH)";
                } else if(next instanceof RejectAssignmentMessage rm) {
                    releaseObligationOfChild(rm.getSenderId());
                    log("-> assignment REJECTED by " + rm.getSenderId());
                    if(rm.isRetryable()) {
                        log("-> assignment is retryable, will attempt to reassign");
                    } else {
                        // Mid-edge exclusion. Scoped to this face's own obligation, so it
                        // survives a retry of the same edge and disappears with the face --
                        // which is what the old comment here had to warn about by hand.
                        banOnCurrentFace(rm.getSenderId());
                        log("-> assignment is NOT retryable, will not attempt to reassign");
                    }
                    return "Reject Assignment Message from " + rm.getSenderId() + "(REJECTED, " + (rm.isRetryable() ? "RETRYABLE)" : " NOT RETRYABLE)");
                } else if(next instanceof AttemptLaterMessage am) {
                    releaseObligationOfChild(am.getSenderId());
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
                    } else if(pm.getCertificate().getInitiatorID() == pm.getSenderId()){
                        HalfEdge nextEdge = inferNextEdge(getAssignedEdgeFromMessage(pm));

                        if(completedCycles.get(nextEdge.getId()) == CycleStatus.complete) {
                            forwardSuccessUpstream(pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), pm.getCertificate());
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
                    forwardSuccessUpstream(pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), pm.getCertificate());
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
                        forwardSuccessUpstream(pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), pm.getCertificate());
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
                if(pendingChildID() != -1) {
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
                // The obligation for this face, created on first attempt and reused on every
                // retry of the same edge -- which is what carries the ban list across a
                // rejection instead of it having to be cleared by hand.
                FaceObligation rootObligation = obligationFor(targetEdge);

                GeometricCycleLatticeRobot childToBuild = findBestNeighborForEdge(targetEdge, rootObligation);
                VoltageCertificate certificate = new VoltageCertificate(self.getRobotId());
                if (childToBuild != null) {
                    PositioningMessage pm = new PositioningMessage(self.getRobotId(), childToBuild.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), certificate);
                    send(childToBuild, pm);
                    Edge drawn = new Edge(self.getRobotId(), childToBuild.getRobotId());
                    self.addEdge(drawn);
                    rootObligation.fulfil(childToBuild.getRobotId(), drawn); // Wait for status
                } else {
                    log("Ran out of options for building cycle on edge " + targetEdgeID + ", failing edge and moving on");
                    setCycleStatusOf(targetEdgeID, CycleStatus.failed);
                    obligations.remove(rootObligation);
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
                if(pendingChildID() != -1) {
                    return "N/A (Waiting for Status Update to complete)";
                }
                if(!alreadyInPosition) {
                    return "N/A (Not in position to broadcast)";
                }
                if(VERBOSE) {
                    log("Sending Message...");
                }

                HalfEdge targetEdge = inferNextEdge();

                // The obligation this robot owes onward, keyed on that outgoing edge. Created
                // here rather than on acceptance because this is the first moment the robot
                // knows it can act -- it is in position, so the edge it owes is decidable.
                FaceObligation obligation = obligationFor(targetEdge);

                GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge, obligation);

                if(child == null) {
                    log("-> NO candidate found, forwarding FAILURE upstream");
                    forwardFailureUpstream();
                    return "Reporting Failure (No candidate found)";
                }
                //ADD BACK HERE
                log("-> assigning robot " + child.getRobotId() + " to edge " + targetEdge.getId());

                // Extend the certificate by the hop INTO this robot -- T(parent -> me), not
                // T(me -> child). The child has not moved yet, so measuring outbound would
                // sample a pose that does not exist. Both endpoints of the inbound hop are
                // settled: the parent relayed earlier, and the alreadyInPosition gate above
                // means this robot is on its own site now.
                RigidBodyTransformation inboundHop = measureInboundHop(anchorParentID);
                if(inboundHop == null) {
                    // The parent is one lattice edge away and parked, so it is meant to be
                    // permanently in range (FaceClosureTest pins every edge below COMM_RANGE).
                    // Losing sight of it means the certificate cannot be extended honestly,
                    // and a certificate extended with a guessed hop certifies nothing. Hold
                    // the obligation and retry next tick rather than relaying a lie; Phase 5
                    // turns a persistent loss into a report the initiator can act on.
                    log("-> parent " + anchorParentID + " not observable, cannot extend certificate this tick");
                    return "N/A (Parent " + anchorParentID + " unobservable, certificate not extended)";
                }
                VoltageCertificate childCertificate = certificate.extend(inboundHop);
                PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(), getVertexIDof(targetEdge), getEdgeIDof(targetEdge), originVertexID, originOutgoingEdgeID, childCertificate);
                send(child, pm);
                Edge drawn = new Edge(self.getRobotId(), child.getRobotId());
                self.addEdge(drawn);
                obligation.fulfil(child.getRobotId(), drawn);
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
     * Ages every claim heard from a neighbour by one tick and drops any that have
     * expired. The host calls this once at the start of each tick.
     */
    public void expireStaleClaims() {
        ageClaims(CLAIM_TTL_PHASES);
    }

    /**
     * Broadcasts this robot's target to every neighbour in range, if it has one to
     * declare.
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

        // The stand-aside directive rides this beacon rather than travelling as a message
        // of its own -- see TargetClaimMessage.getStandAsideId().
        broadcast(new TargetClaimMessage(self.getRobotId(), claim, targetRoleID,
                                         policy.standAsideRequest()));
        return "Broadcast target claim " + claim;
    }

    /*
        ////////////////////////
        COLLISION AVOIDANCE
        ////////////////////////
     */

    /**
     * Updates the collision policy's moving/stationary picture of the neighbourhood from
     * the observations {@link #makeObservations()} just took. Call once per tick, right
     * after observing.
     */
    public void observeMotion() {
        policy.observeMotion(observations);
    }

    /**
     * Turns the robot's true target into the pose it should actually drive at this tick —
     * unchanged when the path is clear, a detour waypoint when a stationary body is in the
     * way, or the current pose when yielding.
     *
     * <p>Deliberately kept out of {@link #getAssignedGlobalPosition()}. That method is
     * what the protocol reads: the arrival gate, the occupancy check, a child's derived
     * target and — critically — {@link #getClaimedLocalTarget()} all go through it. A
     * detour waypoint must never reach any of them, or the contention system would start
     * arbitrating over transient dodges instead of lattice spots. The waypoint's only
     * consumer is {@code GeometricCycleLatticeRobot.move(double)}.
     */
    public OrientedPoint planMotionWaypoint(OrientedPoint trueTarget) {
        return policy.planWaypoint(trueTarget, getParentPoseOrNull());
    }

    /**
     * Where this robot's parent is, or null when there is no link to preserve.
     *
     * <p>Guarded twice over: only a cycleBuilder has a parent worth staying near, the
     * chain list can be empty (and {@code getSenderID()} calls {@code getLast()} on it),
     * and the parent may have drifted out of observation range.
     */
    private OrientedPoint getParentPoseOrNull() {
        if (role != CycleRole.cycleBuilder || certificate == null) {
            return null;
        }
        GeometricCycleLatticeRobot parent = getNeighborByID(anchorParentID);
        return parent == null ? null : parent.getPosition();
    }

    /** Drops any latched detour. Called from every role reset. */
    public void clearDetour() {
        policy.clearDetour();
    }

    /**
     * Acts on the collision layer's top liveness rung: a cycleBuilder that has made no
     * progress for long enough gives its assignment up, so the parent can offer the spot
     * to somebody who can physically reach it.
     *
     * <p>This is the collision layer degrading into the existing protocol rather than
     * inventing an escape of its own. The teardown is the same sequence
     * {@link #detectAssignmentContention(HashMap)} uses when it yields, in the same order:
     * the rejection has to go out <em>before</em> {@link #resetToUnassigned()}, which
     * clears the chain list the parent is read from.
     *
     * <p>Retryability is decided by {@code AvoidancePolicy.giveUpIsPermanent()} — see
     * there for why getting it backwards would introduce a livelock into the protocol.
     *
     * @return a description for the tick log, or null if nothing was given up
     */
    public String applyLivenessGiveUp() {
        if (role != CycleRole.cycleBuilder || !policy.giveUpRequested()) {
            return null;
        }

        boolean permanent = policy.giveUpIsPermanent();
        int blocker = policy.blockingObstacleId();

        forwardRejectionToParent(!permanent);
        resetToUnassigned();
        hasBeenAssigned = false;
        self.clearEdges();

        log("-> gave up assignment: no progress past robot " + blocker
                + (permanent ? " (permanent obstruction, not retryable)" : " (transient, retryable)"));
        return "Gave up assignment, blocked by robot " + blocker
                + (permanent ? " (NOT RETRYABLE)" : " (RETRYABLE)");
    }

    /**
     * Honours a stand-aside directive addressed to this robot, if there is one and this
     * robot is in a position to act on it.
     *
     * <p>Runs before {@link #processMessages(int)} so that an assignment arriving on the
     * same tick cancels the evasion rather than racing it.
     *
     * <p>Directives arrive on the claim beacon, not the protocol queue, which is what
     * makes this work at all. The protocol queue is gated behind {@code pendingChildID},
     * pops one message a tick, and — decisively — the {@code unassigned} branch of
     * {@code processMessages} discards anything that is not a positioning or promotion
     * message, so a directive routed there would simply be thrown away by the only role
     * that can honour it.
     */
    public void consumeStandAside() {
        if (!canHonourStandAside()) {
            return;
        }

        TargetClaimMessage nearest = null;
        Observation nearestObs = null;
        double nearestDistance = Double.POSITIVE_INFINITY;
        OrientedPoint origin = new OrientedPoint(0, 0, 0);

        for (ClaimEntry entry : incomingClaims.values()) {
            TargetClaimMessage claim = entry.claim();
            if (claim.getStandAsideId() != self.getRobotId()) {
                continue;
            }
            Observation obs = observations.get(claim.getSenderId());
            if (obs == null) {
                continue;   // asked by someone this robot cannot currently see
            }
            double distance = obs.getLocalPosition().distance(origin);
            // Nearest asker wins -- it is the one most likely to actually collide. Ties
            // broken on lower id so iteration order cannot decide.
            if (distance < nearestDistance
                    || (distance == nearestDistance && claim.getSenderId() < nearest.getSenderId())) {
                nearestDistance = distance;
                nearest = claim;
                nearestObs = obs;
            }
        }

        if (nearest == null) {
            return;
        }

        OrientedPoint corridorStart = nearestObs.getLocalPosition();
        OrientedPoint corridorEnd = claimInMyFrame(corridorStart, nearest.getClaimInSenderFrame());
        policy.onStandAside(corridorStart, corridorEnd, observations, nearest.getSenderId());
        log("-> standing aside for robot " + nearest.getSenderId());
    }

    /**
     * Whether this robot may act on a stand-aside directive.
     *
     * <p>Narrower than "not anchored", deliberately. A root or a stable is an anchor. A
     * cycleBuilder still en route has a live assignment of its own and must not be pushed
     * off it. And a cycleBuilder that has <em>arrived</em> must not move at all: its
     * children derive their targets from its pose, so shifting it would drag the whole
     * subtree. That leaves exactly one role, which is why the requester never needs to
     * sense who it is asking — everyone else silently drops the request.
     */
    private boolean canHonourStandAside() {
        return role == CycleRole.unassigned && getAssignedGlobalPosition() == null;
    }

    /** Ages any stand-aside commitment by one tick. Called once per tick, for every role. */
    public void ageEvasion() {
        policy.ageEvasion();
    }

    /**
     * Where this robot is stepping aside to, or null.
     *
     * <p>Deliberately <em>not</em> folded into {@link #getAssignedGlobalPosition()}. That
     * method is the protocol's notion of a target; an evasion is a motion waypoint and
     * nothing more. Keeping them apart is what guarantees an evading robot broadcasts no
     * claim and never enters contention.
     */
    public OrientedPoint getEvasionGlobalPosition() {
        return policy.evasionGlobalTarget();
    }

    /** Abandons any stand-aside commitment. */
    public void cancelEvasion() {
        policy.cancelEvasion();
    }

    /** What the collision policy decided this tick. Overlay and tick log only. */
    public AvoidanceState getPlannedAvoidanceState() {
        return policy.state();
    }

    /** The neighbour currently in the way, or -1. Overlay and tick log only. */
    public int getBlockingObstacleId() {
        return policy.blockingObstacleId();
    }

    /** True if (aParked, aID) outranks (bParked, bID): possession first, then lower id. */
    protected boolean outranks(boolean aParked, int aID, boolean bParked, int bID) {
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
     * <p>Symmetry is a property of the steady state, not of any single tick. In the first
     * tick after an assignment lands, whichever robot activates earlier may not have
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
    /** Resolves contention against the observations {@link #makeObservations()} just took. */
    public String detectAssignmentContention() {
        return detectAssignmentContention(observations);
    }

    public String detectAssignmentContention(HashMap<Integer, Observation> phaseObservations) {
        OrientedPoint myClaim = getClaimedLocalTarget();
        if (myClaim == null) {
            return null;
        }

        boolean iAmParked = isParked(myClaim);

        // The maximum possible distance a robot can move in one tick; radius of the "contention zone" around a lattice spot. If two robots individual
        // claims fall within this distance, they can be consider equal if their claimed role IDS are the same.
        //
        // Shared with the collision layer via tickTravel() rather than recomputed here:
        // both layers reason about one tick of travel, and two independent expressions of
        // the same quantity would eventually drift apart.
        double gamma = GeometricCycleLatticeRobot.tickTravel();

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
        // certificate it reads the parent from.
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
        forwardSuccessUpstream(anchorParentID, originVertexID, assignedOutgoingEdgeID, certificate);
        resetToUnassigned();
    }

    /**
     * Reports success to a parent, handing the certificate back with it.
     *
     * <p>The certificate rides the return path rather than being kept: whoever made the
     * offer gets back the exact certificate it sent, so it can act on the outcome without
     * ever having held a copy while the walk was in flight.
     */
    private void forwardSuccessUpstream(int parentId, int originVertexID, int originEdgeID, VoltageCertificate returning) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        StatusMessage sm = new StatusMessage(self.getRobotId(), parentId, true, originVertexID, originEdgeID, returning);
        send(parent, sm);
    }

    /**
     * Reports failure to the parent and stands down.
     *
     * <p>Guarded on both the empty chain list ({@code getSenderID} calls {@code getLast})
     * and a parent that is no longer observable. Both are reachable exactly when this is
     * called: a robot reports failure when it is stuck, and being stuck is correlated with
     * a parent having drifted out of range. The stand-down still happens either way — an
     * undeliverable report is no reason to stay a wedged cycleBuilder.
     */
    private void forwardFailureUpstream() {
        GeometricCycleLatticeRobot parent = certificate == null
                ? null : getNeighborByID(anchorParentID);
        if (parent != null) {
            StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID, certificate);
            send(parent, sm);
        } else {
            log("-> cannot report failure: parent unreachable, standing down anyway");
        }
        resetToUnassigned();
    }

    private void forwardFailureUpstream(PositioningMessage pm) {
        GeometricCycleLatticeRobot parent = getNeighborByID(pm.getSenderId());
        int originVertexID = pm.getOriginVertexID();
        int originOutgoingEdgeID = pm.getOriginOutgoingEdgeID();

        StatusMessage sm = new StatusMessage(self.getRobotId(), parent.getRobotId(), false, originVertexID, originOutgoingEdgeID, pm.getCertificate());
        send(parent, sm);
    }

    private void forwardRejectionUpstream(PositioningMessage pm, boolean isRetryable) {
        RejectAssignmentMessage rm = new RejectAssignmentMessage(pm.getRecipient(), pm.getSenderId(), pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), isRetryable, pm.getCertificate());
        GeometricCycleLatticeRobot robot = getNeighborByID(pm.getSenderId());
        if(!(robot == null)) send(robot, rm);
    }

    /**
     * Rejects this robot's assignment back to its parent.
     *
     * <p>Same guards, same reason, as {@link #forwardFailureUpstream()}. Callers always
     * follow this with their own teardown, so an undeliverable rejection costs only the
     * parent's knowledge of it — the parent independently notices the child leaving via
     * the {@code childHasLeft} check in {@link #makeObservations()}.
     */
    private void forwardRejectionToParent(boolean isRetryable) {
        GeometricCycleLatticeRobot parent = certificate == null
                ? null : getNeighborByID(anchorParentID);
        if (parent == null) {
            log("-> cannot reject to parent: unreachable");
            return;
        }
        RejectAssignmentMessage rm = new RejectAssignmentMessage(self.getRobotId(), parent.getRobotId(), originVertexID, originOutgoingEdgeID, isRetryable, certificate);
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
                GeometricCycleLatticeRobot neighbor = findBestNeighborForEdge(edge, null);
                PromotionMessage pm = new PromotionMessage(self.getRobotId(), neighbor.getRobotId(), getVertexIDof(edge), getEdgeIDof(edge), !hasFailed());
                send(neighbor, pm);
                log("-> promoting neighbor on edge " + getEdgeIDof(edge) + " to root");
            }
            
        }
    }

    //ASSIGNMENT-RELATED UTIL

    private boolean validateSenderIsNeighbor(int senderID) {
        for(Observation obs : observations.values()) {
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
     * and the caller's own certificate assignment, and which an unassigned robot
     * holds permanently.
     *
     * @param robotID the id to test
     * @return true if the chain is non-empty and this id is its root
     */
    private boolean isChainRoot(int robotID) {
        return certificate != null && certificate.getInitiatorID() == robotID;
    }

    /**
     * Picks a robot to take {@code targetEdge}, excluding anyone banned on that face.
     *
     * @param obligation the face this offer belongs to, whose ban list scopes the
     *                   exclusions. Null means no exclusions -- the
     *                   {@code promoteAdjacentVerticesToRoots} path, which today gets that
     *                   behaviour from an empty chain rather than by asking for it.
     */
    private GeometricCycleLatticeRobot findBestNeighborForEdge(HalfEdge targetEdge, FaceObligation obligation) {
        OrientedPoint targetLocal  = getTargetInLocalCoordinates(targetEdge);
            log("Beginning decision process");

        int rootID = certificate == null ? -1 : certificate.getInitiatorID();

        // Root is only a candidate if it's not banned/parent, and check it separately from
        // the rest. The rootID != anchorParentID test replaces the blanket ban the accept
        // path used to place on the parent: a direct child of the initiator must not "close"
        // onto the robot that just assigned it, which would be a one-hop cycle.
        Observation rootObs = observations.get(rootID);
        if (rootObs != null && rootID != anchorParentID && !isBannedOn(obligation, rootID)) {
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

        for (Observation obs : observations.values()) {
            int robotID = obs.getId();
            // Walk-membership exclusion, narrowed. This used to read
            // !certificate.isInList(robotID), excluding every robot already on the walk;
            // the certificate no longer carries that roster, so what remains is the
            // initiator (rootID, above) and the immediate parent, tested explicitly below.
            //
            // What is no longer excluded is deeper ancestors: a builder can offer the next
            // edge to its own grandparent. That is self-correcting by design -- an ancestor
            // is parked a full lattice site away from the offered target, so
            // checkAssignmentForCurrentPosition fails by ~50 units against a tolerance of
            // 0.1, it rejects NON-retryable, that lands it on this face's ban list, and the
            // certificate rides back on the rejection so the offer moves straight to the
            // next candidate. One wasted round trip per ancestor per face, and it cannot
            // repeat.
            //
            // INTERIM HAZARD, until the pending-child gate goes: that gate rotates a
            // PositioningMessage unread whenever the recipient is waiting on a child, so a
            // mid-chain ancestor never reaches the position check above and never sends that
            // rejection. Offerer, ancestor and the robot between them then wait on each other
            // with no timeout. Requires the grandparent to be within COMM_RANGE (75), which
            // means a 4-cycle of 50-unit edges -- OctagonSquare, HexagonSquareTriangle,
            // DodecagonHexagonSquare, ElongatedTriangular. SnubSquare and Square are 70 (99
            // apart, out of range); on triangular faces the grandparent IS the initiator,
            // excluded just below. Deleting the gate closes this.
            if (robotID != rootID && robotID != anchorParentID && !isBannedOn(obligation, robotID)) {
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

    /**
     * Drops every obligation whose child is no longer observable, reporting each loss to
     * the parent that is waiting on it.
     *
     * <p>Removal, not release. Today's code clears the child slot and re-offers on the
     * spot, which works only because this robot still holds a copy of the certificate in a
     * field. In the tuple design the certificate lives in messages: this robot forwarded
     * the only copy it had, the child left with it, and nothing upstream can reconstruct
     * it. Retrying locally is not a weaker option, it is unimplementable -- there is
     * nothing left to offer. Only the initiator can mint a fresh certificate, so the loss
     * has to travel.
     *
     * <p>This is the one place Phase 4 is not behaviour-preserving: a departed child now
     * escalates instead of being replaced locally, so a face takes longer to rebuild and an
     * extra message type appears in the log.
     *
     * @param stillPresent ids of every currently observable neighbour
     */
    private void collectDepartedChildren(Set<Integer> stillPresent) {
        List<FaceObligation> departed = new ArrayList<>();
        for (FaceObligation obligation : obligations.asList()) {
            Integer childId = obligation.getChildId();
            if (childId != null && !stillPresent.contains(childId)) {
                departed.add(obligation);
            }
        }

        for (FaceObligation obligation : departed) {
            log("-> child " + obligation.getChildId() + " has left with the certificate for edge "
                    + obligation.getEdgeId() + "; reporting the loss upstream");
            undrawChildEdge(obligation.getChildEdge());
            obligations.remove(obligation);
            reportCertificateLost(obligation);
        }
    }

    /**
     * Tells the robot waiting on this obligation that its certificate is gone.
     *
     * <p>A root that loses a child is itself the initiator, so there is nobody to tell: it
     * marks its own corner {@code attempted} and picks the face up again on a later pass.
     */
    private void reportCertificateLost(FaceObligation obligation) {
        if (role == CycleRole.root) {
            setCycleStatusOf(obligation.getEdgeId(), CycleStatus.attempted);
            return;
        }

        GeometricCycleLatticeRobot parent = getNeighborByID(obligation.getParentId());
        if (parent == null) {
            // Structurally unreachable: a parent is parked one lattice edge away, and every
            // lattice's edge length is pinned below COMM_RANGE by FaceClosureTest. Logged
            // rather than ignored, because if it ever fires that guard has been broken and
            // the corner it names will hang.
            log("-> cannot report lost certificate: parent " + obligation.getParentId()
                    + " unreachable, which should be impossible");
            return;
        }
        send(parent, new CertificateLostMessage(self.getRobotId(), parent.getRobotId(),
                originVertexID, originOutgoingEdgeID));
    }

    /**
     * Undraws the speculative edge an obligation drew for its child.
     *
     * <p>Reference removal, not a {@code toId} match: this only undoes the edge that this
     * obligation drew, leaving any other already-valid edge to the same robot id alone.
     * The reference now lives on the obligation rather than in one field, because with
     * several obligations live a child id no longer identifies which edge object to pull.
     */
    private void undrawChildEdge(Edge drawn) {
        if (drawn != null) {
            self.getEdges().remove(drawn);
        }
    }

    /**
     * The child this robot is currently waiting on, or {@code -1}.
     *
     * <p>Derived rather than stored. At {@link #MAX_CONCURRENT_OBLIGATIONS} of one this is
     * exactly the old {@code pendingChildID} field; once the cap lifts it becomes "any
     * child", which is all the gates below ever meant by it.
     */
    private int pendingChildID() {
        for (FaceObligation obligation : obligations.asList()) {
            if (!obligation.isUnfulfilled()) {
                return obligation.getChildId();
            }
        }
        return -1;
    }

    /** Whether this robot may take on another face right now. */
    private boolean canAcceptAnotherObligation() {
        return obligations.size() < MAX_CONCURRENT_OBLIGATIONS;
    }

    /** A null obligation means no exclusions -- see {@link #findBestNeighborForEdge}. */
    private static boolean isBannedOn(FaceObligation obligation, int robotID) {
        return obligation != null && obligation.isBanned(robotID);
    }

    /**
     * Frees the child slot of whichever obligation named this robot, keeping the tuple.
     *
     * <p>The parent and edge survive because this robot still owes that edge to somebody --
     * only the candidate changed. Bans accumulated on the face survive too, which is what
     * stops the next offer going straight back to the robot that just declined.
     */
    private void releaseObligationOfChild(int childID) {
        FaceObligation obligation = obligations.findByChild(childID);
        if (obligation == null) {
            return;
        }
        undrawChildEdge(obligation.release());
    }

    /**
     * Excludes a robot from the face this one is currently building.
     *
     * <p>Scoped to the obligation rather than to the robot, so the exclusion disappears when
     * the face does and cannot leak onto an unrelated face. That scoping is what let the
     * four hand-written {@code unableToDoAssignmentIDs.clear()} calls in the reset family
     * go away.
     */
    private void banOnCurrentFace(int robotID) {
        FaceObligation obligation = currentObligation();
        if (obligation != null) {
            obligation.ban(robotID);
        }
    }

    /**
     * The obligation this robot is working on, or null.
     *
     * <p>Single-valued only because the cap is one. When the cap lifts, every caller of this
     * has to name the face it means instead -- which is the point of routing them through
     * one method now.
     */
    private FaceObligation currentObligation() {
        List<FaceObligation> held = obligations.asList();
        return held.isEmpty() ? null : held.get(0);
    }

    /** Forwards a lost-certificate report to this robot's own parent, keeping its tuple. */
    private void forwardCertificateLostUpstream(CertificateLostMessage cm) {
        GeometricCycleLatticeRobot parent = getNeighborByID(anchorParentID);
        if (parent == null) {
            log("-> cannot forward lost certificate: parent " + anchorParentID + " unreachable");
            return;
        }
        send(parent, new CertificateLostMessage(self.getRobotId(), parent.getRobotId(),
                cm.getOriginVertexID(), cm.getOriginOutgoingEdgeID()));
    }

    /**
     * The obligation this robot owes on its outgoing edge for the face it is building,
     * created on first use.
     *
     * <p>Keyed on the <strong>outgoing</strong> edge -- the one handed to the child. For a
     * root that is the target edge chosen by {@link #determineNextCycleToComplete()}, which
     * is already the key {@code completedCycles} uses; for a cycleBuilder it is
     * {@code getNext(assignedEdge)}. Keying on the incoming edge would describe the same
     * partition, since {@code getNext} is a bijection, but would put a root's obligation on
     * a different key from its own cycle-status entry for no gain.
     */
    private FaceObligation obligationFor(HalfEdge outgoingEdge) {
        int parentId = role == CycleRole.root ? self.getRobotId() : anchorParentID;
        return obligations.getOrCreate(parentId, outgoingEdge.getId());
    }

    public void promoteToPrimaryRoot() {
        role = CycleRole.root;
        // An anchor has no business still stepping aside for anyone.
        policy.cancelEvasion();
        policy.clearDetour();
        initializeEdgeMap();
    }

    /**
     * Promotion leaves the obligation set alone, deliberately.
     *
     * <p>A promoted robot has not moved, so the local communication topology its tuples
     * describe is still real and it still owes every response it owed a moment ago.
     * Obligation lifetime keys on position, not role -- only vacating the lattice site
     * clears the set.
     */
    private void promoteSelfToStable() {
        role = CycleRole.stable;
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

        GeometricCycleLatticeRobot parent = getNeighborByID(anchorParentID);
        if (parent == null) return null;

        return globalTransformOf(parent.getPosition(), assignedEdge.getVoltage())
                .asPose();
    }

    /**
     * The transform from a parent's live pose into this robot's own frame -- the single
     * measured hop a relaying robot contributes to a certificate.
     *
     * <p>{@code Observation.getLocalPosition()} is the parent expressed in this robot's
     * frame, which is {@code T(me -> parent)}; the hop wanted is its inverse. Measured
     * rather than looked up: the ideal voltage is identity-by-construction around a face
     * and would certify nothing.
     *
     * @param parentId the robot that relayed to this one
     * @return {@code T(parent -> me)}, or null if the parent is not currently observable
     */
    private RigidBodyTransformation measureInboundHop(int parentId) {
        Observation parentObservation = observations.get(parentId);
        if (parentObservation == null) {
            return null;
        }
        return new RigidBodyTransformation(parentObservation.getLocalPosition()).inverse();
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
        this.obligations.clearAll();
        this.certificate = null;
        this.anchorParentID = -1;
        this.role = CycleRole.unassigned;

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;

        resetObservations();
    }

    public void resetObservations() {
        this.observations.clear();
    }

    /**
     * Clean slate for "unassigned": stableID, obligations, certificate, and the
     * assigned/origin edge references are all cleared, and role is set to unassigned.
     * hasBeenAssigned is the one state-data field deliberately left untouched. Differs from
     * the existing reset() above only in that it also clears stableID.
     *
     * <p><strong>This is the vacate path.</strong> A robot reaching here is giving up its
     * lattice site -- contention yield, liveness give-up, or finding its assignment already
     * occupied -- and the tuples it holds describe a local topology that is about to stop
     * being true, so all of them go. Callers already send their rejection <em>before</em>
     * calling this, which is why they carry that ordering comment.
     *
     * <p>At a cap of one, one rejection from the caller is exactly one rejection per tuple,
     * because every tuple a robot holds names the same {@code anchorParentID}. When the cap
     * lifts that stops being true and this must become
     * {@link FaceObligationSet#drainForVacate()} with one rejection per returned obligation.
     *
     * <p>Note that the ban list is no longer cleared by hand here: bans live on the
     * obligation, so they go when it does.
     */
    public void resetToUnassigned() {
        // The latched detour was chosen against a target this reset is discarding.
        policy.clearDetour();
        policy.resetProgress();
        this.stableID = -1;
        this.obligations.clearAll();
        this.certificate = null;
        this.anchorParentID = -1;
        this.role = CycleRole.unassigned;

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Clean slate for "cycleBuilder": stableID, obligations, certificate, and both the
     * assigned and origin edge references are cleared, and role is set to cycleBuilder.
     * Meant to be immediately followed by setAssignedEdge/setOriginEdge and the caller's own
     * certificate setup, the same way the unassigned -> cycleBuilder PositioningMessage
     * handling does today.
     *
     * <p>Obligations go for the same reason as in {@link #resetToUnassigned()}: accepting a
     * new assignment means moving to a different lattice site, so the topology the old
     * tuples described stops being true. This is not the promotion case -- a promotion
     * leaves the robot where it is and keeps its tuples.
     */
    public void resetToCycleBuilder() {
        // The latched detour was chosen against a target this reset is discarding.
        policy.clearDetour();
        policy.resetProgress();
        // A real assignment always beats getting out of someone's way. Placed here rather
        // than at each call site because the unassigned -> cycleBuilder path already
        // routes through this method, so one edit covers every way an assignment lands.
        policy.cancelEvasion();
        this.stableID = -1;
        this.obligations.clearAll();
        this.hasBeenAssigned = true;
        this.certificate = null;
        this.anchorParentID = -1;
        this.role = CycleRole.cycleBuilder;

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Clean slate for a root moving on to its next outgoing edge: obligations, certificate
     * and the origin edge reference are cleared, role is (re-)set to root, and hasFailed is
     * forced back to false. stableID and the assigned edge are deliberately left untouched
     * -- they're the root's own identity/position in the graph, not per-edge state. Same
     * reasoning excludes completedCycles entirely: it's the record of progress this reset
     * exists to avoid corrupting.
     *
     * <p><strong>Clearing obligations here is a cap-of-one shortcut, not the rule.</strong>
     * A root has not moved, so by the position-not-role principle its tuples should survive;
     * it is only safe to drop them because at a cap of one the single tuple always belongs
     * to the edge this reset is finishing. Once a root can build several faces at once,
     * this must remove only the finished edge's obligation and leave the rest -- otherwise
     * it silently discards responses the root still owes on its other faces.
     */
    public void resetToRoot() {
        // The latched detour was chosen against a target this reset is discarding.
        policy.clearDetour();
        policy.resetProgress();
        // A promoted robot is an anchor; it has no business still stepping aside.
        policy.cancelEvasion();
        this.obligations.clearAll();
        this.certificate = null;
        this.anchorParentID = -1;
        this.role = CycleRole.root;

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
                // Derived, not stored. Keeping this field means RobotFrameView and
                // TickRecord.changed() need no edit, which decouples logging churn from
                // protocol churn -- the obligations below are what new code should read.
                pendingChildID(),
                stableID,
                certificate,
                getAssignedEdge(),
                getOriginEdge(),
                Map.copyOf(completedCycles),
                snapshotQueueInOrder(),
                Map.copyOf(observations),
                List.copyOf(obligations.asList())
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
