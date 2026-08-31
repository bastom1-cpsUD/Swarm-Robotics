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

    /**
     * Corners whose neighbour has already been promoted -- see
     * {@link #promoteAdjacentVerticesToRoots()}.
     *
     * <p>Only needed because promotion is now reached repeatedly rather than once. It is not
     * protocol state: nothing reads it but the promote step, and losing it would cost a
     * duplicate promotion rather than a wrong one.
     */
    private final Set<Integer> announcedCorners = new HashSet<>();
    private final VoltageGraph graph;

    // State-Data
    private int stableID;
    private boolean hasBeenAssigned;

    /**
     * Whether this robot has reached the lattice site it was assigned, and so may stop
     * deriving its target from a parent it has to keep in sight.
     *
     * <p>Needed as of Phase 6, and only because of it. Until the cap lifted, a chain
     * <em>collapsed</em> on every outcome -- a cycleBuilder reporting success or failure
     * reset itself to unassigned -- so no robot stayed a cycleBuilder for long, and
     * re-deriving its pose from a live parent every tick was bounded by that collapse. A
     * robot that now serves several incident faces has to survive the first one resolving,
     * so the derivation is unbounded in time and two consequences stop being invisible:
     *
     * <ul>
     *   <li>{@link #getAssignedGlobalPosition()} returns null the moment the parent stops
     *       being observable. A settled robot would then have no target, which is exactly
     *       {@link #canHonourStandAside()}'s predicate -- it could be asked to step off a
     *       lattice site it is legitimately occupying, and unlike the occupancy case there
     *       is no backstop underneath.</li>
     *   <li>Every derived pose sits one composition further from a measured one, down a
     *       chain that no longer collapses.</li>
     * </ul>
     *
     * <p>Both go away by anchoring an arrived robot to itself -- the same short-circuit
     * {@code root} and {@code stable} have always had. It is exact rather than approximate
     * because {@code GeometricCycleLatticeRobot.updateAssignedPosition} snaps the pose onto
     * the target on arrival, so this freezes the robot on its ideal site rather than on
     * wherever it happened to stop.
     */
    private boolean hasArrived;

    /**
     * The faces this robot currently owes an offer or a response on -- see
     * {@link FaceObligation}.
     *
     * <p><strong>One per incident edge, and no other limit</strong> -- see
     * {@link #maxConcurrentObligations()}. It was capped at a single entry through Phases 4
     * and 5, which is what kept the migration to tuples separable from the concurrency they
     * enable; Phase 6 lifts that, so a robot standing where several faces meet carries all of
     * them at once. Which is the entire point of the representation: at a cap of one, "route
     * the response to the right parent" and "route it to the only parent" are the same
     * instruction, and a wrong implementation is indistinguishable from a right one.
     *
     * <p>Outstanding means {@link FaceObligation#isUnfulfilled()} -- a null child -- and is a
     * per-face condition with no effect on the inbox at all. A robot with an offer still to
     * make makes it in the same activation; a robot with a child recorded has already
     * discharged what it owed and is waiting for a response to route back through the tuple.
     * Neither state stops it listening.
     *
     * <p>That is the end of a line of gates. {@code pendingChildID} blocked while a child
     * <em>was</em> recorded and admitted only that child's traffic, which needed a standing
     * exemption for roots or it would refuse the very {@code PositioningMessage} carrying a
     * certificate home. Phase 5 inverted it to block on an outstanding obligation instead,
     * which needed no exemptions but would have stopped a robot in transit for one face from
     * answering for any other. Phase 6 removes it: what was really being guarded is one tuple
     * per edge, and {@link FaceObligationSet#getOrCreate} guards that directly.
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

    /**
     * Tick-rate collision policy. Held by composition: it decides where to aim when
     * something is in the way, and knows nothing about the cycle-building protocol.
     */
    private final AvoidancePolicy policy;

    // Logging / instrumentation support (see CommsSnapshot, org.logging package)
    private ArrayList<OutgoingMessageRecord> sentThisTick;

    /**
     * How many faces this robot may serve at once. <strong>Lifted in Phase 6.</strong>
     *
     * <p>It was a flat one through Phases 4 and 5, which is what kept the migration to
     * tuples separable from the concurrency they enable: at a cap of one this class behaved
     * exactly as the {@code pendingChildID} version did, so a regression in those phases was
     * a fault in the representation rather than in newly-parallel face building. Setting
     * this method back to {@code return 1} restores that behaviour exactly, which is the
     * rollback the plan asks for.
     *
     * <p>The cap that replaces it is <em>derived from the lattice</em> rather than picked, and
     * it bounds the walks this robot is <strong>carrying</strong>: one per incoming edge, of
     * which there are exactly as many as outgoing edges, since twin is a bijection. That is
     * the number of walks that can legitimately be passing through this site at once, and
     * anything beyond it is a robot being offered the same site twice.
     *
     * <p>It used to read {@code outgoing + 1}, and the {@code +1} needed a paragraph to
     * explain: it was the one tuple not keyed in the incoming space, this robot's own face.
     * That face now has a slot of its own, so the number decomposes into two facts that need
     * no explaining -- at most one carried walk per incoming edge, and at most one attempt.
     *
     * <p>The bound is advisory rather than load-bearing --
     * {@link FaceObligationSet#getOrCreate} already admits only one carried tuple per edge id,
     * so the set cannot grow past it in normal operation. It is kept as the place a violation
     * would be caught.
     */
    private int maxConcurrentObligations() {
        return graph.getOutgoingHalfEdges(getCurrentRole()).size();
    }

    public CyclebuilderComms(GeometricCycleLatticeRobot self, VoltageGraph graph) {
        this.graph = graph;
        this.trust = TrustLevel.Friendly;
        this.stableID = -1;
        this.hasBeenAssigned = false;
        this.hasArrived = false;
        this.anchorParentID = -1;
        this.role = CycleRole.unassigned;
        this.completedCycles = new HashMap<>();
        this.self = self;
        this.observations = new HashMap<>();
        this.policy = new AvoidancePolicy(self);
        this.incomingMessages = new ConcurrentLinkedQueue<>();
        this.sentThisTick = new ArrayList<>();
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

        // A robot still driving to its site answers nothing, and that is about honesty rather
        // than courtesy. Every question an assignment asks -- checkAssignmentForCurrentPosition
        // -- is answered against where this robot is RIGHT NOW, and a moving robot's answer
        // describes a pose it is about to leave. Worse, that answer is permanent: every
        // rejection in the assignment path is non-retryable, so the offerer writes this robot
        // onto that face's ban list, FaceObligation never lifts a ban, and the offerer works
        // down its neighbours until it runs out and kills the face. A robot that was merely
        // EARLY is written off for the life of the face and can take the face with it.
        //
        // Nothing is polled, so nothing is consumed and no budget is spent: the offer stays
        // queued exactly as it arrived, and is answered once the answer means something.
        //
        // This is a deferral, and the two things that make it safe where the old ones were not:
        //
        //   1. The offerer is not blocked. Its child slot is filled, so findUnfulfilled()
        //      rotates past that tuple and serves its other faces -- which is what the tuple
        //      set exists for. Only that one face waits.
        //   2. It is bounded at both ends. ARRIVAL releases it; and a robot that never arrives
        //      descends the liveness ladder, resetToUnassigned() drops its obligations, and the
        //      still-queued offer is then answered by an UNASSIGNED robot, which accepts it.
        //
        // What this does NOT suppress is the part that matters: TargetClaimMessage never enters
        // incomingMessages -- it arrives through receiveClaim into incomingClaims -- and
        // executeTimeStep runs detectAssignmentContention, consumeStandAside,
        // applyAvoidanceWaypoint, applyLivenessGiveUp and broadcastTargetClaim OUTSIDE this
        // method. A travelling robot still yields on contention, still evades, still gives up a
        // site it cannot reach, and still claims its target. That is what makes the gate safe
        // HERE, and the first thing to re-check if it ever moves.
        //
        // Costs one tick on release: hasArrived latches in sendMessage, which executeTimeStep
        // runs after this method, so the answer comes on the following activation. Already the
        // documented behaviour for promotions -- see acceptPromotion and
        // PromotionTest.promotionInTransitIsDeferredThenAccepted.
        //
        // Not gated: a robot picked by findBestNeighborForEdge's exact-position branch is
        // already standing on its site, so updateAssignedPosition() returns true on its first
        // activation as a cycleBuilder and hasArrived latches in the same tick.
        if (role == CycleRole.cycleBuilder && !hasArrived) {
            return "N/A (In transit, answering nothing until I am standing still)";
        }

        // Nothing else gates the inbox.
        //
        // Phase 5 let an OUTSTANDING obligation take the whole tick, on the reasoning that a
        // robot owing an offer should make it before listening to anything else. That was
        // never necessary -- sendMessage runs in the same activation and makes the offer
        // regardless -- and once a robot serves several faces it is actively wrong: a robot
        // still travelling to the site for one face would refuse to answer for any other,
        // which is the pendingChildID gate rebuilt under a new name. Blocking is now
        // expressed where the plan says it belongs, per EDGE, by FaceObligationSet admitting
        // one tuple per edge id.
        //
        // What remains is a rotation. A PositioningMessage this robot has accepted stays
        // queued, because the queue is where its certificate lives while the robot is in
        // transit (see holdInCustody). Those are skipped rather than consumed, so custody
        // costs a robot nothing off its one-message-per-tick budget.
        AbstractMessage next = pollNextActionable();
        if (next == null) {
            return "N/A (Only held assignments queued)";
        }

        if(!validateSenderIsNeighbor(next.getSenderId())) {
            log("-> sender " + next.getSenderId() + " is not a neighbor, discarding message");
            return "N/A (Discarded message from non-neighbor " + next.getSenderId() + ")";
        }

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
                    // forget.
                    //
                    // Set here and nowhere else: this is the only transition that decides
                    // which robot this one is anchored to. See the anchorParentID javadoc.
                    anchorParentID = pm.getSenderId();
                    self.addEdge(new Edge(self.getRobotId(), anchorParentID));
                    // The tuple opens on ACCEPTANCE now, not on arrival. Finding 5 put it at
                    // the first offer so a robot in transit held none, on the grounds that a
                    // tuple would describe a topology that was not true yet. Custody reverses
                    // that: the queued assignment is this robot's only copy of the
                    // certificate, and the tuple is what records whose certificate it is and
                    // who to hand it back to. Without one, a robot that gives its site up
                    // mid-journey tells nobody, and the offerer waits on a robot that left.
                    holdInCustody(pm, pm.getSenderId(), pm.getAssignedOutgoingEdgeID());
                    log("-> became cycleBuilder: edge id=" + getAssignedEdge().getId()
                            + " from vertex " + getAssignedEdge().getOrigin().getId()
                            + ", parent=" + anchorParentID
                            + ", cert=" + pm.getCertificate());
                    return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
                } else if(next instanceof PromotionMessage pm) {
                    return acceptPromotion(pm);
                } else {
                    log("-> unassigned robot received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case cycleBuilder:
                if (next instanceof StatusMessage sm) {
                    // Routed through the tuple, exactly as a root routes one. The old code
                    // read the parent off a field and then called resetToUnassigned(),
                    // collapsing the whole chain on any outcome. That collapse was the cap of
                    // one wearing a disguise: with a single face there was nothing else to
                    // keep, so tearing the robot down was indistinguishable from finishing.
                    // A robot serving several faces must survive the first one resolving, or
                    // the remaining tuples -- and the sites they describe -- go with it.
                    return routeStatusThroughTuple(sm);
                } else if(next instanceof CertificateLostMessage cm) {
                    return routeCertificateLostThroughTuple(cm);
                } else if(next instanceof RejectAssignmentMessage rm) {
                    return routeRejectionThroughTuple(rm);
                } else if(next instanceof PromotionMessage pm) {
                    // A builder used to defer this, forever. See acceptPromotion.
                    return acceptPromotion(pm);
                } else if(next instanceof PositioningMessage pm) {
                    // A second walk arriving at a robot that already has a site.
                    //
                    // This used to be a race between two parents for one robot, arbitrated by
                    // comparing hop counts and initiator ids, with a re-queue-forever branch
                    // underneath for the case neither won. All of it was the cap of one: only
                    // one of the two offers could be served, so one had to be chosen and the
                    // other could only be deferred -- and the deferral's own comment named the
                    // chain collapse as its release valve, which Phase 6 removes.
                    //
                    // Intermediates never filter (plan item 3). If the assignment names the
                    // site this robot is already standing on, it is a second FACE through the
                    // same site, not a rival for it, and both are carried. If it names a
                    // different site, the robot cannot take it and says so; the offerer bans
                    // it and moves to the next candidate, which is bounded where the deferral
                    // was not.
                    if(!checkAssignmentForCurrentPosition(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                        return "Positioning Message from " + pm.getSenderId() + "(REJECTED)";
                    }
                    return acceptForRelay(pm);
                } else {
                    log("-> cycleBuilder received unexpected message type: " + next.getMessageType());
                    return "N/A (Unhandled message type: " + next.getMessageType() + ")";
                }
            case root:
                if(next instanceof StatusMessage sm) {
                    // Identical to the cycleBuilder branch, and that is the point: a root is a
                    // lattice site like any other, and whether an outcome stops here or keeps
                    // travelling is decided by the tuple it routes through, never by role.
                    return routeStatusThroughTuple(sm);
                } else if(next instanceof CertificateLostMessage cm) {
                    return routeCertificateLostThroughTuple(cm);
                } else if(next instanceof RejectAssignmentMessage rm) {
                    return routeRejectionThroughTuple(rm);
                } else if(next instanceof PositioningMessage pm) {
                    if(!checkAssignmentForCurrentPosition(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + "(WILL BREAK FORMATION)");
                        return "Positioning Message from " + pm.getSenderId() + " (REJECTED, WILL BREAK FORMATION)";
                    }
                    if(!assignmentMatchesCurrentRole(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + " (edge lands on the wrong role)");
                        return "Positioning Message from " + pm.getSenderId() + " (REJECTED, WRONG ROLE)";
                    }

                    // Is this my own certificate coming home? That question, and not "am I a
                    // root standing in the right place", is what decides closure now. The old
                    // branch here granted SUCCESS on role and pose alone, so a walk that
                    // started at one root and ended at a different one was certified as a
                    // face (defect 2); and where the sender was itself a root it emitted
                    // AttemptLater instead of carrying the walk, so a ring of roots deferred
                    // to one another forever (defect 1). Both branches are gone.
                    if(mintedHere(pm.getCertificate())) {
                        return evaluateReturningCertificate(pm);
                    }

                    return acceptForRelay(pm);
                } else if(next instanceof PromotionMessage pm) {
                    return acceptPromotion(pm);
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
                    } else if(!assignmentMatchesCurrentRole(pm)) {
                        forwardRejectionUpstream(pm, false);
                        log("-> assignment REJECTED by " + pm.getSenderId() + " (edge lands on the wrong role)");
                        return "Positioning Message from " + pm.getSenderId() + "(REJECTED, WRONG ROLE)";
                    } else {
                        // Stable short-circuits to SUCCESS without asking whose certificate
                        // this is, and that is deliberate rather than an oversight of the
                        // closure rule. A stable robot has already closed every face incident
                        // to its own site, so the face this walk is tracing is one of them --
                        // built, verified and standing. Sending the walk onward would only
                        // re-derive a result already recorded.
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
                // One outstanding tuple is serviced per activation, rotating (see
                // FaceObligationSet.findUnfulfilled), so no incident face can starve another.
                // This runs before the hasFailed() stand-down deliberately: a root that has
                // given up on its own corners is still a lattice site, and refusing to carry a
                // neighbour's walk would strand it.
                //
                // Note what is NOT here any more: the "Waiting for Status Update" gate. A root
                // with a child recorded is precisely a root that has discharged what it owed
                // and is free to start another face; blocking on it was the last surviving
                // spelling of pendingChildID.
                String serviced = serviceOneObligation();
                if (serviced != null) {
                    return serviced;
                }
                // NO hasFailed() stand-down here. There used to be one, and it swallowed the
                // promotion step: a root whose corners end up part complete and part failed
                // returned "ceased operations" from this line and never reached the
                // targetEdgeID == -1 branch below, which is the only place
                // promoteAdjacentVerticesToRoots() is called. So the neighbours standing on the
                // corners that DID close were never told, and the frontier stopped at a root
                // that had genuinely built something.
                //
                // The dead code below was the fingerprint: the branch that returns "not all
                // cycles completed... ceasing operations" is written for exactly the
                // part-failed case, and could not be reached, because nothing between the old
                // guard and it mutates completedCycles.
                //
                // Nothing is needed in its place. hasFailed() means every corner is complete or
                // failed, so determineNextCycleToComplete() returns -1 on its own and the root
                // falls through to the promote-then-stand-down branch. The stand-down still
                // happens, one step later, after it has told its neighbours.

                // ONE face of its own at a time, still. Lifting the cap made a robot able to
                // carry several walks, and that is the whole of what it was for; it did not
                // make a root able to BUILD several of its own corners at once, and letting it
                // try is a loss rather than a gain. Every corner of one root draws candidates
                // from the same neighbourhood, so a second walk started before the first has
                // one merely offers the same nearest robots a second site, collects a
                // non-retryable rejection from each because they are standing on the first,
                // and bans them -- spending its candidates on refusals. Concurrency across
                // DIFFERENT robots is real; concurrency across one robot's own corners is
                // contention with itself.
                if (hasInitiatedFaceInFlight()) {
                    return "N/A (Already building a face of my own)";
                }
                // Find the first outgoing edge that does not have a completed cycle yet
                int targetEdgeID = determineNextCycleToComplete();

                if (targetEdgeID == -1) {
                    // -1 no longer means "nothing left to do". determineNextCycleToComplete
                    // now skips corners with a walk already in flight, so it also returns -1
                    // while this root is simply busy on all of them -- and promoting on that
                    // would declare the site finished with faces still open.
                    if (!obligations.isEmpty()) {
                        return "N/A (Every remaining cycle is already in flight)";
                    }

                    // Every corner is now settled, one way or the other. Announce the ones that
                    // closed BEFORE deciding what this failure means for this robot: a corner
                    // that closed is a real face with a real robot on the far side of it, and
                    // that robot's right to build outward does not depend on how its neighbour's
                    // other corners went.
                    int promoted = promoteAdjacentVerticesToRoots();

                    if(!hasFailed()) {
                        promoteSelfToStable();
                        log("-> all cycles completed, promoting self to stable");
                        return "Done (All cycles completed, promoted to stable)";
                    }

                    // Reachable now, which it was not before. A root that stands down here has
                    // still handed on whatever it built.
                    return "N/A (Ceased operations due to failure"
                            + (promoted > 0 ? ", promoted " + promoted + " neighbour(s) first)" : ")");
                }

                log("Sending Message...");

                HalfEdge targetEdge = retrieveEdgeFromGraph(targetEdgeID);

                // Open the tuple and stop. The offer itself goes out on the next activation,
                // through the same serviceOneObligation path every other face uses, so a root
                // initiating and a root relaying are one code path with one budget rather than
                // two that have to be kept in step.
                obligationForInitiatedFace(targetEdge);
                log("-> opened a face on edge " + targetEdgeID + " of vertex "
                        + targetEdge.getOrigin().getId());
                return "Opened cycle on edge " + targetEdgeID
                        + " of vertex " + targetEdge.getOrigin().getId();
            }
            case CycleRole.cycleBuilder: {
                if(!alreadyInPosition) {
                    return "N/A (Not in position to broadcast)";
                }
                // Arrived, and from here on anchored to itself rather than to a parent it has
                // to keep in sight. See the hasArrived javadoc for why that stops being
                // optional once chains no longer collapse.
                hasArrived = true;

                String serviced = serviceOneObligation();
                if(VERBOSE && serviced != null) {
                    log("Sent Message...");
                }

                return serviced != null ? serviced : "N/A (No outstanding obligation)";
            }
            case CycleRole.unassigned:
                return "N/A (Unassigned robot do not broadcast)";
            default:
                return "N/A (Unhandled)";
        }
    }

    /**
     * Makes the one offer this robot owes this activation, or reports that it owes none.
     *
     * <p>Rotating rather than first-found, so a robot serving several incident faces cannot
     * let a low-numbered edge monopolise it -- {@link FaceObligationSet#findUnfulfilled()}
     * resumes from where it last handed one out.
     *
     * <p>The two kinds of tuple diverge only in where the certificate comes from. A face this
     * robot <em>initiated</em> has none yet, so one is minted here; a face it is
     * <em>carrying</em> has one queued in the inbox, which is the whole of certificate
     * custody. Everything after that -- pick a candidate, draw the edge, fill the child slot
     * -- is shared.
     *
     * @return a description for the tick log, or null if there was nothing outstanding
     */
    private String serviceOneObligation() {
        FaceObligation obligation = obligations.findUnfulfilled();
        if (obligation == null) {
            return null;
        }
        return isInitiatedFace(obligation) ? initiateOnward(obligation) : relayOnward(obligation);
    }

    /** Sends the first offer on a face this robot is building itself, minting the certificate. */
    private String initiateOnward(FaceObligation obligation) {
        HalfEdge targetEdge = edgeOwedBy(obligation);
        if (targetEdge == null) {
            obligations.remove(obligation);
            return "N/A (Obligation names an unknown edge " + obligation.getEdgeId() + ")";
        }

        GeometricCycleLatticeRobot child =
                findBestNeighborForEdge(targetEdge, obligation);
        if (child == null) {
            log("Ran out of options for building cycle on edge " + targetEdge.getId()
                    + ", failing edge and moving on");
            setCycleStatusOf(targetEdge.getId(), CycleStatus.failed);
            obligations.remove(obligation);
            return "Failed (No valid neighbors for cycle on edge " + targetEdge.getId() + ")";
        }

        VoltageCertificate minted = new VoltageCertificate(self.getRobotId());
        offer(obligation, child, targetEdge, targetEdge.getId(),
                getVertexIDof(targetEdge), getEdgeIDof(targetEdge), minted);
        return "Assigned position to robot " + child.getRobotId()
                + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
    }

    /** Carries a walk one hop further, extending the certificate held in the inbox for it. */
    private String relayOnward(FaceObligation obligation) {
        PositioningMessage held = custodyFor(obligation);
        if (held == null) {
            // The certificate this tuple exists to carry is not in the inbox, so there is
            // nothing to relay and no way to get it back. Structurally unreachable -- custody
            // is created and destroyed with the tuple -- and dropping the tuple is the only
            // honest response if it ever happens.
            obligations.remove(obligation);
            log("-> no certificate in custody for edge " + obligation.getEdgeId()
                    + "; dropping the obligation");
            return "N/A (No certificate in custody for edge " + obligation.getEdgeId() + ")";
        }

        HalfEdge targetEdge = edgeOwedBy(obligation);
        if (targetEdge == null) {
            return reportRelayFailure(obligation, held);
        }

        // Extend by the hop INTO this robot -- T(parent -> me), not T(me -> child). The child
        // has not moved yet, so measuring outbound would sample a pose that does not exist.
        // Both endpoints of the inbound hop are settled: the parent relayed earlier, and the
        // alreadyInPosition gate in sendMessage means this robot is on its own site now.
        RigidBodyTransformation inboundHop = measureInboundHop(obligation.getParentId());
        if (inboundHop == null) {
            // The parent is one lattice edge away and parked, so it is meant to be permanently
            // in range (FaceClosureTest pins every edge below COMM_RANGE). Losing sight of it
            // means the certificate cannot be extended honestly, and a certificate extended
            // with a guessed hop certifies nothing. Hold the tuple and retry next tick rather
            // than relaying a measurement this robot did not take.
            log("-> parent " + obligation.getParentId()
                    + " not observable, cannot extend certificate this tick");
            return "N/A (Parent " + obligation.getParentId() + " unobservable, certificate not extended)";
        }

        // Three scalars and nothing else: initiator, hop count, accumulated measured
        // transform. Task 6 of DCEL-Implementation-Plan.md asks for a literal
        // graph.validateCycle(walk, ...) call, which would need the certificate to carry the
        // half-edge ids it traversed -- and that roster is redundant with the transform it
        // already accumulates along exactly that walk. The closure predicate satisfies Task
        // 6's intent instead: length against the face's cycleLength, identity of the minter,
        // and the accumulated product as an exactness cross-check.
        VoltageCertificate onward = held.getCertificate().extend(inboundHop);

        GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge, obligation);
        if (child == null) {
            log("-> NO candidate found, reporting FAILURE to " + obligation.getParentId());
            return reportRelayFailure(obligation, held);
        }

        log("-> relaying robot " + onward.getInitiatorID() + "'s walk to " + child.getRobotId()
                + " on edge " + targetEdge.getId());
        offer(obligation, child, targetEdge, held.getOriginOutgoingEdgeID(),
                held.getOriginVertexID(), held.getOriginOutgoingEdgeID(), onward);
        return "Assigned position to robot " + child.getRobotId()
                + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
    }

    /** Sends one assignment, draws its edge and fills the tuple's child slot. */
    private void offer(FaceObligation obligation, GeometricCycleLatticeRobot child,
                       HalfEdge targetEdge, int loggedEdgeId,
                       int originVertexID, int originEdgeID, VoltageCertificate cert) {
        PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(),
                getVertexIDof(targetEdge), getEdgeIDof(targetEdge),
                originVertexID, originEdgeID, cert);
        send(child, pm);
        Edge drawn = new Edge(self.getRobotId(), child.getRobotId());
        self.addEdge(drawn);
        obligation.fulfil(child.getRobotId(), drawn);
        if (VERBOSE) {
            log("Message sent to " + child.getRobotId() + " for edge " + loggedEdgeId);
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
        if (role != CycleRole.cycleBuilder || anchorParentID == -1) {
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
     * makes this work at all. The protocol queue pops one message a tick, and — decisively —
     * the {@code unassigned} branch of
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
     * True if {@code a} outranks {@code b} for a contested spot: possession first, then
     * distance, then id.
     *
     * <p>Possession is passed in rather than recovered from the distances. Deriving it as
     * {@code isZero(dist)} works for whichever robot is asking -- its own claim is a vector
     * from itself, so zero length is literally "I am standing on it" -- and fails for the
     * rival, whose possession has to come off the claim in its <em>sender's</em> frame; see
     * {@link #isParked}. It also could not express "both parked", because a zero-length
     * {@code distA} would answer before {@code distB} was looked at, and both robots would
     * claim possession and neither would yield. That case is the whole reason rule 3 exists.
     *
     * <p>Both distances must be measured the same way -- each robot to its <em>own</em>
     * declared target -- or the predicate is not symmetric and the pair can both yield or
     * neither. {@link #detectAssignmentContention} reads the rival's from the claim it
     * broadcast, so the two robots compare the identical pair of scalars and
     * {@code Double.compare} is an exact, agreed test rather than a brittle one.
     */
    protected boolean outranks(boolean aParked, double distA, int idA,
                               boolean bParked, double distB, int idB) {
        // 1. Whoever is already standing on the spot keeps it.
        if (aParked != bParked) {
            return aParked;
        }
        // 2. Otherwise whoever has less ground left to cover.
        int byDistance = Double.compare(distA, distB);
        if (byDistance != 0) {
            return byDistance < 0;
        }
        // 3. Equidistant -- including both parked -- goes to the lower id.
        return idA < idB;
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
     * <p>That tie-break is a total order over {@code (parked, distance, id)} -- see
     * {@link #outranks(boolean, double, int, boolean, double, int)}. Possession comes first
     * because neither of the others has any notion of who is already there: an arrived robot
     * that is merely further along its own approach, or holds the higher id, would tear
     * itself down -- rejecting to its parent and dropping its subtree -- in favour of an
     * interloper that had not got there yet. Distance comes next so the robot with less
     * ground to cover finishes the spot instead of two robots trading it. Id is last, and
     * settles the genuinely undecidable cases: equidistant rivals, and two robots both
     * standing on the spot.
     *
     * <p><strong>Every input to that order is read off the claims, never off an
     * observation.</strong> Possession comes from the claim <em>before</em> it is transformed
     * into this frame, via {@link #isParked}, for the reason given there; the rival's
     * distance is the length of that same untransformed claim, which is the rival's distance
     * to its own target as the rival itself computed it. So both robots rank the identical
     * pair of scalars and reach opposite verdicts, which is what makes exactly one of them
     * yield. Measuring the rival against <em>this</em> robot's claim point instead would rank
     * the pair against two reference points up to gamma apart, and they could both yield or
     * neither. Observation decides only <em>whether</em> the two are contending, via the
     * bounding-circle test.
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
        double myDistance = Vec2.of(myClaim).magnitude();
        boolean iAmParked = isParked(myClaim);

        // The maximum possible distance a robot can move in one tick; radius of the "contention zone" around a lattice spot. If two robots individual
        // claims fall within this distance, they can be consider equal if their claimed role IDS are the same.
        //
        // Shared with the collision layer via tickTravel() rather than recomputed here:
        // both layers reason about one tick of travel, and two independent expressions of
        // the same quantity would eventually drift apart.
        double gamma = GeometricCycleLatticeRobot.tickTravel();

        // Resolve against the strongest rival rather than the first one found: iteration
        // order over the claim map must not decide who keeps the assignment. "Strongest" is
        // the same order the verdict itself uses -- parked, then nearer, then lower id -- so
        // that the rival selected is the one this robot would actually lose to. Ranking
        // rivals on a different order than the one that decides could pick a near, moving
        // rival over a parked one and hand back a KEPT on a spot somebody is standing on.
        int bestRivalID = -1;
        double bestRivalDistance = -1;
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

            // The rival's own distance to its own target, read straight off the claim in the
            // frame it was broadcast in -- NOT the observed gap between where this robot sees
            // the rival and where this robot's target is.
            //
            // Two reasons, and both are about symmetry. First, the observed version would
            // measure everyone against MY claim point, while the rival measures everyone
            // against ITS claim point; the two differ by up to gamma, so the pair can rank
            // each other in opposite orders and either both yield or neither does. Read off
            // the claim, both robots compare the identical pair of scalars. Second, a claim
            // is exact where an observation is not: the rival computed this distance from its
            // own pose, so no sensing error enters the ranking at all. Observation still
            // decides WHETHER the two are contending, via the bounding circle above; it no
            // longer decides who wins.
            double theirDistance = Vec2.of(claimInSenderFrame).magnitude();

            // Read off the claim as it arrived, before the frame change above -- see
            // isParked. The rival is a genuine contender for this spot either way; this
            // only decides which of us gives it up.
            boolean rivalParked = isParked(claimInSenderFrame);

            if (bestRivalID == -1 || outranks(rivalParked, theirDistance, senderID,
                                              bestRivalParked, bestRivalDistance, bestRivalID)) {
                bestRivalID = senderID;
                bestRivalParked = rivalParked;
                bestRivalDistance = theirDistance;
            }
        }

        if (bestRivalID == -1) {
            return null;
        }

        // iAmParked and bestRivalParked are handed to the predicate rather than left for it
        // to infer. They used to be computed here and then used only to word the log line,
        // while the decision re-derived possession as isZero(distance) -- which is right for
        // this robot and wrong for the rival, whose possession only survives transport when
        // it is read in the sender's own frame. A parked rival therefore read as merely
        // nearby, and this robot would keep the spot and drive onto it.
        // Keep the spot when this robot OUTRANKS the rival. The test used to be negated --
        // "keep it when I do not outrank" -- which handed the spot to whichever robot was
        // further away, or held the higher id, and the log lines underneath said the
        // opposite of what the branch had actually decided. It survived because the one test
        // covering it put the lower id further from the target, so distance and id disagreed
        // and the inverted answer read as the right one.
        if (outranks(iAmParked, myDistance, self.getRobotId(),
                     bestRivalParked, bestRivalDistance, bestRivalID)) {
            String why = iAmParked && !bestRivalParked ? "I am already parked on it"
                    : myDistance < bestRivalDistance ? "I am closer to it"
                    : "I hold the lower id";
            log("-> Contention with robot " + bestRivalID + " over my assignment. " + why
                    + "; keeping it.");
            return "Assignment contention with robot " + bestRivalID + " (KEPT, "
                    + (iAmParked && !bestRivalParked ? "parked"
                       : myDistance < bestRivalDistance ? "closer" : "lower id") + ")";
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
        String why = lostToPossession ? "It is already parked there"
                : bestRivalDistance < myDistance ? "It is closer to it"
                : "I hold the higher id";
        log("-> Contention with robot " + bestRivalID + " over my assignment. " + why
                + "; yielding and forwarding rejection to parent.");
        return "Assignment contention with robot " + bestRivalID + " (YIELDED, "
                + (lostToPossession ? "rival parked"
                   : bestRivalDistance < myDistance ? "rival closer" : "higher id") + ")";
    }

    /*
        ////////////////////////
        TUPLE ROUTING
        ////////////////////////

        Every response a child can send -- a status, a rejection, a lost certificate --
        arrives with one question attached: whose face is this, and who is waiting on it?
        The three methods below answer it the same way, by looking the sender up in the
        obligation set, and none of them consults `role`.

        That uniformity is the whole point of the migration. Before it, each response was
        handled once per role, and the role branches disagreed: a cycleBuilder tore itself
        down on any status, a root marked one of its own corners, and neither could tell a
        face it had initiated from one it was merely carrying. `isInitiatedFace` -- parented
        to me or not -- is that distinction, and it is a property of the tuple rather than of
        the robot holding it.
     */

    /**
     * Routes a child's status to whoever is waiting on that face, and records the outcome
     * anywhere along the walk that tracks the corner it closed.
     *
     * <p>Two things happen here that used to be one. <strong>Marking</strong> is done by
     * every participant that keeps cycle bookkeeping, not just the initiator -- see
     * {@link #markCornerFromStatus}. <strong>Routing</strong> stops at the initiator and
     * forwards everywhere else.
     */
    private String routeStatusThroughTuple(StatusMessage sm) {
        FaceObligation obligation = obligations.findByChild(sm.getSenderId());
        if (obligation == null) {
            return statusWithNoTuple(sm);
        }

        markCornerFromStatus(obligation, sm);

        obligations.remove(obligation);
        releaseCustody(obligation, sm.getCertificate());
        if (!sm.isSuccessful()) {
            // A failed face leaves no edge worth drawing; a successful one is real topology.
            undrawChildEdge(obligation.getChildEdge());
        }

        if (isInitiatedFace(obligation)) {
            log("-> cycle on edge " + edgeIdOwedBy(obligation)
                    + (sm.isSuccessful() ? " COMPLETED" : " FAILED, moving on"));
            return "Status Message from " + sm.getSenderId()
                    + (sm.isSuccessful() ? "(SUCCESS)" : "(FAILURE)");
        }

        // Verbatim onward: the certificate, the origin edge and the verdict are none of them
        // this robot's to reinterpret.
        int parentId = obligation.getParentId();
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        if (parent == null) {
            log("-> cannot pass a status back to " + parentId + ": unreachable");
            return "Status Message from " + sm.getSenderId() + "(PARENT UNREACHABLE)";
        }
        send(parent, new StatusMessage(self.getRobotId(), parentId, sm.isSuccessful(),
                sm.getOriginVertexID(), sm.getOriginOutgoingEdgeID(), sm.getCertificate()));
        log("-> passed a " + (sm.isSuccessful() ? "SUCCESS" : "FAILURE") + " back to " + parentId);
        return "Status Message from " + sm.getSenderId()
                + (sm.isSuccessful() ? "(SUCCESS, RELAYED)" : "(FAILURE, RELAYED)");
    }

    /**
     * Records a settled face against this robot's own corner of it, if it tracks corners at
     * all.
     *
     * <p><strong>The corner is {@code edgeOwedBy(obligation)}, not the status's
     * {@code originOutgoingEdgeID}.</strong> The plan says otherwise and the plan is wrong
     * here. The origin edge names the corner as the <em>initiator</em> sees it, and every
     * other robot on the walk occupies a different site with a different outgoing edge for
     * the same face -- a participant assigned edge {@code a} owns the corner
     * {@code next(a)}. Marking the origin id at a participant writes a key that belongs to
     * someone else's site, which on a single-role lattice is a plausible-looking edge id and
     * so fails silently. The two spellings agree at exactly one robot, the initiator, which
     * is why the mistake survives every test that only watches an initiator.
     *
     * <p>Marking at all participants is what makes a filtered duplicate idempotent: once a
     * corner is complete, a late certificate or status for it is a no-op rather than a
     * second attempt. That is the rule one-tuple-per-edge rests on, so it is required rather
     * than an optimisation. It is a no-op for a cycleBuilder, whose {@code completedCycles}
     * is empty until promotion fills it -- deliberately, since a promotion re-initialises
     * that map anyway.
     */
    private void markCornerFromStatus(FaceObligation obligation, StatusMessage sm) {
        int corner = edgeIdOwedBy(obligation);
        if (corner == -1 || !completedCycles.containsKey(corner)) {
            return;
        }
        if (sm.isSuccessful()) {
            setCycleStatusOf(corner, CycleStatus.complete);
        } else if (isInitiatedFace(obligation)) {
            // Only the robot building this corner may write it off. A failure passing through
            // is somebody else's dead end and says nothing about whether this robot can close
            // the same face from its own side.
            setCycleStatusOf(corner, CycleStatus.failed);
        }
    }

    /**
     * A status that matches no tuple this robot holds.
     *
     * <p>Expected in exactly one situation: this robot stood down on a face it was
     * co-initiating (see {@link #acceptForRelay}), dropping the tuple while its own offer was
     * still out, and the abandoned child has now answered. If the corner is already complete
     * the walk that won got there first and this is old news, so it goes quietly. Anything
     * else is a status with nowhere to route, which means a tuple was removed while a child
     * still held its certificate -- worth seeing rather than swallowing.
     */
    private String statusWithNoTuple(StatusMessage sm) {
        int origin = sm.getOriginOutgoingEdgeID();
        if (getCycleStatusOf(origin) == CycleStatus.complete) {
            log("-> status for edge " + origin + " arrived after it was already complete; dropping");
            return "Status Message from " + sm.getSenderId() + "(DROPPED, EDGE ALREADY COMPLETE)";
        }
        log("-> ANOMALY: status from " + sm.getSenderId() + " for edge " + origin
                + " matches no obligation, and that edge is not complete. Something removed a "
                + "tuple while its child still held the certificate.");
        return "Status Message from " + sm.getSenderId() + "(ANOMALY, NO MATCHING OBLIGATION)";
    }

    /**
     * Frees the child slot so the same edge can be offered to somebody else, and never
     * propagates.
     *
     * <p>A rejection is the one response that leaves the tuple standing: the face is still
     * viable, only the candidate was wrong. The old cycleBuilder branch answered a
     * <em>retryable</em> rejection by rejecting to its own parent and resetting -- unwinding
     * the entire chain because one candidate below it was momentarily busy. That was the cap
     * of one again: with a single face there was nothing to preserve, so collapsing and
     * re-forming was as good as retrying. It is not, once the robot holds other faces.
     */
    private String routeRejectionThroughTuple(RejectAssignmentMessage rm) {
        FaceObligation obligation = obligations.findByChild(rm.getSenderId());
        log("-> assignment REJECTED by " + rm.getSenderId());
        if (obligation == null) {
            return "Reject Assignment Message from " + rm.getSenderId() + "(NO MATCHING OBLIGATION)";
        }

        undrawChildEdge(obligation.release());

        if (rm.isRetryable()) {
            // No ban: the rejecter said "not now", and it has left the target site, so
            // nearest-candidate selection will not simply hand it back the same offer.
            log("-> assignment is retryable, will re-offer edge " + obligation.getEdgeId());
        } else if (rm.getCertificate() != null
                && rm.getCertificate().getInitiatorID() == rm.getSenderId()) {
            // Never ban the walk's initiator. The ban list gates only the cycle-closing test
            // at the top of findBestNeighborForEdge -- the general candidate loop underneath
            // already excludes the initiator by id -- so banning it buys nothing when the
            // mismatch is real (a genuinely misplaced initiator fails that closure test on its
            // own) and forfeits the close permanently when the mismatch was merely numerical.
            log("-> NOT retryable, but sender initiated this walk; not banning it, will retry the close");
        } else {
            // Scoped to this face's own tuple, so it survives a retry of the same edge and
            // disappears with the face.
            obligation.ban(rm.getSenderId());
            log("-> assignment is NOT retryable, banning " + rm.getSenderId() + " on this face");
        }
        return "Reject Assignment Message from " + rm.getSenderId()
                + (rm.isRetryable() ? "(REJECTED, RETRYABLE)" : "(REJECTED, NOT RETRYABLE)");
    }

    /**
     * Passes a lost-certificate report to whoever can act on it.
     *
     * <p>Branches on the tuple, not on role. The robot that <em>initiated</em> the face is
     * the only one that can mint a replacement, so the report stops there and the corner is
     * marked {@code attempted} rather than {@code unattempted} -- which sends
     * {@link #determineNextCycleToComplete()} to the other edges first and back to this one
     * later, instead of spinning on a corner that may be short of candidates. Everywhere else
     * it keeps travelling.
     */
    private String routeCertificateLostThroughTuple(CertificateLostMessage cm) {
        FaceObligation obligation = obligations.findByChild(cm.getSenderId());
        if (obligation == null) {
            log("-> lost-certificate report from " + cm.getSenderId() + " matches no obligation");
            return "Certificate Lost Message from " + cm.getSenderId() + "(NO MATCHING OBLIGATION)";
        }

        undrawChildEdge(obligation.getChildEdge());
        obligations.remove(obligation);
        releaseCustody(obligation, null);

        if (isInitiatedFace(obligation)) {
            setCycleStatusOf(edgeIdOwedBy(obligation), CycleStatus.attempted);
            log("-> certificate for edge " + edgeIdOwedBy(obligation) + " was lost below "
                    + cm.getSenderId() + "; will relaunch it later");
            return "Certificate Lost Message from " + cm.getSenderId() + "(WILL RELAUNCH)";
        }

        forwardCertificateLostTo(obligation.getParentId(), cm);
        return "Certificate Lost Message from " + cm.getSenderId() + "(FORWARDED)";
    }

    //MESSAGE-PROCESSING UTIL

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
     * Hands every face this robot is holding back to the robot that gave it that face, and
     * empties the set. <strong>The vacate path.</strong>
     *
     * <p>One rejection <em>per tuple</em>, each to its own parent, each carrying back the
     * certificate that tuple has in custody. At a cap of one that was a single call to a
     * single {@code anchorParentID} with a single field for the certificate, and the plan
     * marked it as a shortcut to undo here; with several faces live it is undone by
     * construction, because {@link FaceObligationSet#drainForVacate()} makes "tell each
     * parent, then clear" one operation that cannot be written in the wrong order.
     *
     * <p>Callers run this <em>before</em> their reset, which is why they carry that ordering
     * comment. An undeliverable rejection costs only the parent's knowledge of it -- the
     * parent independently notices the child leaving, via the departure check in
     * {@link #makeObservations()}.
     *
     * @param isRetryable whether the parent should re-offer this same spot to this robot
     *                    later, or write it off and look elsewhere
     */
    private void forwardRejectionToParent(boolean isRetryable) {
        for (FaceObligation obligation : obligations.drainForVacate()) {
            // The drawn edges describe a site this robot is leaving.
            undrawChildEdge(obligation.getChildEdge());

            PositioningMessage held = takeCustody(obligation);
            if (isInitiatedFace(obligation)) {
                continue;   // this robot started that face; there is nobody upstream to tell
            }

            GeometricCycleLatticeRobot parent = getNeighborByID(obligation.getParentId());
            if (parent == null) {
                log("-> cannot reject to parent " + obligation.getParentId() + ": unreachable");
                continue;
            }
            send(parent, new RejectAssignmentMessage(self.getRobotId(), parent.getRobotId(),
                    held == null ? originVertexID : held.getOriginVertexID(),
                    held == null ? originOutgoingEdgeID : held.getOriginOutgoingEdgeID(),
                    isRetryable,
                    held == null ? null : held.getCertificate()));
        }
    }

    /*
        ////////////////////////
        CLOSURE
        ////////////////////////
     */

    /**
     * How exactly the closed product must equal the identity.
     *
     * <p><strong>This is not a drift budget, and must not be loosened into one.</strong> The
     * measured product telescopes -- see {@link VoltageCertificate} -- so a walk that
     * genuinely returned to the robot that minted its certificate closes at zero to floating
     * point, whatever the placement error along the way. There is no accumulated error for a
     * tolerance to absorb, so a lattice-sized epsilon here would buy nothing and would hide
     * the only two things this conjunct can actually catch: a walk that ended at a different
     * robot, and a robot that moved while the certificate was in flight.
     */
    private static final double CLOSURE_EXACTNESS = 1e-9;

    /** Whether this robot minted the certificate, and so is the only one permitted to judge it. */
    private boolean mintedHere(VoltageCertificate cert) {
        return cert != null && cert.getInitiatorID() == self.getRobotId();
    }

    /**
     * Whether the edge being handed to this robot lands on the role it already occupies.
     *
     * <p>Free next to {@link #checkAssignmentForCurrentPosition}, and catches something that
     * check cannot: an assignment whose geometry happens to coincide with this robot's pose
     * but whose edge belongs to a different role. Applied only where the role is settled --
     * root and stable. An unassigned robot has no role yet, and
     * {@link #getCurrentRole()} substitutes the primary role for it, so the same test there
     * would reject every legitimate assignment on any multi-role lattice.
     */
    private boolean assignmentMatchesCurrentRole(PositioningMessage pm) {
        HalfEdge assigned = retrieveEdgeFromGraph(pm.getAssignedOutgoingEdgeID());
        Role mine = getCurrentRole();
        return assigned != null && mine != null && assigned.getTarget().getId() == mine.getId();
    }

    /**
     * Decides a certificate that has come back to the robot that minted it, and reports the
     * outcome to whoever handed it back.
     *
     * <p>Closure needs all three conjuncts of {@link #closesFace}. Identity is already
     * established by the caller; the other two are checked here so each can be logged with
     * its own reason, because "the walk was the wrong length" and "the walk did not land
     * where it started" are different diagnoses and the second one is an anomaly.
     */
    private String evaluateReturningCertificate(PositioningMessage pm) {
        VoltageCertificate cert = pm.getCertificate();
        HalfEdge originEdge = retrieveEdgeFromGraph(pm.getOriginOutgoingEdgeID());

        if (originEdge == null) {
            forwardFailureUpstream(pm);
            log("-> own certificate returned naming an unknown origin edge "
                    + pm.getOriginOutgoingEdgeID() + "; cannot close");
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, UNKNOWN ORIGIN EDGE)";
        }

        int cycleLength = originEdge.getFace().getCycleLength();
        int walkLength = cert.getHops() + 1;   // the closing hop is this robot's to contribute

        if (walkLength != cycleLength) {
            // Length is the conjunct that separates a triangle from a square where both meet
            // at one corner: the walk closed geometrically, but not around the face it set
            // out to trace.
            setCycleStatusOf(pm.getOriginOutgoingEdgeID(), CycleStatus.failed);
            forwardFailureUpstream(pm);
            log("-> own certificate returned after " + walkLength + " hops, but face "
                    + originEdge.getFace().getId() + " is " + cycleLength + " long; NOT a closure");
            return "Positioning Message from " + pm.getSenderId()
                    + " (REJECTED, " + walkLength + " hops on a " + cycleLength + "-hop face)";
        }

        RigidBodyTransformation closingHop = measureInboundHop(pm.getSenderId());
        if (closingHop == null) {
            // Cannot verify honestly this tick. Leave the corner alone and let the walk be
            // re-offered rather than recording a verdict the robot did not reach.
            incomingMessages.add(pm);
            log("-> closing sender " + pm.getSenderId() + " not observable; deferring the verdict");
            return "Positioning Message from " + pm.getSenderId() + " (DEFERRED, sender unobservable)";
        }

        RigidBodyTransformation closed = cert.getMeasuredVoltage().compose(closingHop);
        if (!closed.isApproximatelyIdentity(CLOSURE_EXACTNESS, CLOSURE_EXACTNESS)) {
            // Loud, because this should be unreachable. The product telescopes, so the only
            // ways here are a walk that ended at a robot other than this one, or a robot that
            // moved mid-flight. Both are worth investigating rather than absorbing.
            log("-> ANOMALY: own certificate returned with the right length but a non-identity "
                    + "product " + closed.asPose() + ". The measured product telescopes, so this "
                    + "should be exactly zero; treating as a non-closure. See VoltageCertificate.");
            setCycleStatusOf(pm.getOriginOutgoingEdgeID(), CycleStatus.failed);
            forwardFailureUpstream(pm);
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, NON-IDENTITY PRODUCT)";
        }

        // The face is closed. Mark the corner from the origin edge the certificate names --
        // the edge this robot sent out. At the initiator, and only at the initiator, that is
        // the same edge as edgeOwedBy(its tuple); see markCornerFromStatus for why the two
        // spellings must not be confused anywhere else.
        // The closing hop names the corner before this one in the rotation order, and it is
        // occupied. The walk is cycleLength long -- checked above -- so the last relayer owed
        // prev(origin) and that is what it assigned; twin(prev(h)) is sigma^-1(h) by Edmonds'
        // rule, so this re-arms the corner that neighbours the one just closed, and the robot
        // that sent this message is standing on it.
        //
        // BEFORE the complete-mark, and that ordering is load-bearing. setCycleStatusOf fires
        // promoteAdjacentVerticesToRoots the moment hasFailed() holds, and announcedCorners
        // makes an announcement permanent. Marking first would, whenever this closure settles
        // the last open corner, promote on a failure verdict that the very next line then
        // overturns -- handing a neighbour rootship while this root still has work.
        rearmTwinOfIncomingEdge(pm.getAssignedOutgoingEdgeID());

        setCycleStatusOf(pm.getOriginOutgoingEdgeID(), CycleStatus.complete);

        self.addEdge(new Edge(pm.getRecipient(), pm.getSenderId()));
        forwardSuccessUpstream(pm.getSenderId(), pm.getOriginVertexID(),
                pm.getOriginOutgoingEdgeID(), cert);
        log("-> own certificate returned after " + walkLength + " hops on a " + cycleLength
                + "-hop face, product identity: face on edge " + pm.getOriginOutgoingEdgeID() + " CLOSED");
        return "Positioning Message from " + pm.getSenderId() + " (ACCEPTED, FACE CLOSED)";
    }

    /**
     * The closure rule, in one place and with no robot state in it so it can be tested
     * directly.
     *
     * <p>All three conjuncts are required. Identity says the walk came home; length says it
     * came home around the face it set out to trace, and not around some shorter or longer
     * one sharing this corner; the product says it came home to <em>this</em> robot rather
     * than to a robot standing where this one should be.
     *
     * @param initiatorId     the certificate's minter
     * @param selfId          the robot evaluating it
     * @param hops            hops the certificate accumulated in transit, closing hop excluded
     * @param faceCycleLength the boundary length of the face the origin edge belongs to
     * @param closedProduct   the accumulated transform after the evaluator's own closing hop
     */
    static boolean closesFace(int initiatorId, int selfId, int hops, int faceCycleLength,
                              RigidBodyTransformation closedProduct) {
        return initiatorId == selfId
                && hops + 1 == faceCycleLength
                && closedProduct != null
                && closedProduct.isApproximatelyIdentity(CLOSURE_EXACTNESS, CLOSURE_EXACTNESS);
    }

    /**
     * Takes on somebody else's walk so it can be carried onward next activation.
     *
     * <p>A root is a lattice site like any other, and this is the branch that replaces
     * defect 1. The old code, seeing a walk initiated by the sender, asked whether its own
     * next edge was already complete and emitted {@code AttemptLater} if not -- so roots all
     * the way around a face each deferred to the next and nothing ever closed. A relaying
     * root asks nothing about its own progress; only the initiator may judge the walk.
     */
    private String acceptForRelay(PositioningMessage pm) {
        // The hop cap stays where Phase 5 put it: at robots that verify certificates, which
        // is roots. A relaying cycleBuilder asks nothing about the walk it carries.
        if (role == CycleRole.root && exceedsHopCap(pm.getCertificate())) {
            forwardFailureUpstream(pm);
            log("-> certificate has taken " + pm.getCertificate().getHops() + " hops, past the "
                    + graph.maxCycleLength() + "-hop bound; failing it rather than relaying");
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, HOP CAP)";
        }

        int incomingEdgeId = pm.getAssignedOutgoingEdgeID();
        HalfEdge incomingEdge = retrieveEdgeFromGraph(incomingEdgeId);
        if (incomingEdge == null) {
            forwardRejectionUpstream(pm, false);
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, UNKNOWN EDGE)";
        }

        String collapsed = collapseCoInitiation(pm, incomingEdge);
        if (collapsed != null) {
            return collapsed;
        }

        FaceObligation onThisEdge = obligations.findByEdge(incomingEdgeId);
        if (onThisEdge != null && onThisEdge.getParentId() != pm.getSenderId()) {
            // This robot already owes this same edge to somebody else. A face is traversed in
            // one direction, so two different parents cannot both be handing this robot the
            // same incoming edge -- the offer is spurious and is refused rather than
            // clobbering a live tuple, which is what FaceObligationSet.getOrCreate asserts on.
            //
            // This can no longer fire merely because the robot is building its own face: that
            // lives in its own slot now, in its own key space.
            forwardRejectionUpstream(pm, false);
            log("-> edge " + incomingEdgeId + " is already owed to " + onThisEdge.getParentId()
                    + "; declining " + pm.getSenderId());
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, EDGE ALREADY OWED)";
        }

        if (obligations.carriedSize() >= maxConcurrentObligations() && onThisEdge == null) {
            forwardRejectionUpstream(pm, false);
            log("-> at the obligation cap (" + maxConcurrentObligations()
                    + "), declining to relay for " + pm.getSenderId());
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, AT OBLIGATION CAP)";
        }
        // A walk arriving over this edge is evidence that a robot occupies the site on the
        // far side of it, so a corner previously written off for want of a candidate is worth
        // re-arming. Kept from the branch this replaces.
        rearmTwinOfIncomingEdge(incomingEdgeId);

        // anchorParentID is deliberately NOT set here, for any role. It means "the robot my
        // pose is derived from", which is decided once, on the transition out of unassigned.
        // A root's pose is derived from nothing; an arrived cycleBuilder's is derived from
        // itself; and a second face passing through a settled robot must not re-point either.
        // The face's parent lives on the tuple, which is where it has to live once several
        // walks are in flight anyway.
        holdInCustody(pm, pm.getSenderId(), incomingEdgeId);
        self.addEdge(new Edge(pm.getRecipient(), pm.getSenderId()));

        log("-> carrying robot " + pm.getCertificate().getInitiatorID() + "'s walk onward from edge "
                + incomingEdgeId);
        return "Positioning Message from " + pm.getSenderId() + " (ACCEPTED FOR RELAY)";
    }

    /**
     * Resolves two robots independently building the same face, if that is what this arriving
     * walk is.
     *
     * <p>Every robot on a cycle sees the certificate before it closes, so co-initiation is
     * always detectable somewhere on the walk: this robot is initiating a face on edge
     * {@code e}, and a walk arrives owing the very same {@code e}. Lower initiator id wins,
     * matching {@link #outranks}. The winner's walk is carried; the loser drops its own tuple
     * and re-arms that corner as {@code attempted}, so it is retried later if the winner's
     * walk never closes.
     *
     * <p>Whoever loses may already have an offer out, and that child will answer into a tuple
     * that no longer exists. {@link #statusWithNoTuple} is where that lands, and it is the one
     * case it treats as expected rather than anomalous.
     *
     * @return a description if the arriving walk was refused, or null to carry on accepting it
     */
    private String collapseCoInitiation(PositioningMessage pm, HalfEdge incomingEdge) {
        // One comparison: does the arriving walk owe the very corner this robot is building?
        // The two are keyed in different spaces -- the walk on the edge it arrived over, the
        // attempt on the edge it builds -- so this asks the question directly rather than
        // inferring it from a key collision.
        HalfEdge owed = inferNextEdge(incomingEdge);
        FaceObligation mine = obligations.getAttempt();
        if (owed == null || mine == null || mine.getEdgeId() != owed.getId()) {
            return null;
        }

        int corner = edgeIdOwedBy(mine);
        int theirs = pm.getCertificate().getInitiatorID();
        if (!outranks(false, theirs, false, self.getRobotId())) {
            forwardRejectionUpstream(pm, false);
            log("-> robot " + theirs + " is building the same face on edge " + corner
                    + "; I hold the lower id, so I keep it");
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, I OUTRANK ON THIS FACE)";
        }

        undrawChildEdge(mine.getChildEdge());
        obligations.remove(mine);
        setCycleStatusOf(corner, CycleStatus.attempted);
        log("-> robot " + theirs + " is building the same face on edge " + corner
                + " and holds the lower id; standing down and carrying its walk");
        return null;
    }

    /**
     * Whether a certificate has already travelled further than any face in this lattice is
     * long, and so cannot still be tracing one.
     *
     * <p>This is the bound that replaced {@code AttemptLaterMessage}. That message was the
     * only thing stopping a walk wandering indefinitely, and Phase 5 deleted it, so the
     * replacement landed in the same change -- a certificate past the longest face is failed
     * by whoever holds it rather than passed on.
     */
    private boolean exceedsHopCap(VoltageCertificate cert) {
        return cert != null && cert.getHops() >= graph.maxCycleLength();
    }

    /**
     * Tells the parent that this robot could find nobody to carry the walk further, and drops
     * the tuple.
     *
     * <p>Notably it does <em>not</em> stand the robot down. That was the old builder
     * behaviour -- report failure, reset to unassigned -- and it was the chain collapse in
     * another guise: a dead end on one face says nothing about the site this robot occupies
     * or about the other faces incident to it. A root never stood itself down here for
     * exactly that reason; now nobody does.
     */
    private String reportRelayFailure(FaceObligation obligation, PositioningMessage held) {
        int parentId = obligation.getParentId();
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        if (parent != null) {
            send(parent, new StatusMessage(self.getRobotId(), parentId, false,
                    held.getOriginVertexID(), held.getOriginOutgoingEdgeID(),
                    held.getCertificate()));
        } else {
            log("-> cannot report a dead end to " + parentId + ": unreachable");
        }
        undrawChildEdge(obligation.getChildEdge());
        obligations.remove(obligation);
        releaseCustody(obligation, held.getCertificate());
        log("-> no candidate to carry the walk; reported FAILURE to " + parentId);
        return "Reporting Failure (No candidate to relay to, told " + parentId + ")";
    }

    /*
        ////////////////////////
        CERTIFICATE CUSTODY
        ////////////////////////

        A robot that accepts an assignment it has to travel to cannot relay for several ticks,
        and the certificate has to survive that wait somewhere. It used to live in a field,
        which held exactly one walk -- fine while the cap was one, and silently wrong the
        moment a robot carries two: whichever certificate arrived last would overwrite the
        other, and the older face would then be relayed with a certificate belonging to a
        different walk. A slot on the obligation has the same failure one level down, since
        two certificates can target one edge before either is forwarded, which is why
        FaceObligation asserts it holds none.

        So the certificate stays where it already is: in the PositioningMessage, in the inbox.
        `processMessages` simply stops consuming that message until the walk is discharged.
        The queue is a per-walk container by construction, ordering is preserved for free, and
        the accept path becomes idempotent rather than stateful.

        Custody outlasts the relay itself, deliberately. The plan says the message is consumed
        "at the moment its certificate is relayed onward"; it is consumed when the walk
        RESOLVES instead. The difference shows up on a rejection: the child hands the
        certificate back, the tuple is released, and the robot must re-offer -- which it can
        only do if it still holds the walk. Consuming at relay time would leave it with a
        released tuple and nothing to put in the next offer.
     */

    /**
     * Accepts a walk: opens (or reuses) the tuple for its edge and leaves the message queued.
     *
     * <p>{@code getOrCreate} rather than create, which is item 3 of the phase --
     * <em>intermediates never filter</em>. A second certificate arriving on an edge that
     * already has a tuple reuses that tuple rather than opening a rival one, and waits its
     * turn in the inbox; only the initiator is entitled to evaluate a walk, so an intermediate
     * has no business deciding between two.
     */
    private void holdInCustody(PositioningMessage pm, int parentId, int edgeId) {
        obligations.getOrCreate(parentId, edgeId);
        // Back into the inbox. The message HAS been polled by now -- processMessages consumed
        // it to get here -- so custody is not "leave it alone", it is "put it back". It goes
        // to the tail, which is what keeps a robot in transit rotating its queue instead of
        // blocking it. From the next activation onward pollNextActionable recognises it as
        // held and rotates past it without spending the tick budget.
        incomingMessages.add(pm);
    }

    /**
     * Whether this message is one this robot has already accepted and is holding.
     *
     * <p>Stated as "there is a tuple for this (parent, edge)" rather than tracked separately,
     * which is what makes the accept path idempotent: re-processing a held assignment cannot
     * re-run {@code resetToCycleBuilder}, re-point the anchor, or open a second tuple, because
     * it is never processed at all.
     */
    private boolean isHeldInCustody(PositioningMessage pm) {
        FaceObligation obligation = obligations.findByEdge(pm.getAssignedOutgoingEdgeID());
        return obligation != null && obligation.getParentId() == pm.getSenderId();
    }

    /** The queued assignment whose certificate this tuple is carrying, or null. */
    private PositioningMessage custodyFor(FaceObligation obligation) {
        for (AbstractMessage queued : incomingMessages) {
            if (queued instanceof PositioningMessage pm
                    && pm.getSenderId() == obligation.getParentId()
                    && pm.getAssignedOutgoingEdgeID() == obligation.getEdgeId()) {
                return pm;
            }
        }
        return null;
    }

    /** {@link #custodyFor} plus removal from the inbox, for a tuple that is going away. */
    private PositioningMessage takeCustody(FaceObligation obligation) {
        PositioningMessage held = custodyFor(obligation);
        if (held != null) {
            incomingMessages.remove(held);
        }
        return held;
    }

    /**
     * Consumes the queued assignment whose walk has now resolved.
     *
     * <p>Matched on the returning certificate's initiator where one is available, not merely
     * on the tuple's key. Two certificates can be queued for one edge -- that is the case
     * one-tuple-per-edge exists to make harmless -- and consuming the wrong one would discard
     * a walk that has not been carried yet while leaving a finished one to reopen the tuple.
     *
     * @param resolved the certificate that came back, or null if the walk was lost and
     *                 nothing came back to identify it
     */
    private void releaseCustody(FaceObligation obligation, VoltageCertificate resolved) {
        PositioningMessage fallback = null;
        for (AbstractMessage queued : incomingMessages) {
            if (!(queued instanceof PositioningMessage pm)
                    || pm.getSenderId() != obligation.getParentId()
                    || pm.getAssignedOutgoingEdgeID() != obligation.getEdgeId()) {
                continue;
            }
            if (resolved != null && pm.getCertificate() != null
                    && pm.getCertificate().getInitiatorID() == resolved.getInitiatorID()) {
                incomingMessages.remove(pm);
                return;
            }
            if (fallback == null) {
                fallback = pm;
            }
        }
        if (fallback != null) {
            incomingMessages.remove(fallback);
        }
    }

    /**
     * The next message this robot should actually act on, skipping anything in custody.
     *
     * <p>Rotates rather than consumes, and rotates to the <em>back</em>, so a robot in transit
     * cycles its inbox instead of blocking on it -- a held assignment costs nothing off the
     * one-message-per-tick budget and cannot starve the other faces incident to this robot.
     * Bounded by the queue length at entry, so a queue that is entirely custody terminates
     * rather than spinning.
     *
     * @return the message to handle, or null if there was nothing but held assignments
     */
    private AbstractMessage pollNextActionable() {
        for (int rotations = incomingMessages.size(); rotations > 0; rotations--) {
            AbstractMessage head = incomingMessages.poll();
            if (head == null) {
                return null;
            }
            if (head instanceof PositioningMessage pm && isHeldInCustody(pm)) {
                incomingMessages.add(pm);
                continue;
            }
            return head;
        }
        return null;
    }

    //ROOT-RELATED UTIL

    /**
     * The next corner of this root's own site worth starting a walk on, or -1.
     *
     * <p>Unattempted before attempted, so a corner that has already been tried and lost its
     * certificate waits behind ones that have never been tried at all.
     *
     * <p>Corners with a live tuple are skipped. That check is new in Phase 6 and is what
     * stops a root re-initiating a face it is already building: with the cap at one, a
     * fulfilled tuple blocked {@code sendMessage} outright, so this could never be reached
     * twice for the same edge. Without the cap it would be reached every tick, and the root
     * would offer the same corner to a new candidate over and over.
     */
    private int determineNextCycleToComplete() {
        int attempted = -1;
        for (Entry<Integer, CycleStatus> entry : completedCycles.entrySet()) {
            HalfEdge edge = retrieveEdgeFromGraph(entry.getKey());
            if (edge == null || hasLiveObligationForCorner(edge)) {
                continue;
            }
            if (entry.getValue() == CycleStatus.unattempted) {
                return entry.getKey();
            }
            if (entry.getValue() == CycleStatus.attempted && attempted == -1) {
                attempted = entry.getKey();
            }
        }
        return attempted;
    }

    /**
     * Gives this robot a corner to track for every edge leaving its role. Run once, at the
     * moment it becomes a root, and nowhere else.
     */
    private void initializeEdgeMap() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);
        for(HalfEdge edge : edges) {
            completedCycles.put(getEdgeIDof(edge), CycleStatus.unattempted);
        }
        // Nothing has been announced yet, because nothing has closed yet. This is the one
        // moment a robot starts being a root, so it is the one moment that record starts over.
        announcedCorners.clear();
    }

    /**
     * Tells the neighbour on each corner this robot closed that it may build outward.
     *
     * <p>The only mechanism that extends the frontier, and the only reason a lattice grows
     * past its first cell.
     *
     * <p><strong>Idempotent, which it did not need to be before.</strong> It used to be
     * reached at most once, on the single activation where a root discovered it had nothing
     * left to attempt, because a root that had given up stood down before getting here. Now
     * that a part-failed root reaches this branch every activation, re-sending would be a
     * message storm -- so each corner is announced once and remembered.
     *
     * <p>Never cleared, and nothing needs to clear it: a promoted neighbour becomes a root,
     * and nothing in this class demotes a root. Re-announcing a corner could therefore only
     * repeat something already true. {@link #initializeEdgeMap()} resets it, which is the one
     * moment a robot starts being a root at all.
     *
     * <p>The candidate lookup cannot fail. A corner is {@code complete} only because a walk
     * went out over that edge and came back, which means a robot reached {@code target(edge)};
     * a settled builder does not leave a site it is standing on; and robots do not fail. So
     * {@code findBestNeighborForEdge} finds it by exact position, and the null check that
     * would otherwise belong here would be asserting something the protocol already
     * guarantees.
     *
     * @return how many neighbours were told this time
     */
    private int promoteAdjacentVerticesToRoots() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);

        int promoted = 0;
        for(HalfEdge edge : edges) {
            int edgeId = getEdgeIDof(edge);
            if(completedCycles.get(edgeId) == CycleStatus.complete && announcedCorners.add(edgeId)) {
                GeometricCycleLatticeRobot neighbor = findBestNeighborForEdge(edge, null);
                PromotionMessage pm = new PromotionMessage(self.getRobotId(), neighbor.getRobotId(), getVertexIDof(edge), edgeId, !hasFailed());
                send(neighbor, pm);
                log("-> promoting neighbor on edge " + edgeId + " to root");
                promoted++;
            }
        }
        return promoted;
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
     * Picks a robot to take {@code targetEdge}, excluding anyone banned on that face.
     *
     * <p><strong>The walk's initiator gets no special treatment here any more.</strong> There
     * used to be a block above the candidate loop that looked the initiator up by id and
     * returned it if it sat at the target, plus a matching exclusion keeping it out of the
     * ordinary loop -- a hand-written "is this the closing hop?" test, needed back when
     * nothing else could answer that question. The certificate answers it now, and answers it
     * where it belongs: at the initiator, on accumulated voltage and hop count. So the
     * initiator is an ordinary candidate. If it is genuinely on the target the exact-position
     * branch below picks it, exactly as it picks any other already-placed neighbour, and the
     * face closes; if it is not, picking it would have been wrong, and the closure predicate
     * is what says so rather than a proximity test guessing here.
     *
     * @param obligation the face this offer belongs to, whose ban list scopes the
     *                   exclusions. Null means no exclusions -- the
     *                   {@code promoteAdjacentVerticesToRoots} path, which is not offering a
     *                   face at all.
     */
    private GeometricCycleLatticeRobot findBestNeighborForEdge(HalfEdge targetEdge, FaceObligation obligation) {
        OrientedPoint targetLocal  = getTargetInLocalCoordinates(targetEdge);
            log("Beginning decision process");


        // Who handed this robot the face, read off the obligation rather than off
        // anchorParentID. The two agree for a cycleBuilder carrying the face it was placed
        // for, and only the obligation is right for any other face passing through this site
        // -- a root has no anchor at all, so the field is -1 and the exclusion below would
        // silently stop excluding anyone.
        int parentID = obligation != null ? obligation.getParentId() : anchorParentID;

        ArrayList<Observation> validObservations = new ArrayList<>();
        ArrayList<Observation> priorityObservations = new ArrayList<>();

        for (Observation obs : observations.values()) {
            int robotID = obs.getId();
            // Walk-membership exclusion, narrowed twice over. It used to read
            // !certificate.isInList(robotID), excluding every robot already on the walk; the
            // certificate stopped carrying that roster in Phase 2, leaving the initiator and
            // the immediate parent; and the initiator's exclusion went with the closure
            // shortcut, since the certificate now decides closure and a proximity test here
            // cannot improve on it. What is left is the parent, which is a different kind of
            // rule: a robot must not offer the next edge back to the robot that just assigned
            // it, because that is a one-hop cycle rather than a face.
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
            // The interim deadlock this used to carry is CLOSED. The pending-child gate
            // rotated a PositioningMessage unread whenever the recipient had a child
            // recorded, so a mid-chain ancestor never reached the position check and never
            // sent that rejection -- offerer, ancestor and the robot between them then waited
            // on each other with no timeout, on any 4-cycle of 50-unit edges (OctagonSquare,
            // HexagonSquareTriangle, DodecagonHexagonSquare, ElongatedTriangular). Nothing
            // gates the inbox now, so a busy robot answers an offer it should decline instead
            // of rotating it unread, and the round trip above completes.
            // The ban check stays AHEAD of the exact-position match, and that ordering was
            // tried the other way round and reverted. Letting "this robot is standing on the
            // target" override its ban sounds right -- a ban is a claim from several ticks ago
            // and the observation is a measurement of now -- but it removes the only thing
            // bounding the offer loop. A robot on the target that still refuses for a reason
            // that is not about position (wrong role, a co-initiation stand-down, an obligation
            // it already owes) is re-offered the same edge every activation, refuses every
            // activation, and neither side ever moves on. The ban is what makes an offer
            // sequence finite; the evidence has to arrive some other way.
            if (robotID != parentID && !isBannedOn(obligation, robotID)) {
                // A neighbor already sitting exactly at the target position must win outright,
                // and before the wedge test runs: that test's
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
     * Re-arms a corner this robot had written off, on the evidence that somebody is standing
     * on it after all.
     *
     * <p>The argument is an edge a walk <em>arrived</em> over -- {@code assignedOutgoingEdgeID}
     * on a {@link PositioningMessage}, which is outgoing as the sender named it and incoming
     * as this robot receives it. Source and target swap across the twin, so
     * {@code twin(incoming)} is one of <em>this</em> role's outgoing edges, and its target is
     * the site the sender is standing on. A corner is only ever marked {@code failed} because
     * {@link #findBestNeighborForEdge} found nobody at its target; the arriving message is
     * proof that verdict has expired.
     *
     * <p><strong>Only {@code failed} is re-armed.</strong> The other two non-complete states
     * are live bookkeeping and resetting them loses information:
     * <ul>
     *   <li>{@code attempted} means "tried, deliberately parked at the back of the queue" --
     *       set by {@link #routeCertificateLostThroughTuple} so the root works its other
     *       corners before spinning on one short of candidates, and by
     *       {@link #collapseCoInitiation} when this robot stands down on a face a
     *       lower-id robot is already building. {@link #determineNextCycleToComplete()}
     *       prefers {@code unattempted}, so re-arming would jump the queue in the first case
     *       and re-launch a walk against the co-initiation winner in the second.</li>
     *   <li>{@code unattempted} is already armed, so the write is a no-op -- except that
     *       {@link #setCycleStatusOf} runs the promotion hook on every write, so it is not
     *       free.</li>
     * </ul>
     * A cycleBuilder tracks no corners, so this is a no-op there and needs no role test.
     *
     * @param incomingEdgeId the edge the arriving walk was assigned over, from this robot's
     *                       side; its twin is the corner considered for re-arming
     */
    public void rearmTwinOfIncomingEdge(int incomingEdgeId) {
        HalfEdge incomingEdge = retrieveEdgeFromGraph(incomingEdgeId);
        if (incomingEdge == null || incomingEdge.getTwin() == null) {
            // Both call sites establish the edge upstream -- acceptForRelay resolves it, and
            // the root path has already been through assignmentMatchesCurrentRole, which is
            // false for an unknown edge. Guarded anyway because this is public and the guard
            // no longer travels with the callers.
            return;
        }

        int outgoingEquivalentID = incomingEdge.getTwin().getId();
        if (completedCycles.get(outgoingEquivalentID) == CycleStatus.failed) {
            log("-> a walk arrived over edge " + incomingEdgeId + ", so somebody is standing on "
                    + "corner " + outgoingEquivalentID + " after all; re-arming it");
            setCycleStatusOf(outgoingEquivalentID, CycleStatus.unattempted);
        }
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
     * <p>Branches on <em>who minted the certificate</em>, not on role. A robot that
     * initiated the face is the only one that can mint a replacement, so there is nobody to
     * tell: it marks its own corner {@code attempted} and picks the face up on a later
     * pass. Keying this off {@code role == root} instead would be wrong for a root that is
     * relaying somebody else's walk -- it would swallow the loss and mark an unrelated
     * corner of its own, leaving the real initiator waiting forever.
     */
    private void reportCertificateLost(FaceObligation obligation) {
        PositioningMessage held = takeCustody(obligation);
        if (isInitiatedFace(obligation)) {
            setCycleStatusOf(edgeIdOwedBy(obligation), CycleStatus.attempted);
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
        // Origin ids off the walk in custody, not off the fields. The fields describe
        // whichever face this robot was placed for; the walk that just lost its certificate
        // may be a different one passing through the same site.
        send(parent, new CertificateLostMessage(self.getRobotId(), parent.getRobotId(),
                held == null ? originVertexID : held.getOriginVertexID(),
                held == null ? originOutgoingEdgeID : held.getOriginOutgoingEdgeID()));
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

    /** A null obligation means no exclusions -- see {@link #findBestNeighborForEdge}. */
    private static boolean isBannedOn(FaceObligation obligation, int robotID) {
        return obligation != null && obligation.isBanned(robotID);
    }

    /**
     * Passes a lost-certificate report to one named robot.
     *
     * <p>Takes the parent explicitly rather than reading {@code anchorParentID}, because the
     * robot upstream on a walk is not always the robot this one is anchored to -- a root has
     * no anchor at all, and a settled builder carrying a second face was never anchored to
     * that face's parent. It is read off the tuple by the caller.
     */
    private void forwardCertificateLostTo(int parentId, CertificateLostMessage cm) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        if (parent == null) {
            log("-> cannot forward lost certificate: parent " + parentId + " unreachable");
            return;
        }
        send(parent, new CertificateLostMessage(self.getRobotId(), parent.getRobotId(),
                cm.getOriginVertexID(), cm.getOriginOutgoingEdgeID()));
    }

    /**
     * This robot's attempt on a face of its own, opened on first use.
     *
     * <p>Keyed on the outgoing edge being built -- the same key {@code completedCycles} uses
     * for that corner, so a tuple and its cycle-status entry name the same thing and a log
     * line means what it says.
     *
     * <p>It briefly did not. Keying an initiated face on the outgoing edge while carried ones
     * key on the incoming edge puts the two in overlapping spaces, and a single-slot lookup
     * cannot hold both: on a single-role lattice a root building edge {@code e} could not be
     * offered edge {@code e} by a neighbour, because its own tuple sat on that key, and every
     * root in a ring refused every other. The fix was to key the initiated face on
     * {@code prev(e)} so that everything shared the incoming space -- which worked, at the
     * cost of a key that was not the edge being built.
     *
     * <p>Neither was necessary. The two kinds are not two flavours of one thing that need a
     * shared key; they are different things that were sharing a container. They now sit in
     * separate ones, so both keep the key that suits them and neither can collide with the
     * other. See {@link FaceObligationSet}.
     */
    private FaceObligation obligationForInitiatedFace(HalfEdge outgoingEdge) {
        return obligations.beginAttempt(outgoingEdge.getId());
    }

    /**
     * Whether this robot minted the certificate for the face this obligation serves, as
     * opposed to carrying someone else's.
     *
     * <p>This one predicate replaces every place the code used to branch on {@code role}, and
     * that substitution is most of what Phase 6 is. Role was never the right question -- a
     * root can relay, a settled builder can carry a second face -- and each site that asked
     * it got a different subset of the cases right. Whether a child's departure is reported
     * upstream or absorbed, whether an arriving status stops here or travels, whether a
     * failure writes off a corner, whether an offer mints a certificate or extends one: all
     * of them are properties of the <em>face</em>, and the face is the tuple.
     */
    private boolean isInitiatedFace(FaceObligation obligation) {
        return obligation.getParentId() == FaceObligation.NO_PARENT;
    }

    /**
     * The edge this obligation owes onward.
     *
     * <p>One branch, and it is the difference between the two kinds rather than a quirk of
     * either. A carried walk is keyed on the <em>incoming</em> edge it arrived over, so it
     * owes {@code next} of that -- the next edge round the face. An attempt is keyed on the
     * <em>outgoing</em> edge it is building, which is the edge it owes.
     */
    private HalfEdge edgeOwedBy(FaceObligation obligation) {
        HalfEdge keyed = retrieveEdgeFromGraph(obligation.getEdgeId());
        return isInitiatedFace(obligation) ? keyed : inferNextEdge(keyed);
    }

    /** {@link #edgeOwedBy} as an id, or -1 -- the form {@code completedCycles} is keyed by. */
    private int edgeIdOwedBy(FaceObligation obligation) {
        HalfEdge owed = edgeOwedBy(obligation);
        return owed == null ? -1 : owed.getId();
    }

    /** Whether this robot is already building a corner of its own. */
    private boolean hasInitiatedFaceInFlight() {
        return obligations.getAttempt() != null;
    }

    /** Whether a face is already in flight on this outgoing edge, so it must not be restarted. */
    private boolean hasLiveObligationForCorner(HalfEdge outgoingEdge) {
        for (FaceObligation obligation : obligations.asList()) {
            if (edgeIdOwedBy(obligation) == outgoingEdge.getId()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Takes a promotion, from whichever role this robot currently holds.
     *
     * <p><strong>A cycleBuilder used to defer this, and the deferral never released.</strong>
     * The bet it made was that the robot would free up shortly, which was true for as long as
     * a chain collapsed on its first status: a builder reported success, reset itself to
     * unassigned, and the re-queued promotion landed on the next pass. Phase 6 removed that
     * collapse -- a robot serving several faces has to survive the first one resolving -- and
     * the bet stopped paying. Since {@code promoteAdjacentVerticesToRoots} sends promotions to
     * exactly the robots parked on completed corners, and those are parked cycleBuilders, the
     * frontier could not expand past the first root: measured on a nine-robot square, 4
     * promotions sent, 0 delivered, 0 roots alive at the end, and 43% of every robot-tick in
     * the run spent rotating an undeliverable message through an inbox.
     *
     * <p>Roles have no business differing here. A root and a settled builder do the same work
     * -- route responses through their tuples, carry walks, offer the edges they owe -- and
     * differ only in whether they initiate a face of their own. A promotion grants exactly
     * that, so it is the same event whoever receives it.
     *
     * <p><strong>Nothing carried is lost.</strong> The tuples survive because
     * {@link #resetToRoot()} stopped clearing them in Phase 6; the certificates survive
     * because they live in the inbox, which no reset touches; the relay parents survive
     * because {@link #relayOnward} reads them off the tuple rather than off
     * {@code anchorParentID}. A promotion changes what a robot is <em>trying to do</em>, and
     * none of the carried state is about that.
     */
    private String acceptPromotion(PromotionMessage pm) {
        // The in-transit deferral that used to stand here is gone, absorbed by the gate at the
        // top of processMessages -- a cycleBuilder that has not arrived reaches no branch of
        // this switch at all, so the guard became unreachable rather than merely redundant.
        //
        // The hazard it existed for is real and is now covered more cheaply: a robot still
        // driving toward its site must not become an anchor, because a root's
        // getAssignedGlobalPosition short-circuits to its own pose, so promoting mid-journey
        // would make wherever it happens to be standing into its lattice site -- it stops
        // seeking, and every child it later places is offset from the real lattice. The gate
        // keeps the promotion queued, unread and uncounted, until arrival.

        // "Already a root" asked as the thing it actually means. Tracking corners IS what
        // being a root is -- initializeEdgeMap is what makes one -- so this reads the state
        // rather than the label, and gives the same answer for a root, a builder and an
        // unassigned robot without having to enumerate them.
        if (!completedCycles.isEmpty()) {
            if (pm.hasReachedStable()) {
                resetToRoot();
                reattemptFailedCycles();
                log("-> neighbour " + pm.getSenderId() + " reached stable; re-arming failed cycles");
                return "Promotion Message from " + pm.getSenderId()
                        + "(REACTIVATED, WILL ATTEMPT TO COMPLETE CYCLES)";
            }
            log("-> neighbour " + pm.getSenderId() + " has not reached stable; not re-arming");
            return "Promotion Message from " + pm.getSenderId()
                    + "(NOT REACTIVATED, WILL NOT ATTEMPT TO COMPLETE CYCLES)";
        }

        resetToRoot();
        stableID = pm.getSenderId();
        // The promotion carries the PROMOTER's outgoing edge, which is generally not the edge
        // this robot was assigned by its own parent. Overwriting is safe because both edges
        // terminate at this robot's physical site, and a site's role is a property of the
        // site -- so getCurrentRole(), and therefore the edge map built from it, is unchanged.
        // That rests on the promoter being adjacent and this robot being at target(edge),
        // which promoteAdjacentVerticesToRoots guarantees: it only promotes across a COMPLETE
        // corner, and a corner is complete only because a walk reached a robot standing there.
        setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
        initializeEdgeMap();
        log("-> promoted to root by robot " + stableID);
        return "Promotion Message from " + pm.getSenderId() + "(ACCEPTED)";
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
    public CycleStatus setCycleStatusOf(int outgoingEdgeID, CycleStatus status) { 
        CycleStatus statusToReturn = completedCycles.put(outgoingEdgeID, status);
        if(role == CycleRole.root && hasFailed()) {
            promoteAdjacentVerticesToRoots();
        }
        return statusToReturn;
    }
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
        // Anchors report themselves. A root and a stable always did; an arrived cycleBuilder
        // joins them as of Phase 6, because chains no longer collapse on success and it would
        // otherwise re-derive its pose from a live parent forever -- returning null the moment
        // that parent went out of sight, which is exactly the state canHonourStandAside acts
        // on. See the hasArrived javadoc. Exact rather than approximate: updateAssignedPosition
        // snaps the pose onto the target on arrival, so this freezes the robot on its ideal
        // site rather than wherever it stopped.
        if (role == CycleRole.root || role == CycleRole.stable
                || (role == CycleRole.cycleBuilder && hasArrived)) {
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
        dropEveryObligationAndItsCustody();
        this.anchorParentID = -1;
        this.hasArrived = false;
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
     * Clean slate for "unassigned": stableID, obligations and the assigned/origin edge
     * references are all cleared, and role is set to unassigned. hasBeenAssigned is the one
     * state-data field deliberately left untouched. Differs from the existing reset() above
     * only in that it also clears stableID.
     *
     * <p><strong>This is the vacate path.</strong> A robot reaching here is giving up its
     * lattice site -- contention yield, liveness give-up, or finding its assignment already
     * occupied -- and the tuples it holds describe a local topology that is about to stop
     * being true, so all of them go, along with the certificates held in custody for them.
     * Callers send their rejections <em>before</em> calling this, via
     * {@link #forwardRejectionToParent(boolean)}, which is why they carry that ordering
     * comment; the cap-of-one shortcut that used to make one rejection stand for the whole
     * set is gone with the cap.
     *
     * <p>Note that the ban list is not cleared by hand here: bans live on the obligation, so
     * they go when it does.
     */
    public void resetToUnassigned() {
        // The latched detour was chosen against a target this reset is discarding.
        policy.clearDetour();
        policy.resetProgress();
        this.stableID = -1;
        dropEveryObligationAndItsCustody();
        this.anchorParentID = -1;
        this.hasArrived = false;
        this.role = CycleRole.unassigned;

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Clean slate for "cycleBuilder": stableID, obligations and both the assigned and origin
     * edge references are cleared, and role is set to cycleBuilder. Meant to be immediately
     * followed by setAssignedEdge/setOriginEdge and the caller's own custody call, the same
     * way the unassigned -> cycleBuilder PositioningMessage handling does.
     *
     * <p>Obligations go for the same reason as in {@link #resetToUnassigned()}: accepting a
     * <em>first</em> assignment means moving to a different lattice site, so the topology the
     * old tuples described stops being true. This is not the promotion case -- a promotion
     * leaves the robot where it is and keeps its tuples -- and, as of Phase 6, it is not the
     * second-face case either: a settled builder taking on another walk goes through
     * {@code acceptForRelay} and never comes here.
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
        dropEveryObligationAndItsCustody();
        this.hasBeenAssigned = true;
        this.anchorParentID = -1;
        this.hasArrived = false;
        this.role = CycleRole.cycleBuilder;

        this.assignedVertexID = -1;
        this.assignedOutgoingEdgeID = -1;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Re-sets the role to root and clears the origin edge reference. stableID, the assigned
     * edge and completedCycles are deliberately untouched -- they are the root's identity,
     * position and record of progress, none of them per-edge state.
     *
     * <p><strong>Obligations survive this now.</strong> Clearing them was a cap-of-one
     * shortcut, marked as one in Phase 4 and widened in Phase 5, and this is where the plan
     * says to undo it. A root has not moved, so by the position-not-role principle its tuples
     * are still true; dropping them silently discarded every response the root still owed on
     * its other faces, and dropped any walk it was carrying for a neighbour. At a cap of one
     * there was never more than the single tuple this reset was finishing, so the bug had
     * nothing to bite.
     *
     * <p>Which leaves this method with almost nothing to do, and that is the honest outcome:
     * "finishing an edge" was never a state transition, it was one tuple being removed by
     * whichever response resolved it.
     */
    public void resetToRoot() {
        // The latched detour was chosen against a target this reset is discarding.
        policy.clearDetour();
        policy.resetProgress();
        // A promoted robot is an anchor; it has no business still stepping aside.
        policy.cancelEvasion();
        this.anchorParentID = -1;
        this.role = CycleRole.root;

        this.originVertexID = -1;
        this.originOutgoingEdgeID = -1;
    }

    /**
     * Empties the obligation set and the inbox custody that goes with it.
     *
     * <p>The two have to move together. A queued assignment whose tuple has been cleared is
     * no longer recognised as held, so {@code pollNextActionable} would hand it back to
     * {@code processMessages} as a fresh offer -- and this robot would accept, from a parent
     * it has just walked away from, a site it no longer occupies.
     */
    private void dropEveryObligationAndItsCustody() {
        for (FaceObligation obligation : obligations.drainForVacate()) {
            takeCustody(obligation);
        }
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
                stableID,
                getAssignedEdge(),
                getOriginEdge(),
                Map.copyOf(completedCycles),
                snapshotQueueInOrder(),
                Map.copyOf(observations),
                List.copyOf(obligations.asList()),
                graph
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
