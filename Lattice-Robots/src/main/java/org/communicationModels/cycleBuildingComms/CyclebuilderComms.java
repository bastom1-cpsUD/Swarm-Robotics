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
     * <p><strong>The carried links are permanent.</strong> One per incoming edge, opened by the
     * first walk to arrive over that edge and kept for as long as this robot occupies the site.
     * That is what makes them <em>links</em> rather than per-walk commitments: a robot in
     * formation has fixed lattice neighbours, so {@code (parent, edge) -> child} is a standing
     * fact, and the next certificate over that edge goes straight to the same child with no
     * candidate search at all. Only two things end one -- the child leaving range, which releases
     * the binding, and this robot vacating its site, which drops the lot.
     *
     * <p><strong>The attempt is the one transient entry, and the only in-flight gate.</strong> A
     * root opens it when it picks a corner to build and it is dropped when that corner's status
     * comes home. It stops this root starting a <em>second</em> face of its own; it does not stop
     * anything being relayed. A certificate arriving from a neighbour that owes the very corner
     * this root is building is forwarded like any other -- two walks on one face are redundant,
     * not conflicting.
     *
     * <p>There is no cap and no capacity check. There was one, refusing a walk when the carried
     * count reached the role's degree, and it was one of the three refusals that produced the
     * livelock; with permanent links it would also become permanently true. What it was really
     * guarding is one link per edge, and {@link FaceObligationSet#getOrCreate} guards that
     * structurally.
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
        // And nothing rotates any more either. There used to be a custody skip here: a
        // PositioningMessage this robot had accepted stayed queued while it travelled, and
        // pollNextActionable rotated past it by asking isHeldInCustody -- which recognised custody
        // as "a tuple exists for this (sender, edge)". Tuples are PERMANENT now, so that test is
        // true for every established link, and every future assignment arriving over a working
        // link would be rotated past unread forever.
        //
        // Nothing replaces it, because nothing needs to. The gate above returns BEFORE this line,
        // so a robot in transit polls nothing at all and the assignment it accepted simply sits in
        // the queue until it arrives. The queue does the holding; no marker is needed to recognise
        // what it is holding.
        //
        // That accepted assignment was consumed to be acted on, so holdInCustody puts it back, at
        // the TAIL. Two questions that raises, both answered:
        //
        //   Can it be re-processed as a fresh acceptance? No. resetToCycleBuilder and the anchor
        //   are set on the unassigned branch only; by the time it is read again the robot is a
        //   cycleBuilder, so it falls through to getOrCreate, finds the tuple it already opened
        //   for that (parent, edge), and relays.
        //
        //   Can the reorder conflict with an assignment queued behind it? No. Whichever is read
        //   first, each is answered against where this robot is STANDING: one naming a different
        //   site is refused because it genuinely cannot be there, and one naming this same site is
        //   a second face through it and gets its own tuple on its own edge. Neither answer
        //   depends on which was read first, and FIFO drains one per tick, so the re-queued
        //   message advances every tick and cannot be starved.
        //
        // Tail rather than head is deliberate: the other two deferrals below -- a closing sender
        // or a parent that is momentarily unobservable -- must not block the whole queue behind a
        // neighbour that has drifted out of view.
        AbstractMessage next = incomingMessages.poll();
        if (next == null) {
            return "N/A (EMPTY)";
        }

        // A message from a robot that has drifted out of observation range is dropped, and nothing
        // is done to recover it. That is not a hole, and it deliberately has no timeout behind it:
        // loss of contact is detected from BOTH ends, and the other end is the one that can act.
        //
        // Whatever this message was, the robot that sent it is a parent or a child of some walk.
        // The parent side notices independently -- collectDepartedChildren sweeps every link whose
        // child is no longer observable, releases the binding, and raises a CertificateLostMessage
        // addressed to that walk's initiator, which is the only robot able to mint a replacement.
        // So the face is relaunched by the one participant with the standing to relaunch it, and a
        // recovery path here would be a second mechanism racing the first.
        //
        // Measured at 3 drops in a 600-tick, 20-robot run, all of them robots moving apart while a
        // message was in flight.
        if(!validateSenderIsNeighbor(next.getSenderId())) {
            log("-> sender " + next.getSenderId() + " is not a neighbor, discarding message");
            return "N/A (Discarded message from non-neighbor " + next.getSenderId() + ")";
        }

        log("Received " + next.getMessageType() + " from robot " + next.getSenderId());

        // ONE rule per message type, at every role.
        //
        // Four branches used to stand here -- unassigned, cycleBuilder, root, stable -- and every
        // difference between them was a place a robot standing on the correct site could still be
        // refused, or a place a walk was judged by something other than its own certificate. A
        // root answered an assignment by arbitrating co-initiation; a stable robot answered by
        // declaring SUCCESS on role and pose alone, without asking whose certificate it was
        // holding. Both are gone. A lattice site is a lattice site: it answers for where it is
        // standing, judges only what it minted, and passes everything else along the link.
        if (next instanceof PositioningMessage pm) {
            return handleAssignment(pm);
        } else if (next instanceof StatusMessage sm) {
            return routeStatusThroughTuple(sm);
        } else if (next instanceof CertificateLostMessage cm) {
            return routeCertificateLostThroughTuple(cm);
        } else if (next instanceof RejectAssignmentMessage rm) {
            return routeRejectionThroughTuple(rm);
        } else if (next instanceof PromotionMessage pm) {
            return acceptPromotion(pm);
        }

        log("-> unexpected message type: " + next.getMessageType());
        return "N/A (Unhandled message type: " + next.getMessageType() + ")";
    }

    /**
     * The whole of what a robot does with an assignment, at every role.
     *
     * <pre>
     *   1. not to where I am standing        -> deny, send a rejection
     *   2. a certificate I minted            -> judge it, and report on it
     *   3. a link already exists for this    -> pass it along that link
     *   4. otherwise                         -> open a link and pass it along that
     * </pre>
     *
     * <p>Steps 3 and 4 are one call, because {@link FaceObligationSet#getOrCreate} is what makes
     * them one: the link either exists or is opened, and either way the walk is carried. Nothing
     * between step 1 and the end of step 4 can refuse.
     *
     * <p>That is the entire fix for the livelock. Every refusal that used to live in here --
     * co-initiation arbitration, "this edge is already owed", "I am at my obligation cap" -- fired
     * at a robot standing <em>exactly on the site being offered</em>, and every one of them was
     * non-retryable. The offerer banned the one robot that could occupy that site, worked down its
     * remaining candidates (each also not on the site, each also refusing), and wrote the corner
     * off. A temporary condition became a permanent verdict.
     */
    private String handleAssignment(PositioningMessage pm) {
        // (1) Is this assignment to where I am standing? A robot that has never been placed has no
        // lattice site to contradict, and is being recruited rather than re-assigned -- that is
        // how the formation grows, so it is exempt rather than an exception.
        if (occupiesLatticeSite() && !checkAssignmentForCurrentPosition(pm)) {
            forwardRejectionUpstream(pm, false);
            log("-> assignment REJECTED by " + pm.getSenderId() + " (not my position)");
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, NOT MY POSITION)";
        }

        // Companion to (1), and it catches what (1) cannot: an edge whose geometry coincides with
        // this robot's pose but which lands on a different role. Applied wherever the role is
        // known -- getCurrentRole() substitutes the primary role when there is no assigned edge,
        // so asking it of a robot that has none would refuse every legitimate offer on a
        // multi-role lattice.
        if (getAssignedEdge() != null && !assignmentMatchesCurrentRole(pm)) {
            forwardRejectionUpstream(pm, false);
            log("-> assignment REJECTED by " + pm.getSenderId() + " (edge lands on the wrong role)");
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, WRONG ROLE)";
        }

        // (2) My own certificate, home again. This -- and not "am I a root standing in the right
        // place" -- is what decides closure. Only the minter may judge a walk.
        if (mintedHere(pm.getCertificate())) {
            return evaluateReturningCertificate(pm);
        }

        // Recruitment. An unassigned robot takes the site and drives to it; it cannot relay until
        // it gets there, so the assignment goes back in the queue and the gate at the top of
        // processMessages holds it until arrival.
        if (role == CycleRole.unassigned) {
            return acceptFirstAssignment(pm);
        }

        // (3)/(4)
        return relayAlongTuple(pm);
    }

    /**
     * Whether this robot is standing on a lattice site it is entitled to defend.
     *
     * <p>True for a root, a stable, and an arrived cycleBuilder -- and for a robot that has been
     * dropped back to unassigned but has not moved off the spot it last held. False only for a
     * robot that has never been placed, which is exactly the robot an assignment is allowed to
     * recruit to somewhere it is not yet standing.
     *
     * <p>An in-transit cycleBuilder is not considered, because it never reaches here: the gate at
     * the top of {@link #processMessages(int)} answers nothing while a robot is still moving.
     */
    private boolean occupiesLatticeSite() {
        return role != CycleRole.unassigned || hasBeenAssigned;
    }

    /**
     * Takes a first assignment: adopts the site, opens the link back to the parent, and puts the
     * assignment back in the queue to be relayed once this robot has actually arrived.
     */
    private String acceptFirstAssignment(PositioningMessage pm) {
        resetToCycleBuilder();
        setAssignedEdge(pm.getAssignedVertexID(), pm.getAssignedOutgoingEdgeID());
        setOriginEdge(pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
        // The parent used to be banned here, into a robot-scoped exclusion list. It is now
        // excluded structurally by anchorParentID in findBestNeighborForEdge, which is both
        // narrower and impossible to forget.
        //
        // Set here and nowhere else: this is the only transition that decides which robot this one
        // is anchored to. See the anchorParentID javadoc.
        anchorParentID = pm.getSenderId();
        // A builder tracks the corners of the site it is taking, from the moment it takes it.
        //
        // AFTER setAssignedEdge, necessarily: initializeEdgeMap reads getCurrentRole(), which
        // resolves through getAssignedEdge().getTarget(), so the edge has to be in place or the
        // map would be built for the primary role instead of this robot's.
        //
        // Doing this on acceptance rather than on arrival is safe because processMessages returns
        // early for a builder that has not arrived -- nothing can write into the map while the
        // robot is still travelling to the site the map describes.
        initializeEdgeMap();
        self.addEdge(new Edge(self.getRobotId(), anchorParentID));
        holdInCustody(pm, pm.getSenderId(), pm.getAssignedOutgoingEdgeID());
        log("-> became cycleBuilder: edge id=" + getAssignedEdge().getId()
                + " from vertex " + getAssignedEdge().getOrigin().getId()
                + ", parent=" + anchorParentID
                + ", cert=" + pm.getCertificate());
        return "Positioning Message from " + pm.getSenderId() + "(ACCEPTED)";
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
                    // -1 means every corner is settled, and now it means only that. It used to also
                    // mean "busy on all of them", because determineNextCycleToComplete skipped
                    // corners with a live tuple -- which needed the guard below, testing whether
                    // any obligation was still held. Persistent links make that guard permanently
                    // true and the whole robot stuck, which is the shape the old livelock ended in:
                    // every root reporting "Every remaining cycle is already in flight" while
                    // nothing closed. In-flight is the attempt slot's business, and it was already
                    // asked, above.

                    // Announce the corners that closed BEFORE deciding what any failure means for
                    // this robot: a corner that closed is a real face with a real robot on the far
                    // side of it, and that robot's right to build outward does not depend on how
                    // its neighbour's other corners went.
                    boolean allMatched = promoteAdjacentVerticesToRoots();

                    // Stable needs BOTH: every corner complete, and every one of them handed on to
                    // the robot standing there. A root whose corners all closed but whose occupants
                    // are not all observable has not finished the handing-on, so it stays a root
                    // and tries again -- rather than declaring itself finished and taking the
                    // frontier's only route outward with it.
                    if (!hasFailed() && allMatched) {
                        promoteSelfToStable();
                        log("-> all cycles completed and all neighbours promoted, promoting self to stable");
                        return "Done (All cycles completed, promoted to stable)";
                    }
                    if (!hasFailed()) {
                        return "N/A (All cycles complete, waiting to see every corner's occupant)";
                    }

                    // A root that stands down here has still handed on whatever it built, and does
                    // so with hasReachedStable = false, which is what keeps its neighbours
                    // re-arming failed cycles.
                    return "N/A (Ceased operations due to failure)";
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
     * Carries somebody else's walk one hop further, along the link for the edge it arrived over --
     * opening that link if this is the first walk to use it.
     *
     * <p><strong>Inline, in the same activation the assignment is read.</strong> Relaying used to
     * be deferred: the message was pushed back into the inbox and picked up next tick by an
     * obligation-servicing loop in {@code sendMessage}. That indirection existed to hold a
     * certificate for a robot that could not yet act on it, and the only robot that still cannot is
     * one driving to its first site -- which never reaches this method, because the gate at the top
     * of {@link #processMessages(int)} answers nothing while it moves. Every robot that gets here
     * is standing still on its own site and can relay now.
     *
     * <p><strong>An established link is not re-shopped.</strong> If the tuple already names a
     * child, the walk goes to that child with no candidate search at all -- which is the point of
     * keeping tuples: once a robot is in formation its lattice neighbours are fixed, so the link is
     * a standing fact rather than something to re-derive per walk. The search runs only for a link
     * being opened, or one whose child has gone out of range.
     *
     * <p><strong>Nothing here filters, and duplicate walks are expected.</strong> Two certificates
     * tracing one face visit the same sites in the same direction and share every link along it.
     * They are carried one after the other and each closes at its own initiator; a corner that is
     * already complete makes the later one a no-op. Refusing one, or holding it until the other
     * resolved, was tried and produced the two failures this rewrite exists to remove.
     */
    private String relayAlongTuple(PositioningMessage pm) {
        int incomingEdgeId = pm.getAssignedOutgoingEdgeID();
        HalfEdge incomingEdge = retrieveEdgeFromGraph(incomingEdgeId);
        if (incomingEdge == null) {
            forwardRejectionUpstream(pm, false);
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, UNKNOWN EDGE)";
        }

        // The hop cap stays where Phase 5 put it: at robots that verify certificates, which is
        // roots. A relaying cycleBuilder asks nothing about the walk it carries.
        if (role == CycleRole.root && exceedsHopCap(pm.getCertificate())) {
            forwardFailureUpstream(pm);
            log("-> certificate has taken " + pm.getCertificate().getHops() + " hops, past the "
                    + graph.maxCycleLength() + "-hop bound; failing it rather than relaying");
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, HOP CAP)";
        }

        // Extend by the hop INTO this robot -- T(parent -> me), not T(me -> child). The child has
        // not moved yet, so measuring outbound would sample a pose that does not exist. Both
        // endpoints of the inbound hop are settled: the parent relayed earlier, and this robot is
        // standing on its own site or it would not have got past the gate.
        RigidBodyTransformation inboundHop = measureInboundHop(pm.getSenderId());
        if (inboundHop == null) {
            // A certificate extended with a guessed hop certifies nothing. Put the assignment back
            // and retry when the parent is visible again; the tail placement keeps the rest of the
            // queue moving meanwhile.
            incomingMessages.add(pm);
            log("-> parent " + pm.getSenderId()
                    + " not observable, cannot extend certificate this tick");
            return "N/A (Parent " + pm.getSenderId() + " unobservable, certificate not extended)";
        }

        // A walk arriving over this edge is evidence that a robot occupies the site on the far side
        // of it, so a corner previously written off for want of a candidate is worth re-arming.
        rearmTwinOfIncomingEdge(incomingEdgeId);

        // anchorParentID is deliberately NOT set here, for any role. It means "the robot my pose is
        // derived from", which is decided once, on the transition out of unassigned. A root's pose
        // is derived from nothing; an arrived cycleBuilder's is derived from itself; and a second
        // face passing through a settled robot must not re-point either. The face's parent lives on
        // the tuple, which is where it has to live once several walks are in flight anyway.
        FaceObligation tuple = obligations.getOrCreate(pm.getSenderId(), incomingEdgeId);
        // Runs on every walk over this link, not once per link, and that is fine: addEdge is a
        // set-add, now that Edge has value equality.
        self.addEdge(new Edge(pm.getRecipient(), pm.getSenderId()));

        HalfEdge owed = edgeOwedBy(tuple);
        if (owed == null) {
            log("-> no next edge from " + incomingEdgeId + "; reporting FAILURE to " + pm.getSenderId());
            return reportRelayFailure(tuple, pm.getCertificate(),
                    pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
        }

        // Whoever is actually standing on the site carries the walk, ahead of whoever this link
        // happens to name.
        //
        // Reusing the recorded child unconditionally is what a persistent link is for, and it is
        // wrong in exactly one case that turns out to be common: the binding was made by
        // findBestNeighborForEdge, which picks the NEAREST candidate rather than an occupant, so it
        // can name a robot that was merely closest at the time. If some other robot has since
        // settled onto that site, re-sending the recorded child there is an assignment to a spot
        // that is already taken -- two robots driving at one lattice site, which the old code
        // avoided only because it re-ran the search, and its exact-position branch found the
        // occupant, on every single walk.
        //
        // A banned occupant is still skipped. That keeps the offer sequence finite, which is the
        // property findBestNeighborForEdge deliberately puts its ban check ahead of its
        // exact-position match to preserve -- and it is safe here only because bans are now scoped
        // to a single walk and cleared when it resolves.
        GeometricCycleLatticeRobot occupant = findNeighborStandingOn(owed);
        GeometricCycleLatticeRobot child = null;
        if (occupant != null && !tuple.isBanned(occupant.getRobotId())) {
            child = occupant;
        } else if (tuple.getChildId() != null) {
            child = getNeighborByID(tuple.getChildId());
        }
        if (child == null) {
            child = findBestNeighborForEdge(owed, tuple);
        }
        if (child == null) {
            log("-> NO candidate found, reporting FAILURE to " + tuple.getParentId());
            return reportRelayFailure(tuple, pm.getCertificate(),
                    pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID());
        }

        VoltageCertificate onward = pm.getCertificate().extend(inboundHop);
        log("-> relaying robot " + onward.getInitiatorID() + "'s walk to " + child.getRobotId()
                + " on edge " + owed.getId());
        offer(tuple, child, owed, owed.getId(),
                pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), onward);
        return "Assigned position to robot " + child.getRobotId()
                + " for edge " + owed.getId() + " of vertex " + owed.getOrigin().getId();
    }

    /**
     * Makes the one offer this robot owes this activation, or reports that it owes none.
     *
     * <p>Only ever this robot's <em>own</em> face now. Carried walks are relayed inline by
     * {@link #relayAlongTuple}, in the activation their assignment is read, so this path no longer
     * rotates over a set of obligations looking for one to serve -- there is at most one thing it
     * can be serving, and {@link FaceObligationSet#getAttempt()} names it directly.
     *
     * @return a description for the tick log, or null if there was nothing outstanding
     */
    private String serviceOneObligation() {
        FaceObligation attempt = obligations.getAttempt();
        if (attempt == null || !attempt.isUnfulfilled()) {
            return null;
        }
        return initiateOnward(attempt);
    }

    /** Sends the first offer on a face this robot is building itself, minting the certificate. */
    private String initiateOnward(FaceObligation obligation) {
        HalfEdge targetEdge = edgeOwedBy(obligation);
        if (targetEdge == null) {
            obligations.clearAttempt();
            return "N/A (Obligation names an unknown edge " + obligation.getEdgeId() + ")";
        }

        GeometricCycleLatticeRobot child = findBestNeighborForEdge(targetEdge, obligation);
        if (child == null) {
            // A dead end on a face this robot started. There is no status lap to wait for -- the
            // walk died here, at its own minter -- so the corner is written off directly. This is
            // the one place an initiator marks its own corner without a status coming home, and it
            // is safe for exactly that reason: no message is in flight to contradict it.
            log("Ran out of options for building cycle on edge " + targetEdge.getId()
                    + ", failing edge and moving on");
            setCycleStatusOf(targetEdge.getId(), CycleStatus.failed);
            obligations.clearAttempt();
            return "Failed (No valid neighbors for cycle on edge " + targetEdge.getId() + ")";
        }

        VoltageCertificate minted = new VoltageCertificate(self.getRobotId());
        offer(obligation, child, targetEdge, targetEdge.getId(),
                getVertexIDof(targetEdge), getEdgeIDof(targetEdge), minted);
        return "Assigned position to robot " + child.getRobotId()
                + " for edge " + targetEdge.getId() + " of vertex " + targetEdge.getOrigin().getId();
    }

    /**
     * Sends one assignment along a link, binding the child and drawing the edge for it.
     *
     * <p>Both are unconditional and both are idempotent, which is what lets this run on every
     * certificate rather than once per link: {@code fulfil} writes an id, and {@code addEdge} is a
     * set-add because {@link Edge} carries value equality.
     */
    private void offer(FaceObligation obligation, GeometricCycleLatticeRobot child,
                       HalfEdge targetEdge, int loggedEdgeId,
                       int originVertexID, int originEdgeID, VoltageCertificate cert) {
        PositioningMessage pm = new PositioningMessage(self.getRobotId(), child.getRobotId(),
                getVertexIDof(targetEdge), getEdgeIDof(targetEdge),
                originVertexID, originEdgeID, cert);
        send(child, pm);
        obligation.fulfil(child.getRobotId());
        self.addEdge(new Edge(self.getRobotId(), child.getRobotId()));
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
        face it had initiated from one it was merely carrying.

        A response is now routed by ONE question: did I mint the walk it reports on? A status
        and a lost-certificate report both carry `initiatorId` for exactly that, and both stop
        at the robot it names. Everywhere else they are passed along the link, child -> parent,
        and the link is KEPT -- it describes a lattice adjacency that is still true whatever
        happened to the walk.

        A tuple is bidirectional and the direction is decided by message type: an assignment
        reads parent -> child, a status or a lost certificate reads child -> parent. There is
        exactly one outgoing half-edge per connection, so a child id names its tuple uniquely
        and `findByChild` needs no disambiguating key.

        A rejection is the exception, and deliberately not a "response" in this sense: it
        answers the robot that made the offer and travels no further. See there.
     */

    /**
     * Routes a status back down the chain the walk came up, and stops it at the robot that
     * minted that walk.
     *
     * <p><strong>A status is born at the initiator and dies at the initiator.</strong> A
     * certificate laps its face and returns to its minter as an assignment; the minter judges it
     * and emits a status to whoever handed it back; the status then walks the chain
     * <em>backwards</em> -- each robot forwarding to the parent of the tuple whose child sent it --
     * until it reaches the minter again. {@link StatusMessage#getInitiatorId()} is what recognises
     * that arrival. Without it the status has nothing to stop on and circulates forever.
     *
     * <p>The second lap is not ceremony. It is what lets every robot on the face record the outcome
     * against its own corner, which is what makes a duplicate certificate on a face that is already
     * built a no-op rather than a second attempt.
     *
     * <p><strong>The tuple survives.</strong> It used to be removed here, which was right when a
     * tuple was a per-walk commitment and wrong now that it is a standing communication link: the
     * robots either side of it have not moved, so the adjacency it records is still true and the
     * next certificate over that edge should go straight to the same child.
     */
    private String routeStatusThroughTuple(StatusMessage sm) {
        if (sm.getInitiatorId() == self.getRobotId()) {
            return settleOwnWalk(sm);
        }

        FaceObligation tuple = carriedLinkOwing(sm.getViaEdgeId());
        if (tuple == null) {
            log("-> ANOMALY: status from " + sm.getSenderId() + " for robot "
                    + sm.getInitiatorId() + "'s walk matches no link this robot holds");
            return "Status Message from " + sm.getSenderId() + "(ANOMALY, NO MATCHING TUPLE)";
        }

        markCornerFromStatus(tuple, sm);

        // The binding is NOT released, on either verdict, and a failure is the case that makes the
        // difference. Releasing on failure -- which is what the pre-tuple code did, when a status
        // tore the tuple down anyway -- breaks the link out from under every OTHER walk in flight
        // over it. Duplicate walks on one face share every link along that face by construction, so
        // the second walk's status then arrives to find no child recorded, matches nothing, and is
        // logged as an anomaly while its face hangs. Neither robot moved; the adjacency is intact;
        // only a walk ended.
        tuple.clearForResolvedWalk();

        // Verbatim onward: the verdict, the origin edge and the initiator are none of them this
        // robot's to reinterpret.
        int parentId = tuple.getParentId();
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        if (parent == null) {
            log("-> cannot pass a status back to " + parentId + ": unreachable");
            return "Status Message from " + sm.getSenderId() + "(PARENT UNREACHABLE)";
        }
        // Verdict, origin and initiator verbatim; the via-edge re-stamped to this robot's own key,
        // because it describes THIS hop and not the one below it.
        send(parent, new StatusMessage(self.getRobotId(), parentId, sm.isSuccessful(),
                sm.getOriginVertexID(), sm.getOriginOutgoingEdgeID(),
                sm.getInitiatorId(), tuple.getEdgeId(), sm.getCertificate()));
        log("-> passed a " + (sm.isSuccessful() ? "SUCCESS" : "FAILURE") + " back to " + parentId
                + " for robot " + sm.getInitiatorId() + "'s walk");
        return "Status Message from " + sm.getSenderId()
                + (sm.isSuccessful() ? "(SUCCESS, RELAYED)" : "(FAILURE, RELAYED)");
    }

    /**
     * A status for a walk this robot minted, having come the whole way round and back.
     *
     * <p><strong>This, and not {@code evaluateReturningCertificate}, is where the corner is
     * marked.</strong> Judging the certificate and recording the verdict are two events one lap
     * apart: the certificate coming home says the face closed, and the status coming home says
     * every robot on it has been told. Marking at the earlier moment would end the attempt while
     * the status was still travelling, and the corner would be re-opened underneath a lap already
     * in progress.
     *
     * <p>It also settles <em>which</em> corner without needing anything on the wire to say so. The
     * attempt is still open when its status returns -- that is the invariant this ordering buys --
     * so it is the only candidate, and {@code edgeIdOwedBy} of it is this robot's own outgoing
     * edge by construction.
     */
    private String settleOwnWalk(StatusMessage sm) {
        FaceObligation attempt = obligations.getAttempt();
        if (attempt == null) {
            // Nothing to settle. Reachable when a second certificate for the same corner laps
            // behind the first: the corner is already recorded and this is old news.
            log("-> my own status returned for a walk with no open attempt; already settled");
            return "Status Message from " + sm.getSenderId() + "(DROPPED, ALREADY SETTLED)";
        }

        int corner = edgeIdOwedBy(attempt);
        setCycleStatusOf(corner, sm.isSuccessful() ? CycleStatus.complete : CycleStatus.failed);
        obligations.clearAttempt();
        log("-> my own walk came home: cycle on edge " + corner
                + (sm.isSuccessful() ? " COMPLETED" : " FAILED, moving on"));
        return "Status Message from " + sm.getSenderId()
                + (sm.isSuccessful() ? "(SUCCESS)" : "(FAILURE)");
    }

    /**
     * Records a settled face against this robot's own corner of it, if it tracks corners at all.
     *
     * <p><strong>The corner is {@code edgeOwedBy(tuple)}, not the status's
     * {@code originOutgoingEdgeID}.</strong> The origin edge names the corner as the
     * <em>initiator</em> sees it, and every other robot on the walk occupies a different site with
     * a different outgoing edge for the same face -- a participant that was handed edge {@code a}
     * owns the corner {@code next(a)}. Marking the origin id at a participant writes a key
     * belonging to someone else's site, which on a single-role lattice is a plausible-looking edge
     * id and so fails silently. The two spellings agree at exactly one robot, the initiator, which
     * is why the mistake survives every test that only watches an initiator.
     *
     * <p><strong>Relays mark {@code complete}, never {@code failed}.</strong> A dead end below this
     * robot is somebody else's and says nothing about whether this robot can close the same face
     * from its own side. Only {@link #settleOwnWalk} writes a failure, and only for the walk it
     * minted.
     *
     * <p><strong>Every participant records, not just the roots.</strong> A builder tracks the
     * corners of its site from the moment it takes one, so the wrapping status is recorded by
     * everyone it passes rather than delivered and discarded. It used to be a no-op for a builder,
     * whose map was empty until promotion filled it -- and the cost was measurable: each
     * participant re-derived the same face from scratch after promotion, minting its own
     * certificate and running a full lap to learn what this status already told it.
     *
     * <p>The {@code containsKey} guard is therefore no longer a role filter. It is what it reads
     * as: a refusal to invent a corner this robot does not own.
     */
    private void markCornerFromStatus(FaceObligation tuple, StatusMessage sm) {
        int corner = edgeIdOwedBy(tuple);
        if (corner == -1 || !completedCycles.containsKey(corner) || !sm.isSuccessful()) {
            return;
        }
        setCycleStatusOf(corner, CycleStatus.complete);
    }

    /**
     * Frees the child slot so the same edge can be offered to somebody else, re-offers on the
     * spot, and never propagates.
     *
     * <p>A rejection is not a return message in the sense the two above are: it carries no
     * initiator and travels exactly one hop, back to the robot that made the offer. The face is
     * still viable; only the candidate was wrong.
     *
     * <p><strong>The certificate rides back on it, which is what makes the re-offer possible.</strong>
     * Relaying is inline now, so the robot no longer holds a copy of the walk it forwarded -- the
     * rejection returning that certificate is the only thing that keeps the walk alive. Leaving the
     * link merely unbound would strand it: nothing else would ever carry it, and the face would
     * hang until its initiator gave up.
     */
    /**
     * Whether a rejection is answering a walk this robot minted itself, rather than one it carries.
     *
     * <p>The rejection path is the one return path with no {@code initiatorId == self} branch of its
     * own, and it needs one, because the attempt is allowed to duplicate a carried link: a root
     * building corner {@code c} while relaying somebody else's walk that also owes {@code c} offers
     * both to the same neighbour. {@link FaceObligationSet#findByChild} scans carried first, so
     * without this test a rejection meant for the attempt would release the wrong link, re-offer
     * somebody else's certificate, and mark the wrong corner.
     *
     * <p><strong>Answered from the message.</strong> The certificate riding the rejection names its
     * own minter, so this asks the walk rather than the link. The field this replaced --
     * {@code FaceObligation.inFlightInitiatorId} -- tried to answer it from the link and held only
     * one walk, so a second certificate over the same edge erased the first's identity. A message
     * cannot be overwritten by another message.
     *
     * <p>A rejection carrying no certificate names no walk, so it falls through to the carried link.
     * That is the same answer the overwritten field gave, and the branch that then abandons the walk
     * is unchanged -- see {@link #reofferAfterRejection}.
     */
    private boolean rejectionAnswersMyOwnWalk(RejectAssignmentMessage rm) {
        FaceObligation attempt = obligations.getAttempt();
        return rm.getCertificate() != null
                && rm.getCertificate().getInitiatorID() == self.getRobotId()
                && attempt != null
                && edgeIdOwedBy(attempt) == rm.getViaEdgeId();
    }

    /**
     * The carried link that owes this edge onward -- the one a return message travelled back over.
     *
     * <p><strong>The routing key for every return path.</strong> Not the child, which does not
     * identify a link: a robot relaying two faces to one neighbour holds two links naming that
     * robot, and picking the wrong one marks the wrong corner and forwards to the wrong parent
     * without any of it being visible. Not the child plus who-minted-the-walk either, which is what
     * {@code FaceObligation.inFlightInitiatorId} was -- one slot per link, overwritten by the next
     * certificate across it, so it disambiguated only until it mattered.
     *
     * <p>The edge is exact and needs no state: {@code edgeOwedBy} is injective over carried tuples,
     * since {@code next} is a bijection on half-edges and one tuple is admitted per incoming edge.
     * The attempt is excluded deliberately -- it can owe the same edge as a carried link, and every
     * caller here has already settled that question from the message.
     */
    private FaceObligation carriedLinkOwing(int edgeId) {
        for (FaceObligation obligation : obligations.asList()) {
            if (!obligations.isAttempt(obligation) && edgeIdOwedBy(obligation) == edgeId) {
                return obligation;
            }
        }
        return null;
    }

    private String routeRejectionThroughTuple(RejectAssignmentMessage rm) {
        FaceObligation tuple = rejectionAnswersMyOwnWalk(rm)
                ? obligations.getAttempt()
                : carriedLinkOwing(rm.getViaEdgeId());
        log("-> assignment REJECTED by " + rm.getSenderId());
        if (tuple == null) {
            return "Reject Assignment Message from " + rm.getSenderId() + "(NO MATCHING OBLIGATION)";
        }

        // Read the child before releasing it -- release() is what clears the slot the undraw needs
        // to ask about.
        int refuser = rm.getSenderId();
        tuple.release();
        undrawEdgeTo(refuser);

        if (rm.isRetryable()) {
            // No ban: the rejecter said "not now", and it has left the target site, so
            // nearest-candidate selection will not simply hand it back the same offer.
            log("-> assignment is retryable, will re-offer edge " + tuple.getEdgeId());
        } else {
            // Scoped to this face's own tuple, so it survives a retry of the same edge.
            tuple.ban(rm.getSenderId());
            log("-> assignment is NOT retryable, banning " + rm.getSenderId() + " on this face");
        }

        String reoffer = reofferAfterRejection(tuple, rm);
        return "Reject Assignment Message from " + rm.getSenderId()
                + (rm.isRetryable() ? "(REJECTED, RETRYABLE)" : "(REJECTED, NOT RETRYABLE)")
                + (reoffer == null ? "" : " | " + reoffer);
    }

    /**
     * Hands the returned certificate to the next candidate on the same edge.
     *
     * <p>This robot's own attempt is left alone: {@link #serviceOneObligation} picks it up on the
     * next activation and mints afresh, which costs a tick and keeps the initiating path in one
     * place. A carried walk has no such fallback -- nobody else holds that certificate -- so it is
     * re-offered here or it is gone.
     *
     * @return a description for the tick log, or null if nothing was re-offered
     */
    private String reofferAfterRejection(FaceObligation tuple, RejectAssignmentMessage rm) {
        if (obligations.isAttempt(tuple) || rm.getCertificate() == null) {
            return null;
        }
        HalfEdge owed = edgeOwedBy(tuple);
        if (owed == null) {
            return reportRelayFailure(tuple, rm.getCertificate(),
                    rm.getOriginVertexID(), rm.getOriginOutgoingEdgeID());
        }
        GeometricCycleLatticeRobot child = findBestNeighborForEdge(owed, tuple);
        if (child == null) {
            log("-> no candidate left on edge " + owed.getId() + " after the rejection");
            return reportRelayFailure(tuple, rm.getCertificate(),
                    rm.getOriginVertexID(), rm.getOriginOutgoingEdgeID());
        }
        offer(tuple, child, owed, owed.getId(),
                rm.getOriginVertexID(), rm.getOriginOutgoingEdgeID(), rm.getCertificate());
        return "Re-offered edge " + owed.getId() + " to robot " + child.getRobotId();
    }

    /**
     * Cancels this robot's own attempt on the broken face if it had one, then passes the report on
     * up -- and lets it die at the first robot with nobody above it.
     *
     * <p><strong>Addressed to nobody.</strong> This used to route on an {@code initiatorId} and stop
     * at the robot that minted the lost walk, exactly like a status. That was the bug rather than
     * the design: a link carries several walks at once and recorded only one of them, so when two
     * roots raced the same face the report reached whichever offered last and the other was left
     * sitting on an attempt that could never resolve. See {@link CertificateLostMessage} for the
     * argument that the parent chain above the break is exactly the set of walks that just died --
     * which is why telling all of them is correct rather than merely safe.
     *
     * <p>Two independent steps, in this order. <em>Cancel</em> asks a question about this robot;
     * <em>forward</em> asks a question about the link the report came in on. They are not the same
     * question and neither is a special case of the other: a root that is both relaying the face and
     * building it does both, and the duplicate tuples that used to need disambiguating are simply
     * one answer each.
     *
     * <p>{@code attempted}, not {@code failed} -- the carrier left, which says nothing about whether
     * the face can be built. {@link #determineNextCycleToComplete()} already prefers
     * {@code unattempted}, so the corner goes to the back of the queue and is retried.
     *
     * <p>The link itself is kept everywhere it is forwarded, binding included: only the message in
     * flight was lost, and the adjacency it travelled over is still real.
     */
    private String routeCertificateLostThroughTuple(CertificateLostMessage cm) {
        if (exceedsHopCap(cm)) {
            log("-> ANOMALY: lost-certificate report has climbed " + cm.getHops() + " links, past "
                    + "the " + graph.maxCycleLength() + "-hop bound; dropping it");
            return "Certificate Lost Message from " + cm.getSenderId() + "(ANOMALY, HOP CAP)";
        }

        String cancelled = cancelAttemptOnBrokenFace(cm);

        // Is there a parent above me to pass this to? A carried link owing the edge the sender was
        // offered is what says so, and it is the same lookup every other return path uses. No such
        // link -- because this robot minted the walk, or because it holds nothing on that edge --
        // means the report has climbed the whole chain and stops here.
        FaceObligation tuple = carriedLinkOwing(cm.getLostOnEdgeId());
        if (tuple == null) {
            log("-> nobody above me carries this walk; the report ends here");
            return "Certificate Lost Message from " + cm.getSenderId() + "(CHAIN END)" + cancelled;
        }

        tuple.clearForResolvedWalk();
        forwardCertificateLostTo(tuple, cm);
        return "Certificate Lost Message from " + cm.getSenderId() + "(FORWARDED)" + cancelled;
    }

    /**
     * Drops this robot's own attempt when the walk that just broke was its own.
     *
     * <p>A plain role check, not an inference from tuple state. Only the root branch of
     * {@link #sendMessage} ever opens an attempt, so a non-root has nothing here to cancel and the
     * guard is belt-and-braces -- which is what roles are for.
     *
     * <p><strong>Both conditions are needed, and together they are exact.</strong>
     * {@code matchesChild} says this robot's own walk went to the robot now reporting the loss --
     * the attempt may have been rebound to somebody else since, in which case its walk went
     * elsewhere and is very possibly still alive. The edge comparison says it went in over the
     * <em>same</em> edge, so both walks entered the same downstream tuple and both really did die;
     * {@link CertificateLostMessage#getLostOnEdgeId()} is re-stamped at each hop precisely so this
     * stays answerable all the way up.
     *
     * <p>Over-cancelling is not the harmless direction. A live walk whose attempt has been forgotten
     * comes home to {@link #settleOwnWalk}, finds no attempt, and is dropped as already settled --
     * so a face that genuinely closed is never recorded and the whole lap is re-run.
     *
     * @return a suffix for the tick log, empty when nothing was cancelled
     */
    private String cancelAttemptOnBrokenFace(CertificateLostMessage cm) {
        if (role != CycleRole.root) {
            return "";
        }
        FaceObligation attempt = obligations.getAttempt();
        if (attempt == null
                || !attempt.matchesChild(cm.getSenderId())
                || edgeIdOwedBy(attempt) != cm.getLostOnEdgeId()) {
            return "";
        }

        int corner = edgeIdOwedBy(attempt);
        setCycleStatusOf(corner, CycleStatus.attempted);
        obligations.clearAttempt();
        log("-> my own walk on edge " + corner + " went through " + cm.getSenderId()
                + " and died with it; will relaunch that corner later");
        return " | cancelled my attempt on edge " + corner;
    }

    //MESSAGE-PROCESSING UTIL

    /**
     * Reports success to a parent, handing the certificate back with it.
     *
     * <p>The certificate rides the return path rather than being kept: whoever made the
     * offer gets back the exact certificate it sent, so it can act on the outcome without
     * ever having held a copy while the walk was in flight.
     */
    private void forwardSuccessUpstream(int parentId, int originVertexID, int originEdgeID,
                                        int viaEdgeId, VoltageCertificate returning) {
        sendVerdictUpstream(parentId, true, originVertexID, originEdgeID, viaEdgeId, returning);
    }

    private void forwardFailureUpstream(PositioningMessage pm) {
        sendVerdictUpstream(pm.getSenderId(), false, pm.getOriginVertexID(),
                pm.getOriginOutgoingEdgeID(), pm.getAssignedOutgoingEdgeID(), pm.getCertificate());
    }

    /**
     * Emits one verdict to the robot that handed this walk over, stamped with the walk's initiator
     * so it knows where to stop.
     *
     * <p>Both callers used to dereference {@code getNeighborByID} unchecked -- one passed a
     * possibly-null parent straight into {@code send}, the other called {@code getRobotId()} on it.
     * A neighbour that has drifted out of range is ordinary, and the NPE that followed was caught
     * by {@code AsyncRobotPanel.tickRobot}, so the robot survived but <em>the rest of its
     * activation did not</em>: the message had already been consumed, and nothing after the throw
     * ran -- no response sent, no claim broadcast. A silently abandoned tick is far harder to see
     * in a log than a missing status, so this reports and continues.
     *
     * <p>The initiator is read off the certificate here because this is the one place a verdict is
     * <em>created</em> rather than relayed, and at creation the certificate is always in hand.
     */
    private void sendVerdictUpstream(int parentId, boolean successful, int originVertexID,
                                     int originEdgeID, int viaEdgeId, VoltageCertificate cert) {
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        if (parent == null) {
            log("-> cannot report a " + (successful ? "SUCCESS" : "FAILURE") + " to " + parentId
                    + ": unreachable");
            return;
        }
        int initiatorId = cert == null ? parentId : cert.getInitiatorID();
        send(parent, new StatusMessage(self.getRobotId(), parentId, successful,
                originVertexID, originEdgeID, initiatorId, viaEdgeId, cert));
    }

    private void forwardRejectionUpstream(PositioningMessage pm, boolean isRetryable) {
        RejectAssignmentMessage rm = new RejectAssignmentMessage(pm.getRecipient(), pm.getSenderId(),
                pm.getOriginVertexID(), pm.getOriginOutgoingEdgeID(), isRetryable,
                pm.getAssignedOutgoingEdgeID(), pm.getCertificate());
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
            // The drawn edges describe a site this robot is leaving, so all of them go. That falls
            // out of the predicate rather than needing an exception: drainForVacate has already
            // emptied both containers, so nothing is permanently linked to anything any more.
            undrawEdgeTo(obligation.getChildId() == null ? -1 : obligation.getChildId());

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
                    // The edge this robot was offered, which is this tuple's own key -- so the
                    // parent can tell which of its links is being handed back even when two of
                    // them point at this same robot.
                    obligation.getEdgeId(),
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
     *
     * <p><strong>This method judges. It does not record.</strong> Every verdict leaves here as a
     * {@code StatusMessage} stamped with this robot's own id, laps the face a second time so every
     * participant can mark its own corner, and comes home to {@link #settleOwnWalk} -- which is
     * where {@code completedCycles} is written and the attempt tuple is dropped.
     *
     * <p>It used to do both, and the two cannot happen here. Marking on arrival of the certificate
     * would end the attempt while its status was still travelling, freeing the corner to be
     * re-opened underneath a lap already in progress; and it wrote {@code completedCycles} keyed on
     * {@code pm.getOriginOutgoingEdgeID()} -- a value off the wire, into an unconditional
     * {@code put} -- so a stale or foreign origin id created a corner that does not exist, which
     * {@code hasFailed()} then counts and {@code determineNextCycleToComplete()} then scans.
     * {@link #settleOwnWalk} derives the corner from the open attempt instead, which is this
     * robot's own outgoing edge by construction.
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
            // out to trace. The verdict travels rather than being written here -- see the
            // method javadoc.
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
            forwardFailureUpstream(pm);
            return "Positioning Message from " + pm.getSenderId() + " (REJECTED, NON-IDENTITY PRODUCT)";
        }

        // The face is closed. The closing hop names the corner before this one in the rotation
        // order, and it is occupied. The walk is cycleLength long -- checked above -- so the last
        // relayer owed prev(origin) and that is what it assigned; twin(prev(h)) is sigma^-1(h) by
        // Edmonds' rule, so this re-arms the corner that neighbours the one just closed, and the
        // robot that sent this message is standing on it.
        //
        // The statement-ordering hazard that used to live here is gone with the mark. This method
        // no longer writes completedCycles at all, so there is no longer a window in which a
        // failure verdict could fire promoteAdjacentVerticesToRoots a line before a success
        // overturned it.
        rearmTwinOfIncomingEdge(pm.getAssignedOutgoingEdgeID());

        self.addEdge(new Edge(pm.getRecipient(), pm.getSenderId()));
        // Stamped with the edge the CLOSING hop offered this robot, not with the corner this walk
        // was minted on. The status is about to travel back down the chain, and the first robot it
        // reaches is the one that made that closing offer -- so the link it must route through is
        // the one owing this edge.
        forwardSuccessUpstream(pm.getSenderId(), pm.getOriginVertexID(),
                pm.getOriginOutgoingEdgeID(), pm.getAssignedOutgoingEdgeID(), cert);
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
     * The same bound for a lost-certificate report climbing the parent chain.
     *
     * <p>Pure guard, and expected never to fire. The chain is a simple path -- the closing hop of a
     * face is handled by {@code evaluateReturningCertificate} and never opens a carried tuple, so
     * there is nothing to loop through -- and it ends at the first robot with no parent. But this
     * report is addressed to nobody, and an unaddressed return message that finds a cycle is one
     * that circulates forever; the old {@code initiatorId} ruled that out by construction and this
     * is what replaces it. Logged as an anomaly rather than silently dropped, because if it fires
     * the chain argument is wrong and that is worth seeing.
     */
    private boolean exceedsHopCap(CertificateLostMessage cm) {
        return cm.getHops() >= graph.maxCycleLength();
    }

    /**
     * Tells the parent that this robot could find nobody to carry the walk further.
     *
     * <p>This is the "the face cannot be built out from here" generator. The failure is stamped
     * with the walk's own initiator so it dies at the robot that minted it, exactly as a success
     * does -- the two verdicts travel the same path and are recorded by the same code.
     *
     * <p><strong>The link survives.</strong> The tuple used to be removed here. That was right when
     * a tuple was a per-walk commitment and is wrong now: this robot has not moved, its parent has
     * not moved, and the adjacency between them is as true after a dead end as before it. What is
     * released is the child slot, because the child is exactly what could not be found.
     *
     * <p>Notably it does not stand the robot down either. That was the old builder behaviour --
     * report failure, reset to unassigned -- and it was the chain collapse in another guise: a dead
     * end on one face says nothing about the site this robot occupies or about the other faces
     * incident to it.
     *
     * @param cert the walk that could not be carried, whose initiator this failure is addressed to
     */
    private String reportRelayFailure(FaceObligation tuple, VoltageCertificate cert,
                                      int originVertexID, int originEdgeID) {
        int parentId = tuple.getParentId();
        // Child read before release, which is what clears the slot the undraw asks about.
        int deadEnd = tuple.getChildId() == null ? -1 : tuple.getChildId();
        tuple.release();
        undrawEdgeTo(deadEnd);
        // The walk is over, so its exclusions go with it. Failing is a resolution like any other,
        // and a link that kept the bans it accumulated on the way to a dead end would carry them
        // for the rest of the robot's life -- eventually hiding whoever settles onto the site it
        // points at. A later certificate over this edge is a different walk and deserves the whole
        // neighbourhood again.
        tuple.clearForResolvedWalk();
        sendVerdictUpstream(parentId, false, originVertexID, originEdgeID, tuple.getEdgeId(), cert);
        log("-> no candidate to carry the walk; reported FAILURE to " + parentId);
        return "Reporting Failure (No candidate to relay to, told " + parentId + ")";
    }

    /*
        ////////////////////////
        CERTIFICATE CUSTODY
        ////////////////////////

        Almost nothing left, and that is the point.

        Custody used to be a whole mechanism: a robot that accepted an assignment could not relay
        for several ticks, so the message was pushed back into the inbox, `isHeldInCustody`
        recognised it by "a tuple exists for this (sender, edge)", and `pollNextActionable` rotated
        past it. All of it existed because relaying was deferred to a later activation.

        Relaying is inline now -- `relayAlongTuple` runs in the activation the assignment is read --
        so the only robot that still cannot act immediately is one driving to its FIRST site. That
        robot never reaches the poll at all: the gate at the top of `processMessages` answers
        nothing while it moves, so its assignment simply sits in the queue. The queue does the
        holding, and no marker is needed to recognise what it is holding.

        Which is just as well, because the old marker would now be actively harmful. Tuples are
        permanent, so "a tuple exists for this (sender, edge)" is true for every established link --
        every future assignment over a working link would have been rotated past unread, forever.
     */

    /**
     * Accepts a first assignment: opens the link for its edge and puts the message back in the
     * queue to be relayed once this robot has arrived.
     */
    private void holdInCustody(PositioningMessage pm, int parentId, int edgeId) {
        obligations.getOrCreate(parentId, edgeId);
        // The message HAS been consumed by now -- processMessages polled it to get here -- so this
        // is "put it back", not "leave it alone". See the poll site for why the tail is the right
        // end and why the reorder is harmless.
        incomingMessages.add(pm);
    }

    /** The queued assignment this link is still holding, or null. */
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

    /**
     * {@link #custodyFor} plus removal from the inbox, for a robot giving up its site.
     *
     * <p>Only ever finds anything for a robot that is still in transit -- everyone else relayed
     * inline and holds nothing -- which is exactly the robot that owes its parent the certificate
     * back.
     */
    private PositioningMessage takeCustody(FaceObligation obligation) {
        PositioningMessage held = custodyFor(obligation);
        if (held != null) {
            incomingMessages.remove(held);
        }
        return held;
    }

    //ROOT-RELATED UTIL

    /**
     * The next corner of this root's own site worth starting a walk on, or -1.
     *
     * <p>Unattempted before attempted, so a corner that has already been tried and lost its
     * certificate waits behind ones that have never been tried at all.
     *
     * <p><strong>Decided from {@code completedCycles} alone.</strong> There used to be a second
     * test here -- skip any corner with a live tuple -- and it cannot survive persistent links. A
     * carried tuple owes {@code next} of the edge it arrived over, which is one of <em>this
     * robot's own corners</em>; once that tuple never goes away, the corner it owes is permanently
     * "live" and this method could never return it again. A root would stop being able to build
     * any corner a neighbour's walk had ever passed through, which is most of them.
     *
     * <p>What that test was really guarding -- do not start a second walk down a corner already
     * being built -- is the attempt slot's job, and {@link #hasInitiatedFaceInFlight()} asks it
     * directly in {@code sendMessage}. One gate, on the transient thing, rather than a second one
     * inferred from the permanent thing.
     */
    private int determineNextCycleToComplete() {
        int attempted = -1;
        for (Entry<Integer, CycleStatus> entry : completedCycles.entrySet()) {
            HalfEdge edge = retrieveEdgeFromGraph(entry.getKey());
            if (edge == null) {
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
     * Gives this robot a corner to track for every edge leaving its role, <em>and no others</em>.
     *
     * <p>Run once per lattice site a robot occupies: when it accepts an assignment and becomes a
     * builder, or when it is promoted straight from unassigned. <strong>Not</strong> when a builder
     * is promoted to root -- it already has this map, and re-running would reset every corner to
     * {@code unattempted}, discarding the closures a wrapping status recorded while it was
     * building. That is the one call site where getting this wrong is silent: the tests still pass
     * and the robot simply re-derives everything it already knew.
     *
     * <p>Clears before it fills, which it did not used to. Adding one entry per outgoing edge
     * without removing anything cannot establish the postcondition the name claims: a robot
     * arriving here with keys from a different role would keep them, and on a multi-role lattice
     * those are corners of somebody else's site. Clearing makes the map mean "the corners of where
     * I am standing" by construction, which is the invariant {@link #setCycleStatusOf} now guards.
     */
    private void initializeEdgeMap() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);
        completedCycles.clear();
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
     * <p><strong>Only a robot standing exactly on the corner may be crowned.</strong> This used to
     * call {@code findBestNeighborForEdge}, whose contract is "nearest candidate" -- the
     * exact-position branch is a fast path inside it, not a guarantee. The javadoc here claimed the
     * lookup could not fail, on the reasoning that a complete corner always has its occupant. Both
     * halves of that were wrong, and each failed differently:
     *
     * <ul>
     *   <li>When some <em>other</em> robot was nearer, it was crowned instead. It then ran
     *       {@code initializeEdgeMap()} against a site it does not occupy and began emitting walks
     *       that can never close -- a spurious root, indistinguishable in the log from a real
     *       one.</li>
     *   <li>When {@code observations} was empty the call returned null and
     *       {@code neighbor.getRobotId()} threw. {@code AsyncRobotPanel.tickRobot} catches it, so
     *       the robot survived -- but the rest of its activation did not, and this runs from inside
     *       {@code setCycleStatusOf}, i.e. from the middle of message processing. A message was
     *       consumed, no response went out, no claim was broadcast, and nothing in the log said
     *       so.</li>
     * </ul>
     *
     * <p>So the match is required, and a corner whose occupant is not currently observable is
     * skipped and reported rather than substituted for. That is a deferral, not a loss: this method
     * is idempotent and reached again on later activations, and {@code announcedCorners} means the
     * corners that did match are not re-announced.
     *
     * @return whether every complete corner found its occupant. The caller uses this as one of the
     *         two conditions for going stable -- a root whose corners all closed but whose
     *         neighbours are not all in sight has not finished handing on what it built, so it
     *         stays a root and tries again. Recomputed each pass rather than remembered: a corner
     *         momentarily out of view must not lock the robot out of stable permanently.
     */
    private boolean promoteAdjacentVerticesToRoots() {
        Role myRole = getCurrentRole();
        List<HalfEdge> edges = graph.getOutgoingHalfEdges(myRole);

        boolean allMatched = true;
        for(HalfEdge edge : edges) {
            int edgeId = getEdgeIDof(edge);
            if(completedCycles.get(edgeId) != CycleStatus.complete) {
                continue;
            }
            GeometricCycleLatticeRobot occupant = findNeighborStandingOn(edge);
            if (occupant == null) {
                allMatched = false;
                log("-> corner " + edgeId + " is complete but nobody is observably standing on it; "
                        + "not promoting anyone across it this activation");
                continue;
            }
            if (!announcedCorners.add(edgeId)) {
                continue;
            }
            send(occupant, new PromotionMessage(self.getRobotId(), occupant.getRobotId(),
                    getVertexIDof(edge), edgeId, !hasFailed()));
            log("-> promoting neighbor " + occupant.getRobotId() + " on edge " + edgeId + " to root");
        }
        return allMatched;
    }

    /**
     * The neighbour occupying {@code targetEdge}'s far end, by exact position, or null.
     *
     * <p>Deliberately not {@code findBestNeighborForEdge}: that method picks the <em>nearest</em>
     * candidate and is right for choosing who to send toward a site, whereas this asks whether a
     * particular site is already occupied and must answer no when it is not. Same tolerance as the
     * exact-position branch inside it, so the two agree about what "standing on it" means.
     */
    private GeometricCycleLatticeRobot findNeighborStandingOn(HalfEdge targetEdge) {
        OrientedPoint targetLocal = getTargetInLocalCoordinates(targetEdge);
        for (Observation obs : observations.values()) {
            if (MathUtils.isZero(targetLocal.distance(obs.getLocalPosition()), MathUtils.EPSILON)) {
                return getNeighborByID(obs.getId());
            }
        }
        return null;
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
     *       corners before spinning on one short of candidates, rather than re-launching
     *       immediately into a neighbourhood that just failed to supply a candidate.
     *       {@link #determineNextCycleToComplete()} prefers {@code unattempted}, so re-arming
     *       would jump that queue.</li>
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
            // Both call sites establish the edge upstream -- relayAlongTuple resolves it, and
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
            reportCertificateLost(obligation);
            // Release, not remove. The child is gone, so the binding is stale -- but this robot
            // and its parent have not moved, so the link between them is as true as it was, and
            // the bans it carries are still the right exclusions for the next candidate. An
            // attempt is a different matter: it is transient by design and is cleared below.
            //
            // ORDER IS LOAD-BEARING. The release has to happen BEFORE the undraw: it is what stops
            // this link naming the departed robot as its child, and therefore what lets
            // isPermanentlyLinkedTo answer "no" and the edge actually go. Undraw first and every
            // robot that ever wandered off keeps its line forever, because the link it left behind
            // still vouches for it.
            int departedChild = obligation.getChildId() == null ? -1 : obligation.getChildId();
            obligation.release();
            if (obligations.isAttempt(obligation)) {
                obligations.clearAttempt();
            }
            undrawEdgeTo(departedChild);
        }
    }

    /**
     * Tells the robot that minted this walk that its certificate is gone.
     *
     * <p><strong>A slot check, not a question about who minted anything.</strong> If the broken link
     * is this robot's own attempt then its own walk died and there is nobody upstream to tell: it
     * marks the corner {@code attempted} and picks the face up later. Otherwise the report starts
     * climbing, and every root on the way up cancels whatever of its own died with it.
     *
     * <p>That used to be a comparison against {@link FaceObligation}'s recorded in-flight initiator,
     * and it was the wrong shape twice over. The field held one walk where the link carried several,
     * so the report named the wrong root; and {@code isAttempt} was the thing being approximated
     * anyway. The container knows which tuple is the attempt -- it holds it in a slot of its own --
     * so ask it.
     *
     * <p>{@code takeCustody} is still called and its result still discarded on purpose: nothing here
     * reads the certificate any more, but a queued assignment for a walk that has just died should
     * not be left sitting in the inbox to be relayed later.
     */
    private void reportCertificateLost(FaceObligation obligation) {
        takeCustody(obligation);

        if (obligations.isAttempt(obligation)) {
            setCycleStatusOf(edgeIdOwedBy(obligation), CycleStatus.attempted);
            log("-> the walk that broke was my own; corner " + edgeIdOwedBy(obligation)
                    + " goes back in the queue");
            return;
        }

        GeometricCycleLatticeRobot parent = getNeighborByID(obligation.getParentId());
        if (parent == null) {
            // Structurally unreachable: a parent is parked one lattice edge away, and every
            // lattice's edge length is pinned below COMM_RANGE by FaceClosureTest. Logged
            // rather than ignored, because if it ever fires that guard has been broken and
            // every corner above this one will hang.
            log("-> cannot report lost certificate: parent " + obligation.getParentId()
                    + " unreachable, which should be impossible");
            return;
        }
        // Stamped with the edge THIS robot was offered, which is what tells the parent whether its
        // own attempt died here -- see CertificateLostMessage#getLostOnEdgeId.
        send(parent, new CertificateLostMessage(self.getRobotId(), parent.getRobotId(),
                obligation.getEdgeId()));
    }

    /**
     * Undraws the edge toward a robot, <strong>unless a permanent link still connects them</strong>.
     *
     * <p>A drawn edge is a claim about topology, so whether to remove one is a question about
     * topology -- not about which code path drew it. Keep it while
     * {@link FaceObligationSet#isPermanentlyLinkedTo} holds, and drop it when the only thing that
     * ever connected the two was a transient attempt, or nothing at all. That single test gives
     * every case: a child that wandered off loses its edge (the sweep releases its link first, so
     * the predicate is already false), a robot that refuses a site it could never take loses its
     * edge, a robot reached across a long comm range loses its edge -- and a settled neighbour that
     * refuses a futile offer <em>keeps</em> its edge, because the two of them have been relaying for
     * each other all along.
     *
     * <p><strong>Keyed on the robot, not on an {@link Edge} object, and that is the fix.</strong>
     * This used to take the exact instance an obligation had drawn and remove it by reference,
     * which was right while duplicates existed. Giving {@code Edge} value equality made
     * {@code addEdge} deduplicate -- so {@code offer}'s freshly built instance is discarded when a
     * real link already drew that pair, the obligation stores the discarded object, and removing it
     * matched the surviving good edge by value and deleted that instead. A root refusing a
     * neighbour's futile offer erased a connection both of them were still using.
     *
     * <p>Matching on the far endpoint alone is exact: every edge this robot draws has itself as
     * {@code fromId} -- {@code acceptFirstAssignment}, {@code relayAlongTuple},
     * {@code evaluateReturningCertificate} and {@code offer} all do.
     *
     * @param otherRobotId the robot at the far end, or a negative id for "nothing to undraw"
     */
    private void undrawEdgeTo(int otherRobotId) {
        if (otherRobotId < 0) {
            return;
        }
        if (obligations.isPermanentlyLinkedTo(otherRobotId)) {
            log("-> keeping the edge to " + otherRobotId + ": a permanent link still connects us");
            return;
        }
        if (occupiesAdjacentSite(otherRobotId)) {
            log("-> keeping the edge to " + otherRobotId + ": it is standing on one of my lattice sites");
            return;
        }
        self.getEdges().removeIf(edge -> edge.getFromId() == self.getRobotId()
                && edge.getToId() == otherRobotId);
    }

    /**
     * Whether that robot is observed standing exactly on one of this site's lattice neighbours.
     *
     * <p><strong>The case a carried link cannot express.</strong> A link is
     * {@code (parent, incoming edge) -> child}, and it is created when a walk <em>arrives</em> --
     * so a root that <em>placed</em> a robot has no link naming it, because the root is the one who
     * sent. That relationship is recorded only in the attempt tuple, which is transient by design
     * and cleared the moment the walk resolves. A root whose walk then dead-ends, offers its next
     * corner to that same robot (still the nearest candidate), and is correctly refused, would find
     * nothing vouching for a neighbour it put there itself, and delete the edge.
     *
     * <p>So the fact is asked directly instead of inferred from message history: standing on
     * {@code target(e)} for one of this role's outgoing edges <em>is</em> lattice adjacency,
     * whether the face closed, failed, or is still in flight.
     *
     * <p>Kept alongside {@link FaceObligationSet#isPermanentlyLinkedTo} rather than replacing it,
     * because the two fail in opposite directions: this one answers false for a neighbour that is
     * momentarily unobservable, and that one answers false for a robot this one placed. Either
     * alone still deletes edges that should live.
     *
     * <p>Not expressed via {@link #findNeighborStandingOn}: that scans every observation per edge
     * and returns a robot, where this asks about one known robot and wants a boolean. Both use the
     * same {@code MathUtils.EPSILON} match, so "standing on it" means one thing throughout.
     */
    private boolean occupiesAdjacentSite(int robotId) {
        Observation observed = observations.get(robotId);
        Role myRole = getCurrentRole();
        if (observed == null || myRole == null) {
            return false;
        }
        for (HalfEdge outgoing : graph.getOutgoingHalfEdges(myRole)) {
            if (MathUtils.isZero(
                    getTargetInLocalCoordinates(outgoing).distance(observed.getLocalPosition()),
                    MathUtils.EPSILON)) {
                return true;
            }
        }
        return false;
    }

    /** A null obligation means no exclusions -- see {@link #findBestNeighborForEdge}. */
    private static boolean isBannedOn(FaceObligation obligation, int robotID) {
        return obligation != null && obligation.isBanned(robotID);
    }

    /**
     * Passes a lost-certificate report one link further up, re-stamped for the robot receiving it.
     *
     * <p>Takes the tuple rather than a parent id, because both things it needs come off it. The
     * parent is not always the robot this one is anchored to -- a root has no anchor at all, and a
     * settled builder carrying a second face was never anchored to that face's parent.
     *
     * <p><strong>{@code lostOnEdgeId} is rewritten, not passed through.</strong> It means "the edge
     * the sender was offered", so it is this robot's own tuple key going up, exactly as it was the
     * reporter's going into this one. That is what keeps it comparable at the receiver: a parent's
     * attempt owes the edge it offered this robot, and nothing else does.
     */
    private void forwardCertificateLostTo(FaceObligation tuple, CertificateLostMessage cm) {
        int parentId = tuple.getParentId();
        GeometricCycleLatticeRobot parent = getNeighborByID(parentId);
        if (parent == null) {
            log("-> cannot forward lost certificate: parent " + parentId + " unreachable");
            return;
        }
        send(parent, new CertificateLostMessage(self.getRobotId(), parent.getRobotId(),
                tuple.getEdgeId(), cm.getHops() + 1));
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

    /**
     * Whether this robot is already building a corner of its own.
     *
     * <p>The single in-flight gate. It reads the attempt slot, which is the one entry in the
     * obligation set that is transient by design; the carried links are permanent and say nothing
     * about whether anything is in flight.
     */
    private boolean hasInitiatedFaceInFlight() {
        return obligations.getAttempt() != null;
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
     * <p><strong>Nothing carried is lost.</strong> The links survive because
     * {@link #resetToRoot()} does not clear them -- a promoted robot has not moved, so every
     * adjacency they record is still true -- and the relay parents survive with them, because
     * {@link #relayAlongTuple} reads the parent off the link rather than off
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

        // "Am I already a root" asked of the role, which is what roles are for. This used to read
        // `!completedCycles.isEmpty()`, on the reasoning that tracking corners IS what being a root
        // is -- true only while roots were the sole robots with a corner map. Builders track their
        // own corners now, so that test would answer "already a root" for every builder ever
        // promoted, and the branch below that grants rootship would never run.
        if (role == CycleRole.root) {
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

        // A different question, and the map-emptiness test is the right one to answer it with: do I
        // have a corner map yet? A promoted BUILDER does, built when it took its site, and it must
        // keep it -- re-initialising resets every corner to unattempted and discards exactly the
        // marks a wrapping status left behind, which is the whole reason builders track corners.
        // A promoted UNASSIGNED robot does not, and that is reachable:
        // promoteAdjacentVerticesToRoots crowns whoever is standing on a complete corner, and that
        // can be a robot that never took an assignment.
        boolean alreadyTracking = !completedCycles.isEmpty();

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
        if (!alreadyTracking) {
            initializeEdgeMap();
        }
        log("-> promoted to root by robot " + stableID
                + (alreadyTracking ? ", keeping the corners it already recorded: " + completedCycles
                                   : ""));
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
    /**
     * Records the outcome of one of <strong>this robot's own</strong> corners.
     *
     * <p><strong>An edge id this robot does not track is refused, loudly.</strong> This used to be
     * an unconditional {@code put}, which meant any caller with the wrong key silently
     * <em>created</em> a corner rather than being told off -- and a phantom corner is not inert:
     * {@link #hasFailed()} counts it and {@link #determineNextCycleToComplete()} hands it out, so
     * the root either chases a corner that does not exist or stands down on one.
     *
     * <p>No caller does that today. Every writer keys off either the open attempt -- whose edge id
     * came from {@code determineNextCycleToComplete()}, so it is a key by construction -- or an
     * explicit {@code containsKey} guard. But that safety rests on a fact stated nowhere: a
     * populated map can never reach a robot standing somewhere else, because a root never gets
     * dropped back to unassigned. It happens to be true (all three vacate paths are gated on
     * {@code role == cycleBuilder}), and it is exactly the kind of global argument that a later
     * change breaks without anything noticing. So the guard is here to make the next violation a
     * log line instead of a root that quietly never finishes.
     *
     * <p>The promotion check stays. Writing a corner and then asking whether every corner is now
     * settled is what this method is <em>for</em>; it is not a side effect.
     *
     * @return the status this replaced, or null if the edge is not one of this robot's corners
     */
    public CycleStatus setCycleStatusOf(int outgoingEdgeID, CycleStatus status) {
        if (!completedCycles.containsKey(outgoingEdgeID)) {
            log("-> ANOMALY: asked to mark edge " + outgoingEdgeID + " as " + status
                    + ", but that is not a corner of this robot's site. Tracked corners: "
                    + completedCycles.keySet() + ". Refusing rather than inventing one.");
            return null;
        }
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
        forgetTrackedCorners();
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
        forgetTrackedCorners();
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
     * <p>Links go for the same reason as in {@link #resetToUnassigned()}: accepting a
     * <em>first</em> assignment means moving to a different lattice site, so every adjacency the
     * old links recorded stops being true. This is not the promotion case -- a promotion leaves
     * the robot where it is and keeps its links -- and it is not the second-face case either: a
     * settled robot taking on another walk goes through {@link #relayAlongTuple} and never comes
     * here.
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
        forgetTrackedCorners();
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
     * <p>The two have to move together. A queued assignment left behind after its link is gone
     * would be read as a fresh offer -- and this robot would accept, from a parent it has just
     * walked away from, a site it no longer occupies.
     */
    private void dropEveryObligationAndItsCustody() {
        for (FaceObligation obligation : obligations.drainForVacate()) {
            takeCustody(obligation);
        }
    }

    /**
     * Gives up the corner bookkeeping for a lattice site this robot is leaving.
     *
     * <p>Called from the three vacate paths and from nowhere else. A robot dropped out of the
     * formation does not own the corners of a site it has walked away from, and keeping them is not
     * merely untidy:
     *
     * <ul>
     *   <li>{@code acceptPromotion} asks "do I have a corner map yet?" as
     *       {@code !completedCycles.isEmpty()} and skips {@link #initializeEdgeMap()} when the
     *       answer is yes, so a robot that kept a stale map would be handed rootship still
     *       tracking the corners of the site it left.</li>
     *   <li>{@code CommsSnapshot.tracksCycles()} asks the same thing for the tick log, so the log
     *       would report corner statuses for a robot that owns no corners.</li>
     *   <li>{@link #hasFailed()} would answer about a site nobody is standing on.</li>
     * </ul>
     *
     * <p>Note that the first of those is no longer the "am I already a root?" question -- that is
     * {@code role == CycleRole.root} now, since builders carry a corner map too and the map can no
     * longer stand in for the role.</p>
     *
     * <p>Deliberately <strong>not</strong> called from {@link #resetToRoot()} -- that is the
     * promotion path, and a promoted robot has not moved, so its corners and its record of what it
     * has already announced are both still true. Nor from {@code promoteSelfToStable()}, for the
     * same reason.
     *
     * <p>{@code announcedCorners} goes with the map because it is scoped to it: it records which of
     * <em>these</em> corners have already been handed on, and outliving them would suppress a real
     * announcement later.
     */
    private void forgetTrackedCorners() {
        completedCycles.clear();
        announcedCorners.clear();
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
